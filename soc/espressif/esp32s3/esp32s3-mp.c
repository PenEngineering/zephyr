/*
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/zsr.h>

#include <soc.h>
#include <esp_log.h>
#include <esp_cpu.h>
#include "esp_rom_uart.h"

#include "esp_mcuboot_image.h"
#include "esp_memory_utils.h"
#include "hw_init.h"

#define TAG "amp"

/* AMP support */
#ifdef CONFIG_SOC_ENABLE_APPCPU

#include "bootloader_flash_priv.h"

#define sys_mmap   bootloader_mmap
#define sys_munmap bootloader_munmap

void esp_appcpu_start(void *entry_point)
{
	esp_cpu_unstall(1);

	if (!REG_GET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN)) {
		REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN);
		REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RUNSTALL);
		REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETTING);
		REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETTING);
	}

	esp_rom_ets_set_appcpu_boot_addr((void *)entry_point);

	esp_cpu_reset(1);
}

/* SMP support — ported from esp32/esp32-mp.c (the only Zephyr SoC in this
 * tree with a working CONFIG_SMP block; esp32s3 had none). DPORT_* renamed
 * to SYSTEM_* (confirmed 1:1 in soc/system_reg.h — same bit layout, base
 * address DR_REG_SYSTEM_BASE instead of DPORT). IPI interrupt sources use
 * numeric ETS_FROM_CPU_INTR0/1_SOURCE directly instead of DT_NODELABEL(ipi0/
 * ipi1) — no such devicetree nodes exist for this SoC/board, and
 * esp_intr_alloc() takes a raw source number anyway, so DT was never load-
 * bearing here. No per-core cache enable call (unlike esp32-mp.c's Cache_
 * Flush(1)/Cache_Read_Enable(1)) — confirmed via TRM: ESP32-S3 has ONE
 * shared, arbitrated ICache/DCache for both cores, not per-core caches, so
 * there is nothing to enable per-core. UNVERIFIED ON HARDWARE: the ROM-UART
 * "smp_log" timing dependency documented in esp32-mp.c as required-but-
 * unexplained for reliable APPCPU start is specific to classic ESP32 ROM;
 * kept here defensively (same reasoning, same risk) since removing it is
 * unverified either way and it is cheap to keep. */
#ifdef CONFIG_SMP

#include <ipi.h>
#include <zephyr/dt-bindings/interrupt-controller/esp32s3-xtensa-intmux.h>
#include "esp_intr_alloc.h"
#include "soc/periph_defs.h"

#ifndef CONFIG_SOC_ESP32S3_PROCPU
static struct k_spinlock loglock;
#endif

struct cpustart_rec {
	int cpu;
	arch_cpustart_t fn;
	char *stack_top;
	void *arg;
	int vecbase;
	volatile int *alive;
	uint32_t ccount0;
};

volatile struct cpustart_rec *start_rec;
static void *appcpu_top;
static bool cpus_active[CONFIG_MP_MAX_NUM_CPUS];
static struct k_spinlock loglock;

/* See esp32-mp.c smp_log() for the full history: at least one board hangs
 * spuriously on APPCPU start without this, and the cause was never root-
 * caused even there. Left in place unverified on S3 — cheap insurance,
 * remove only after confirming boot is reliable without it. */
void smp_log(const char *msg)
{
	k_spinlock_key_t key = k_spin_lock(&loglock);

	while (*msg) {
		esp_rom_uart_tx_one_char(*msg++);
	}
	esp_rom_uart_tx_one_char('\r');
	esp_rom_uart_tx_one_char('\n');

	k_spin_unlock(&loglock, key);
}

static void appcpu_entry2(void)
{
	volatile int ps, ie;

	/* CCOUNT is per-core on Xtensa; xtensa_sys_timer.c's shared
	 * last_count assumes one continuous counter. Sync CPU1's CCOUNT to
	 * CPU0's snapshot from handoff so the tick ISR's unsigned subtraction
	 * never underflows.
	 */
	__asm__ volatile("wsr.CCOUNT %0; rsync" : : "r"(start_rec->ccount0));

	__asm__ volatile("rsr.PS %0" : "=r"(ps));
	ps &= ~(XCHAL_PS_EXCM_MASK | XCHAL_PS_INTLEVEL_MASK);
	__asm__ volatile("wsr.PS %0" : : "r"(ps));

	ie = 0;
	__asm__ volatile("wsr.INTENABLE %0" : : "r"(ie));
	__asm__ volatile("wsr.VECBASE %0" : : "r"(start_rec->vecbase));
	__asm__ volatile("rsync");

	_cpu_t *cpu = &_kernel.cpus[1];

	/* Must be ZSR_CPU (resolves to MISC1 in this build, see loader.c's
	 * matching fix for CPU0), not hardcoded MISC0 — MISC0 is
	 * ZSR_A0SAVE, a different slot. */
	__asm__ volatile("wsr." STRINGIFY(ZSR_CPU) " %0" : : "r"(cpu));

	smp_log("ESP32S3: APPCPU running");

	/* Snapshot fn/arg before signaling alive: start_rec points at a
	 * stack-local in CPU0's arch_cpu_start(), which returns and reuses
	 * that stack the instant alive_flag is observed set. Touching
	 * start_rec after the signal races CPU0 tearing it down.
	 */
	arch_cpustart_t fn = start_rec->fn;
	void *arg = start_rec->arg;

	*start_rec->alive = 1;
	fn(arg);
}

void z_appcpu_stack_switch(void *stack, void *entry);
__asm__("\n"
	".align 4"		"\n"
	"z_appcpu_stack_switch:"	"\n\t"
	"entry a1, 16"		"\n\t"
	"addi a1, a2, 16"	"\n\t"
	/* entry already set WINDOWSTART's bit for this window as its own
	 * defined side effect (measured live: WINDOWBASE=5 here, so bit 5).
	 * Writing WINDOWSTART afterward — to 0 (original) or any hardcoded
	 * bit — clobbers that correct value instead of a real bit, leaving
	 * this window's call8 overflow-chain linkage never legitimately
	 * populated. A later _WindowOverflow8 walking back through it then
	 * reads stale .noinit stack content as a store address
	 * (StoreProhibited). Just don't touch WINDOWSTART here.
	 */
	"rsr.PS a0"		"\n\t"
	"movi a2, 0xfffcffff"	"\n\t"
	"and a0, a0, a2"	"\n\t"
	"wsr.PS a0"		"\n\t"
	"rsync"			"\n\t"
	"movi a0, 0"		"\n\t"
	"jx a3"			"\n\t");

static void appcpu_entry1(void)
{
	z_appcpu_stack_switch(appcpu_top, appcpu_entry2);
}

IRAM_ATTR static void esp_crosscore_isr(void *arg)
{
	ARG_UNUSED(arg);

	z_sched_ipi();

	const int core_id = esp_core_id();

	if (core_id == 0) {
		WRITE_PERI_REG(SYSTEM_CPU_INTR_FROM_CPU_0_REG, 0);
	} else {
		WRITE_PERI_REG(SYSTEM_CPU_INTR_FROM_CPU_1_REG, 0);
	}
}

void arch_cpu_start(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	volatile struct cpustart_rec sr;
	int vb;
	volatile int alive_flag;
	uint32_t ccount0;

	/* Raw ROM UART, no printk/spinlock — printk produced nothing at this
	 * boot stage (deferred log backend likely not initialized yet), but
	 * MCUboot's own banner used exactly this path and DID reach the
	 * console, so it should work this early too. */
	{
		const char *m = "AKIRA: arch_cpu_start entered\r\n";
		while (*m) {
			esp_rom_uart_tx_one_char(*m++);
		}
	}

	__ASSERT(cpu_num == 1, "ESP32-S3 supports only two CPUs");

	__asm__ volatile("rsr.VECBASE %0\n\t" : "=r"(vb));

	alive_flag = 0;

	sr.cpu = cpu_num;
	sr.fn = fn;
	sr.stack_top = K_KERNEL_STACK_BUFFER(stack) + sz;
	sr.arg = arg;
	sr.vecbase = vb;
	sr.alive = &alive_flag;

	appcpu_top = K_KERNEL_STACK_BUFFER(stack) + sz;

	start_rec = &sr;

	printk("AKIRA: arch_cpu_start before esp_appcpu_start\n");
	__asm__ volatile("rsr.CCOUNT %0" : "=r"(ccount0));
	sr.ccount0 = ccount0;
	esp_appcpu_start(appcpu_entry1);
	printk("AKIRA: arch_cpu_start after esp_appcpu_start, waiting for alive\n");

	uint32_t spins = 0;
	while (!alive_flag) {
		spins++;
		if ((spins & 0xFFFFF) == 0) {
			printk("AKIRA: still waiting for alive_flag, spins=%u\n", spins);
		}
	}
	printk("AKIRA: alive_flag set after %u spins\n", spins);

	cpus_active[0] = true;
	cpus_active[cpu_num] = true;

	esp_intr_alloc(ETS_FROM_CPU_INTR0_SOURCE,
		ESP_PRIO_TO_FLAGS(IRQ_DEFAULT_PRIORITY) | ESP_INTR_FLAG_IRAM,
		esp_crosscore_isr, NULL, NULL);

	esp_intr_alloc(ETS_FROM_CPU_INTR1_SOURCE,
		ESP_PRIO_TO_FLAGS(IRQ_DEFAULT_PRIORITY) | ESP_INTR_FLAG_IRAM,
		esp_crosscore_isr, NULL, NULL);

	smp_log("ESP32S3: APPCPU initialized");
}

void arch_sched_directed_ipi(uint32_t cpu_bitmap)
{
	const int core_id = esp_core_id();

	ARG_UNUSED(cpu_bitmap);

	if (core_id == 0) {
		WRITE_PERI_REG(SYSTEM_CPU_INTR_FROM_CPU_0_REG, SYSTEM_CPU_INTR_FROM_CPU_0);
	} else {
		WRITE_PERI_REG(SYSTEM_CPU_INTR_FROM_CPU_1_REG, SYSTEM_CPU_INTR_FROM_CPU_1);
	}
}

void arch_sched_broadcast_ipi(void)
{
	arch_sched_directed_ipi(IPI_ALL_CPUS_MASK);
}

IRAM_ATTR bool arch_cpu_active(int cpu_num)
{
	return cpus_active[cpu_num];
}
#endif /* CONFIG_SMP */

static int load_segment(uint32_t src_addr, uint32_t src_len, uint32_t dst_addr)
{
	const uint32_t *data = (const uint32_t *)sys_mmap(src_addr, src_len);

	if (!data) {
		ESP_EARLY_LOGE(TAG, "%s: mmap failed", __func__);
		return -1;
	}

	volatile uint32_t *dst = (volatile uint32_t *)dst_addr;

	for (int i = 0; i < src_len / 4; i++) {
		dst[i] = data[i];
	}

	sys_munmap(data);

	return 0;
}

/* This whole chain (image_load/_stop/_start/init) only makes sense with a
 * real slot0_appcpu_partition — see CONFIG_ESP_APPCPU_AUTOLOAD_IMAGE. */
#ifdef CONFIG_ESP_APPCPU_AUTOLOAD_IMAGE

int IRAM_ATTR esp_appcpu_image_load(unsigned int hdr_offset, unsigned int *entry_addr)
{
	const uint32_t fa_offset = FIXED_PARTITION_OFFSET(slot0_appcpu_partition);
	const uint32_t fa_size = FIXED_PARTITION_SIZE(slot0_appcpu_partition);
	const uint8_t fa_id = FIXED_PARTITION_ID(slot0_appcpu_partition);

	if (entry_addr == NULL) {
		ESP_EARLY_LOGE(TAG, "Can't return the entry address. Aborting!");
		abort();
		return -1;
	}

	uint32_t mcuboot_header[8] = {0};
	esp_image_load_header_t image_header = {0};

	const uint32_t *data = (const uint32_t *)sys_mmap(fa_offset, 0x80);

	memcpy((void *)&mcuboot_header, data, sizeof(mcuboot_header));
	memcpy((void *)&image_header, data + (hdr_offset / sizeof(uint32_t)),
	       sizeof(esp_image_load_header_t));

	sys_munmap(data);

	if (image_header.header_magic == ESP_LOAD_HEADER_MAGIC) {
		ESP_EARLY_LOGI(TAG,
			"APPCPU image, area id: %d, offset: 0x%x, hdr.off: 0x%x, size: %d kB",
			fa_id, fa_offset, hdr_offset, fa_size / 1024);
	} else if ((image_header.header_magic & 0xff) == 0xE9) {
		ESP_EARLY_LOGE(TAG, "ESP image format is not supported");
		abort();
	} else {
		ESP_EARLY_LOGE(TAG, "Unknown or empty image detected. Aborting!");
		abort();
	}

	if (!esp_ptr_in_iram((void *)image_header.iram_dest_addr) ||
	    !esp_ptr_in_iram((void *)(image_header.iram_dest_addr + image_header.iram_size))) {
		ESP_EARLY_LOGE(TAG, "IRAM region in load header is not valid. Aborting");
		abort();
	}

	if (!esp_ptr_in_dram((void *)image_header.dram_dest_addr) ||
	    !esp_ptr_in_dram((void *)(image_header.dram_dest_addr + image_header.dram_size))) {
		ESP_EARLY_LOGE(TAG, "DRAM region in load header is not valid. Aborting");
		abort();
	}

	if (!esp_ptr_in_iram((void *)image_header.entry_addr)) {
		ESP_EARLY_LOGE(TAG, "Application entry point (%xh) is not in IRAM. Aborting",
			   image_header.entry_addr);
		abort();
	}

	ESP_EARLY_LOGI(TAG, "IRAM segment: paddr=%08xh, vaddr=%08xh, size=%05xh (%6d) load",
		   (fa_offset + image_header.iram_flash_offset), image_header.iram_dest_addr,
		   image_header.iram_size, image_header.iram_size);

	load_segment(fa_offset + image_header.iram_flash_offset, image_header.iram_size,
		     image_header.iram_dest_addr);

	ESP_EARLY_LOGI(TAG, "DRAM segment: paddr=%08xh, vaddr=%08xh, size=%05xh (%6d) load",
		   (fa_offset + image_header.dram_flash_offset), image_header.dram_dest_addr,
		   image_header.dram_size, image_header.dram_size);

	load_segment(fa_offset + image_header.dram_flash_offset, image_header.dram_size,
		     image_header.dram_dest_addr);

	ESP_EARLY_LOGI(TAG, "IROM segment: paddr=%08xh, vaddr=%08xh, size=%05xh (%6d) map",
		   (fa_offset + image_header.irom_flash_offset), image_header.irom_map_addr,
		   image_header.irom_size, image_header.irom_size);

	ESP_EARLY_LOGI(TAG, "DROM segment: paddr=%08xh, vaddr=%08xh, size=%05xh (%6d) map",
		   (fa_offset + image_header.drom_flash_offset), image_header.drom_map_addr,
		   image_header.drom_size, image_header.drom_size);

	struct rom_segments rom = {
		image_header.drom_map_addr,
		image_header.drom_flash_offset + fa_offset,
		image_header.drom_size,
		image_header.irom_map_addr,
		image_header.irom_flash_offset + fa_offset,
		image_header.irom_size,
	};

	map_rom_segments(1, &rom);

	ESP_EARLY_LOGI(TAG, "Application start=%xh\n\n", image_header.entry_addr);
	esp_rom_uart_tx_wait_idle(0);

	assert(entry_addr != NULL);
	*entry_addr = image_header.entry_addr;

	return 0;
}

void esp_appcpu_image_stop(void)
{
	esp_cpu_stall(1);
}

void esp_appcpu_image_start(unsigned int hdr_offset)
{
	static int started;
	unsigned int entry_addr = 0;

	if (started) {
		printk("APPCPU already started.\r\n");
		return;
	}

	/* Input image meta header, output appcpu entry point */
	esp_appcpu_image_load(hdr_offset, &entry_addr);

	esp_appcpu_start((void *)entry_addr);
}

int esp_appcpu_init(void)
{
	/* Load APPCPU image using image header offset
	 * (skipping the MCUBoot header)
	 */
	esp_appcpu_image_start(0x20);

	return 0;
}

/* AkiraEar: this auto-init loads a SEPARATE appcpu image from a
 * slot0_appcpu_partition flash partition — the normal Zephyr AMP sysbuild
 * flow. We don't use that flow (single-image AMP/SMP via esp_appcpu_start()/
 * arch_cpu_start() with a raw IRAM function pointer already linked into THIS
 * binary), and without the partition this SYS_INIT hits an invalid image
 * header and abort()s at boot. Gated off via CONFIG_ESP_APPCPU_AUTOLOAD_IMAGE
 * (see common/Kconfig.amp) instead of deleting it, so the stock sysbuild flow
 * still works for anyone who wants it. */
#if !defined(CONFIG_MCUBOOT)
extern int esp_appcpu_init(void);
SYS_INIT(esp_appcpu_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif

#endif /* CONFIG_ESP_APPCPU_AUTOLOAD_IMAGE */

#endif /* CONFIG_SOC_ENABLE_APPCPU */
