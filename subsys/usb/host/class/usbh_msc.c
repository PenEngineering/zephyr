/*
 * Copyright (c) 2026 AkiraOS Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB host Mass Storage class driver
 *
 * Speaks Bulk-Only Transport (USB Mass Storage Class - Bulk Only Transport,
 * Revision 1.0) and a minimal SCSI Primary/Block command set (TEST UNIT
 * READY, READ CAPACITY(10), READ(10), WRITE(10)) to a connected device, and
 * exposes it as a disk_access disk so a filesystem can be mounted on it.
 *
 * Single device, single LUN (LUN 0) only.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usbh_msc.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <errno.h>

#include <usbh_class.h>
#include <usbh_class_api.h>
#include <usbh_device.h>

LOG_MODULE_REGISTER(usbh_msc, CONFIG_USBH_MASS_STORAGE_CLASS_LOG_LEVEL);

#define MSC_SUBCLASS_SCSI	0x06
#define MSC_PROTOCOL_BBB	0x50

#define CBW_SIGNATURE		0x43425355
#define CSW_SIGNATURE		0x53425355
#define CBW_LEN			31
#define CSW_LEN			13

#define SCSI_TEST_UNIT_READY	0x00
#define SCSI_READ_CAPACITY_10	0x25
#define SCSI_READ_10		0x28
#define SCSI_WRITE_10		0x2A

#define MSC_XFER_TIMEOUT_MS	5000
#define MSC_TUR_RETRIES		10
#define MSC_TUR_RETRY_DELAY_MS	200

struct msc_data {
	struct usb_device *udev;
	uint8_t iface;
	uint8_t ep_in;
	uint8_t ep_out;
	uint32_t block_size;
	uint32_t block_count;
	uint32_t tag;
	struct k_sem xfer_done;
	bool disk_registered;
};

/* Single device instance: this driver targets one MSC device at a time. */
static struct msc_data msc;
static usbh_msc_event_cb_t msc_event_cb;
static void *msc_event_user;

void usbh_msc_register_callback(usbh_msc_event_cb_t cb, void *user_data)
{
	msc_event_cb = cb;
	msc_event_user = user_data;
}

static int msc_xfer_cb(struct usb_device *const udev, struct uhc_transfer *const xfer)
{
	ARG_UNUSED(udev);
	ARG_UNUSED(xfer);
	k_sem_give(&msc.xfer_done);

	return 0;
}

/* Submit one bulk transfer and block until it completes or times out. */
static int msc_bulk_xfer(uint8_t ep, struct net_buf *buf)
{
	struct uhc_transfer *xfer;
	int ret;

	xfer = usbh_xfer_alloc(msc.udev, ep, msc_xfer_cb, NULL);
	if (xfer == NULL) {
		return -ENOMEM;
	}

	ret = usbh_xfer_buf_add(msc.udev, xfer, buf);
	if (ret != 0) {
		usbh_xfer_free(msc.udev, xfer);
		return ret;
	}

	ret = usbh_xfer_enqueue(msc.udev, xfer);
	if (ret != 0) {
		usbh_xfer_free(msc.udev, xfer);
		return ret;
	}

	if (k_sem_take(&msc.xfer_done, K_MSEC(MSC_XFER_TIMEOUT_MS)) != 0) {
		usbh_xfer_dequeue(msc.udev, xfer);
		usbh_xfer_free(msc.udev, xfer);
		return -ETIMEDOUT;
	}

	ret = xfer->err;
	usbh_xfer_free(msc.udev, xfer);

	return ret;
}

/*
 * Run one Bulk-Only Transport command: CBW out, optional data stage,
 * CSW in. `data` is filled (data_in) or read (!data_in) via `data_len`
 * bytes; pass data_len 0 and data NULL for commands with no data stage.
 */
static int msc_bbb_cmd(const uint8_t *cdb, uint8_t cdb_len,
			void *data, uint32_t data_len, bool data_in)
{
	struct net_buf *buf;
	uint8_t csw[CSW_LEN];
	uint32_t sig, tag, residue;
	uint8_t status;
	int ret;

	msc.tag++;

	buf = usbh_xfer_buf_alloc(msc.udev, CBW_LEN);
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_add_le32(buf, CBW_SIGNATURE);
	net_buf_add_le32(buf, msc.tag);
	net_buf_add_le32(buf, data_len);
	net_buf_add_u8(buf, data_len == 0 ? 0 : (data_in ? 0x80 : 0x00));
	net_buf_add_u8(buf, 0); /* LUN 0 */
	net_buf_add_u8(buf, cdb_len);
	net_buf_add_mem(buf, cdb, cdb_len);
	for (uint8_t i = cdb_len; i < 16; i++) {
		net_buf_add_u8(buf, 0);
	}

	ret = msc_bulk_xfer(msc.ep_out, buf);
	net_buf_unref(buf);
	if (ret != 0) {
		LOG_ERR("CBW send failed: %d", ret);
		return ret;
	}

	if (data_len != 0) {
		if (data_in) {
			buf = usbh_xfer_buf_alloc(msc.udev, data_len);
		} else {
			buf = usbh_xfer_buf_alloc(msc.udev, data_len);
			if (buf != NULL) {
				net_buf_add_mem(buf, data, data_len);
			}
		}
		if (buf == NULL) {
			return -ENOMEM;
		}

		ret = msc_bulk_xfer(data_in ? msc.ep_in : msc.ep_out, buf);
		if (ret == 0 && data_in) {
			memcpy(data, buf->data, MIN(data_len, buf->len));
		}
		net_buf_unref(buf);
		if (ret != 0) {
			LOG_ERR("Data stage failed: %d", ret);
			return ret;
		}
	}

	buf = usbh_xfer_buf_alloc(msc.udev, CSW_LEN);
	if (buf == NULL) {
		return -ENOMEM;
	}

	ret = msc_bulk_xfer(msc.ep_in, buf);
	if (ret != 0) {
		net_buf_unref(buf);
		LOG_ERR("CSW read failed: %d", ret);
		return ret;
	}

	if (buf->len < CSW_LEN) {
		net_buf_unref(buf);
		LOG_ERR("Short CSW: %u bytes", buf->len);
		return -EIO;
	}
	memcpy(csw, buf->data, CSW_LEN);
	net_buf_unref(buf);

	sig = sys_get_le32(&csw[0]);
	tag = sys_get_le32(&csw[4]);
	residue = sys_get_le32(&csw[8]);
	status = csw[12];
	ARG_UNUSED(residue);

	if (sig != CSW_SIGNATURE || tag != msc.tag) {
		LOG_ERR("Bad CSW signature/tag");
		return -EIO;
	}

	if (status != 0) {
		LOG_DBG("SCSI command failed, CSW status %u", status);
		return -EIO;
	}

	return 0;
}

static int msc_test_unit_ready(void)
{
	const uint8_t cdb[6] = {SCSI_TEST_UNIT_READY, 0, 0, 0, 0, 0};

	return msc_bbb_cmd(cdb, sizeof(cdb), NULL, 0, false);
}

static int msc_read_capacity(uint32_t *last_lba, uint32_t *block_size)
{
	const uint8_t cdb[10] = {SCSI_READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t data[8];
	int ret;

	ret = msc_bbb_cmd(cdb, sizeof(cdb), data, sizeof(data), true);
	if (ret != 0) {
		return ret;
	}

	*last_lba = sys_get_be32(&data[0]);
	*block_size = sys_get_be32(&data[4]);

	return 0;
}

static int msc_read10(uint32_t lba, uint16_t num_blocks, void *data)
{
	uint8_t cdb[10] = {SCSI_READ_10, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	sys_put_be32(lba, &cdb[2]);
	sys_put_be16(num_blocks, &cdb[7]);

	return msc_bbb_cmd(cdb, sizeof(cdb), data, (uint32_t)num_blocks * msc.block_size, true);
}

static int msc_write10(uint32_t lba, uint16_t num_blocks, const void *data)
{
	uint8_t cdb[10] = {SCSI_WRITE_10, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	sys_put_be32(lba, &cdb[2]);
	sys_put_be16(num_blocks, &cdb[7]);

	return msc_bbb_cmd(cdb, sizeof(cdb), (void *)data,
			    (uint32_t)num_blocks * msc.block_size, false);
}

/* --- disk_access bridge --- */

static int disk_msc_init(struct disk_info *disk)
{
	ARG_UNUSED(disk);
	return 0;
}

static int disk_msc_status(struct disk_info *disk)
{
	ARG_UNUSED(disk);
	return msc.udev != NULL ? DISK_STATUS_OK : DISK_STATUS_NOMEDIA;
}

static int disk_msc_read(struct disk_info *disk, uint8_t *data_buf,
			 uint32_t start_sector, uint32_t num_sector)
{
	ARG_UNUSED(disk);

	if (msc.udev == NULL) {
		return -ENODEV;
	}

	/* Split into chunks so one heap-backed net_buf per request stays small. */
	while (num_sector > 0) {
		uint16_t chunk = (uint16_t)MIN(num_sector, 32);
		int ret = msc_read10(start_sector, chunk, data_buf);

		if (ret != 0) {
			return ret;
		}

		data_buf += (size_t)chunk * msc.block_size;
		start_sector += chunk;
		num_sector -= chunk;
	}

	return 0;
}

static int disk_msc_write(struct disk_info *disk, const uint8_t *data_buf,
			  uint32_t start_sector, uint32_t num_sector)
{
	ARG_UNUSED(disk);

	if (msc.udev == NULL) {
		return -ENODEV;
	}

	/* Split into chunks so one heap-backed net_buf per request stays small. */
	while (num_sector > 0) {
		uint16_t chunk = (uint16_t)MIN(num_sector, 32);
		int ret = msc_write10(start_sector, chunk, data_buf);

		if (ret != 0) {
			return ret;
		}

		data_buf += (size_t)chunk * msc.block_size;
		start_sector += chunk;
		num_sector -= chunk;
	}

	return 0;
}

static int disk_msc_ioctl(struct disk_info *disk, uint8_t cmd, void *buf)
{
	ARG_UNUSED(disk);

	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t *)buf = msc.block_count;
		return 0;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t *)buf = msc.block_size;
		return 0;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(uint32_t *)buf = 1;
		return 0;
	case DISK_IOCTL_CTRL_SYNC:
	case DISK_IOCTL_CTRL_INIT:
	case DISK_IOCTL_CTRL_DEINIT:
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct disk_operations msc_disk_ops = {
	.init = disk_msc_init,
	.status = disk_msc_status,
	.read = disk_msc_read,
	.write = disk_msc_write,
	.ioctl = disk_msc_ioctl,
};

static struct disk_info msc_disk = {
	.name = USBH_MSC_DISK_NAME,
	.ops = &msc_disk_ops,
};

/* --- USB host class API --- */

static int msc_find_bulk_endpoints(struct usb_device *udev)
{
	msc.ep_in = 0;
	msc.ep_out = 0;

	for (uint8_t i = 1; i < 16; i++) {
		struct usb_ep_descriptor *desc = udev->ep_in[i].desc;

		if (desc != NULL &&
		    (desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_BULK) {
			msc.ep_in = desc->bEndpointAddress;
			break;
		}
	}

	for (uint8_t i = 1; i < 16; i++) {
		struct usb_ep_descriptor *desc = udev->ep_out[i].desc;

		if (desc != NULL &&
		    (desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_BULK) {
			msc.ep_out = desc->bEndpointAddress;
			break;
		}
	}

	if (msc.ep_in == 0 || msc.ep_out == 0) {
		LOG_ERR("Bulk IN/OUT endpoints not found");
		return -ENODEV;
	}

	return 0;
}

static int usbh_msc_probe(struct usbh_class_data *const c_data,
			  struct usb_device *const udev,
			  const uint8_t iface)
{
	ARG_UNUSED(c_data);
	int ret;

	if (msc.udev != NULL) {
		LOG_WRN("MSC device already bound, ignoring additional device");
		return -EBUSY;
	}

	msc.udev = udev;
	msc.iface = iface;
	msc.tag = 0;
	k_sem_init(&msc.xfer_done, 0, 1);

	ret = msc_find_bulk_endpoints(udev);
	if (ret != 0) {
		goto fail;
	}

	/* Some drives need a moment to spin up / report ready after reset. */
	for (int i = 0; i < MSC_TUR_RETRIES; i++) {
		ret = msc_test_unit_ready();
		if (ret == 0) {
			break;
		}
		k_sleep(K_MSEC(MSC_TUR_RETRY_DELAY_MS));
	}
	if (ret != 0) {
		LOG_ERR("Device not ready after %d attempts", MSC_TUR_RETRIES);
		goto fail;
	}

	uint32_t last_lba;

	ret = msc_read_capacity(&last_lba, &msc.block_size);
	if (ret != 0) {
		LOG_ERR("READ CAPACITY(10) failed: %d", ret);
		goto fail;
	}
	msc.block_count = last_lba + 1;

	LOG_INF("MSC device ready: %u blocks x %u bytes (%u MiB)",
		msc.block_count, msc.block_size,
		(uint32_t)(((uint64_t)msc.block_count * msc.block_size) / (1024 * 1024)));

	ret = disk_access_register(&msc_disk);
	if (ret != 0) {
		LOG_ERR("Failed to register disk: %d", ret);
		goto fail;
	}
	msc.disk_registered = true;

	if (msc_event_cb != NULL) {
		msc_event_cb(true, msc_event_user);
	}

	return 0;

fail:
	msc.udev = NULL;
	return ret;
}

static int usbh_msc_removed(struct usbh_class_data *const c_data)
{
	ARG_UNUSED(c_data);

	if (msc_event_cb != NULL) {
		msc_event_cb(false, msc_event_user);
	}

	if (msc.disk_registered) {
		disk_access_unregister(&msc_disk);
		msc.disk_registered = false;
	}

	msc.udev = NULL;

	return 0;
}

static int usbh_msc_init(struct usbh_class_data *const c_data)
{
	ARG_UNUSED(c_data);
	return 0;
}

static struct usbh_class_api usbh_msc_class_api = {
	.init = usbh_msc_init,
	.probe = usbh_msc_probe,
	.removed = usbh_msc_removed,
};

static struct usbh_class_filter usbh_msc_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_MASS_STORAGE,
		.sub = MSC_SUBCLASS_SCSI,
		.proto = MSC_PROTOCOL_BBB,
	},
	{0},
};

USBH_DEFINE_CLASS(usbh_msc, &usbh_msc_class_api, NULL, usbh_msc_filters);
