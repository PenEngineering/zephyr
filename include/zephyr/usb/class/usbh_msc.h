/*
 * Copyright (c) 2026 AkiraOS Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB host Mass Storage class notifications
 *
 * The driver registers/unregisters a disk_access disk named "USB" as
 * devices connect and disconnect. This callback lets application code
 * (filesystem mount logic) react without polling disk_access_status().
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBH_MSC_H
#define ZEPHYR_INCLUDE_USB_CLASS_USBH_MSC_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Name of the disk_access disk registered by this driver. Must NOT include
 * a trailing ':' — FatFs's get_ldnumber() volume-ID match fails if the
 * VolumeStr[] entry has one (see CONFIG_FS_FATFS_CUSTOM_MOUNT_POINTS,
 * which must use the same bare name).
 */
#define USBH_MSC_DISK_NAME "USB"

/**
 * @brief USB host MSC connection event callback
 *
 * @param connected true once the disk is registered and ready for
 *                  disk_access calls, false once it has been removed.
 * @param user_data Opaque pointer passed to usbh_msc_register_callback().
 */
typedef void (*usbh_msc_event_cb_t)(bool connected, void *user_data);

/**
 * @brief Register for USB host MSC connect/disconnect notifications
 *
 * Only one callback is supported; a later call replaces the former.
 *
 * @param cb Callback function, or NULL to unregister.
 * @param user_data Opaque pointer passed back to the callback.
 */
void usbh_msc_register_callback(usbh_msc_event_cb_t cb, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBH_MSC_H */
