// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Goodix GTX8 Touchscreens
 *
 * Copyright (c) 2019 - 2020 Goodix, Inc.
 * Copyright (C) 2023 Linaro Ltd.
 * Copyright (c) 2025 Jens Reidel <adrian@mainlining.org>
 *
 * Based on gtx8_driver_linux vendor driver and goodix_berlin kernel driver.
 *
 * The driver currently only supports Normandy / Yellowstone ICs.
 * Pen support is also missing.
 */
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/jiffies.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>

#include "goodix_gtx8.h"

static const struct regmap_config goodix_gtx8_regmap_conf = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_raw_read = I2C_MAX_TRANSFER_SIZE,
	.max_raw_write = I2C_MAX_TRANSFER_SIZE,
};

/* vendor & product left unassigned here, should probably be updated from fw info */
static const struct input_id goodix_gtx8_input_id = {
	.bustype = BUS_I2C,
};

#define GOODIX_GTX8_CFG_BIN_VERSION_START	5
#define GOODIX_GTX8_CFG_BIN_HEAD_RESERVED_LEN	6
#define GOODIX_GTX8_CFG_OFFSET_LEN		2
#define GOODIX_GTX8_CFG_IC_TYPE_NAME_MAX_LEN	15
#define GOODIX_GTX8_CFG_BLOCK_PID_LEN		8
#define GOODIX_GTX8_CFG_BLOCK_VID_LEN		8
#define GOODIX_GTX8_CFG_BLOCK_FW_MASK_LEN	9
#define GOODIX_GTX8_CFG_BLOCK_FW_PATCH_LEN	4
#define GOODIX_GTX8_CFG_BLOCK_RESERVED_LEN	9

#define GOODIX_GTX8_CFG_TYPE_NORMAL		0x01

#define GOODIX_GTX8_CMD_START_SEND_CFG		0x80
#define GOODIX_GTX8_CMD_SEND_CFG_PREPARE_OK	0x82
#define GOODIX_GTX8_CMD_END_SEND_CFG		0x83
#define GOODIX_GTX8_CMD_END_SEND_CFG_YS		0x7d
#define GOODIX_GTX8_CMD_CFG_ERR			0x7e
#define GOODIX_GTX8_CMD_CFG_OK			0x7f
#define GOODIX_GTX8_CMD_REG_READY		0xff

#define GOODIX_GTX8_CFG_REPLY_DATA_EQU		0x07
#define GOODIX_GTX8_WAIT_CMD_FREE_RETRY		10
#define GOODIX_GTX8_WAIT_CFG_READY_RETRY	30
#define GOODIX_GTX8_REG_ADDR_SIZE		2
#define GOODIX_GTX8_POLL_INTERVAL_MIN_MS	20
#define GOODIX_GTX8_POLL_INTERVAL_MAX_MS	1000
#define GOODIX_GTX8_POLL_MAX_ERRORS		3
#define GOODIX_GTX8_WAKE_POLL_INTERVAL_MS	150
#define GOODIX_GTX8_WAKE_POLL_ATTEMPTS		2000
#define GOODIX_GTX8_WAKE_ADDR			0x38

static const u8 goodix_gtx8_wake_regs[] = {
	0xa3, 0x9f, 0xa8, 0x00, 0x01,
};

struct goodix_gtx8_cfg_pkg_reg {
	__le16 addr;
	u8 reserved1;
	u8 reserved2;
} __packed;

struct goodix_gtx8_cfg_pkg_const_info {
	__le32 pkg_len;
	u8 ic_type[GOODIX_GTX8_CFG_IC_TYPE_NAME_MAX_LEN];
	u8 cfg_type;
	u8 sensor_id;
	u8 hw_pid[GOODIX_GTX8_CFG_BLOCK_PID_LEN];
	u8 hw_vid[GOODIX_GTX8_CFG_BLOCK_VID_LEN];
	u8 fw_mask[GOODIX_GTX8_CFG_BLOCK_FW_MASK_LEN];
	u8 fw_patch[GOODIX_GTX8_CFG_BLOCK_FW_PATCH_LEN];
	__le16 x_res_offset;
	__le16 y_res_offset;
	__le16 trigger_offset;
} __packed;

struct goodix_gtx8_cfg_pkg_reg_info {
	struct goodix_gtx8_cfg_pkg_reg cfg_send_flag;
	struct goodix_gtx8_cfg_pkg_reg version_base;
	struct goodix_gtx8_cfg_pkg_reg pid;
	struct goodix_gtx8_cfg_pkg_reg vid;
	struct goodix_gtx8_cfg_pkg_reg sensor_id;
	struct goodix_gtx8_cfg_pkg_reg fw_mask;
	struct goodix_gtx8_cfg_pkg_reg fw_status;
	struct goodix_gtx8_cfg_pkg_reg cfg_addr;
	struct goodix_gtx8_cfg_pkg_reg esd;
	struct goodix_gtx8_cfg_pkg_reg command;
	struct goodix_gtx8_cfg_pkg_reg coor;
	struct goodix_gtx8_cfg_pkg_reg gesture;
	struct goodix_gtx8_cfg_pkg_reg fw_request;
	struct goodix_gtx8_cfg_pkg_reg proximity;
	u8 reserved[GOODIX_GTX8_CFG_BLOCK_RESERVED_LEN];
} __packed;

struct goodix_gtx8_cfg_bin_head {
	__le32 bin_len;
	u8 checksum;
	u8 bin_version[4];
	u8 pkg_num;
} __packed;

#define GOODIX_GTX8_CFG_BIN_HEAD_LEN \
	(sizeof(struct goodix_gtx8_cfg_bin_head) + \
	 GOODIX_GTX8_CFG_BIN_HEAD_RESERVED_LEN)
#define GOODIX_GTX8_CFG_PKG_CONST_INFO_LEN \
	sizeof(struct goodix_gtx8_cfg_pkg_const_info)
#define GOODIX_GTX8_CFG_PKG_REG_INFO_LEN \
	sizeof(struct goodix_gtx8_cfg_pkg_reg_info)
#define GOODIX_GTX8_CFG_PKG_HEAD_LEN \
	(GOODIX_GTX8_CFG_PKG_CONST_INFO_LEN + GOODIX_GTX8_CFG_PKG_REG_INFO_LEN)

static const u8 goodix_gt9896_cfg_bin[] = {
	0x50, 0x04, 0x00, 0x00, 0x13, 0x33, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x12, 0x00, 0x3e, 0x04, 0x00, 0x00, 0x79, 0x65,
	0x6c, 0x6c, 0x6f, 0x77, 0x73, 0x74, 0x6f, 0x6e, 0x65, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x01, 0x39, 0x38, 0x39, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x34, 0x40, 0x04, 0x00, 0x14, 0x40, 0x87, 0x00, 0x22, 0x40,
	0x04, 0x00, 0x2a, 0x40, 0x04, 0x00, 0x2f, 0x40, 0x0f, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x96, 0x00, 0x00, 0x66, 0x41,
	0x00, 0x00, 0x60, 0x41, 0x00, 0x00, 0x80, 0x41, 0x00, 0x00, 0x80, 0x41,
	0x00, 0x00, 0x80, 0x41, 0x00, 0x00, 0x82, 0x41, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x01, 0x1e, 0x00, 0x29,
	0x01, 0x04, 0x5f, 0xad, 0xa7, 0xdc, 0x02, 0x94, 0x02, 0x1f, 0x14, 0xf1,
	0x48, 0x92, 0x31, 0x33, 0x04, 0x01, 0x23, 0x01, 0x10, 0x38, 0x08, 0x08,
	0x04, 0x00, 0x00, 0x10, 0x23, 0x02, 0x22, 0x22, 0x22, 0x01, 0x11, 0x14,
	0x44, 0x00, 0x0f, 0xff, 0xff, 0x05, 0xfb, 0x03, 0x64, 0x41, 0x29, 0x25,
	0x24, 0x27, 0x26, 0x2b, 0x2a, 0x4a, 0x2c, 0x04, 0x4b, 0x01, 0x00, 0x0c,
	0x03, 0x0d, 0x06, 0x10, 0x0f, 0x12, 0x11, 0x42, 0x0e, 0x02, 0x05, 0x1a,
	0x13, 0x18, 0x1b, 0x19, 0x1f, 0x1c, 0x23, 0x21, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f,
	0x37, 0x3c, 0x3a, 0x39, 0x38, 0x36, 0x33, 0x35, 0x34, 0x32, 0x30, 0x31,
	0x2f, 0x2e, 0x2d, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0x38, 0x50, 0x04, 0x64, 0x27, 0x1e, 0x25, 0x23, 0x24, 0x21, 0x20,
	0x1f, 0x1d, 0x1c, 0x1a, 0x19, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11,
	0x10, 0x0f, 0x0e, 0x26, 0x0d, 0x1b, 0x18, 0x0a, 0x0b, 0x0c, 0x08, 0x09,
	0x05, 0x07, 0x02, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x14, 0x1e, 0x16, 0x1a, 0x19,
	0x1d, 0x1c, 0x20, 0x1f, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x36, 0x05, 0x05,
	0x09, 0x01, 0x02, 0x01, 0x06, 0x26, 0x3b, 0x10, 0x38, 0x01, 0x00, 0xc2,
	0x06, 0x0a, 0x04, 0x38, 0x09, 0x60, 0x00, 0x46, 0x00, 0x69, 0x0a, 0x0f,
	0x01, 0x7d, 0x07, 0x02, 0x01, 0x00, 0x00, 0x0a, 0x08, 0x14, 0x14, 0x17,
	0x18, 0x1e, 0x17, 0x18, 0x1e, 0x17, 0x18, 0x1e, 0x17, 0x18, 0x1f, 0x76,
	0x00, 0x3a, 0x10, 0x06, 0x0c, 0x15, 0x02, 0x4c, 0x09, 0x08, 0x00, 0x32,
	0x00, 0x50, 0x00, 0x98, 0x02, 0x02, 0x01, 0x2f, 0x0a, 0x09, 0x09, 0x00,
	0x12, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x18, 0x00, 0x54, 0x0b, 0x14, 0x0b,
	0x14, 0x03, 0x00, 0x1c, 0x0a, 0x00, 0x0f, 0x04, 0x84, 0x7d, 0x78, 0x73,
	0x6f, 0x6e, 0x29, 0x2d, 0x31, 0x35, 0x39, 0x04, 0x38, 0x0c, 0x02, 0x0a,
	0x05, 0x00, 0x1d, 0x0d, 0xd1, 0x21, 0x32, 0x75, 0x02, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x55, 0x00, 0x01, 0x00, 0x2d, 0x00,
	0x1f, 0x00, 0x23, 0x00, 0x16, 0x00, 0x5d, 0x00, 0x54, 0x00, 0x23, 0x00,
	0x23, 0x00, 0x23, 0x00, 0x23, 0x00, 0x23, 0x00, 0x23, 0x00, 0x23, 0x00,
	0x23, 0x00, 0x23, 0x00, 0x23, 0x00, 0x23, 0x00, 0x23, 0x00, 0x23, 0x00,
	0x23, 0x00, 0x23, 0x00, 0x23, 0x00, 0x23, 0x00, 0x23, 0x00, 0x23, 0x00,
	0x23, 0x00, 0x28, 0x00, 0x23, 0x00, 0x1e, 0x00, 0x23, 0x00, 0x23, 0x00,
	0x1d, 0x00, 0x1d, 0x00, 0x23, 0x00, 0x1d, 0x00, 0x1e, 0x00, 0x20, 0x00,
	0x1d, 0x00, 0x1d, 0x00, 0x1d, 0x00, 0x1d, 0x00, 0x1d, 0x00, 0x1d, 0x00,
	0x1d, 0x00, 0x1e, 0x00, 0x29, 0x00, 0x16, 0x00, 0x16, 0x00, 0x11, 0x00,
	0x11, 0x00, 0x16, 0x00, 0x12, 0x00, 0x16, 0x00, 0x12, 0x00, 0x13, 0x00,
	0x12, 0x00, 0x14, 0x00, 0x13, 0x00, 0x13, 0x00, 0x14, 0x00, 0x15, 0x00,
	0x15, 0x00, 0x15, 0x00, 0x15, 0x00, 0x15, 0x00, 0x15, 0x00, 0x15, 0x00,
	0x15, 0x00, 0x15, 0x00, 0x16, 0x00, 0x14, 0x00, 0x16, 0x00, 0x16, 0x00,
	0x14, 0x00, 0x16, 0x00, 0x16, 0x00, 0x17, 0x00, 0x16, 0x00, 0x16, 0x00,
	0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x15, 0x00, 0x16, 0x00, 0x16, 0x00,
	0x1d, 0xd8, 0x59, 0x01, 0x02, 0x04, 0x00, 0x2b, 0x0b, 0xb8, 0x00, 0x2f,
	0x0b, 0xb8, 0x37, 0x37, 0x37, 0x34, 0x34, 0x34, 0x33, 0x03, 0x10, 0xa3,
	0x0e, 0x01, 0x18, 0x00, 0x27, 0x10, 0x0f, 0x00, 0x32, 0x00, 0x46, 0x00,
	0x28, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x05, 0x10, 0x01, 0x1c,
	0x11, 0x11, 0x22, 0x00, 0x01, 0x0d, 0x50, 0x37, 0x64, 0xaa, 0x50, 0x00,
	0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x41, 0x12, 0x03, 0x01,
	0x12, 0x00, 0x00, 0x28, 0x13, 0x0d, 0x04, 0x02, 0x00, 0x10, 0x46, 0x41,
	0x1a, 0x03, 0x1e, 0x02, 0x1e, 0x10, 0x46, 0x01, 0x6e, 0x16, 0x0b, 0x51,
	0x40, 0x20, 0x01, 0xf4, 0x00, 0x78, 0x00, 0x64, 0x00, 0x00, 0x02, 0xa3,
	0x17, 0x06, 0x3f, 0x00, 0x02, 0x58, 0x00, 0xc8, 0x01, 0x7e, 0x1a, 0x11,
	0x0d, 0x32, 0x00, 0x64, 0x00, 0xc8, 0x29, 0x2e, 0x31, 0x38, 0x00, 0x00,
	0x25, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7b, 0x1c, 0x3e, 0x00, 0x00, 0x00,
	0x3c, 0x00, 0x32, 0x01, 0x05, 0x04, 0x9a, 0x1b, 0x66, 0x06, 0xcd, 0x3f,
	0x33, 0x04, 0x9a, 0x00, 0x00, 0x05, 0x05, 0x05, 0x08, 0x00, 0x02, 0x00,
	0x20, 0x8d, 0x9c, 0x40, 0x03, 0x32, 0x02, 0x00, 0x20, 0x8d, 0x14, 0x06,
	0xcc, 0x07, 0x86, 0x01, 0xbf, 0x02, 0x79, 0x06, 0x19, 0x43, 0x24, 0x04,
	0x00, 0x14, 0x31, 0x02, 0x1c, 0x07, 0x29, 0x00, 0x74, 0x87, 0x50, 0x0b,
	0x6c, 0x1d, 0x5e, 0x00, 0x04, 0x02, 0x15, 0x03, 0xb5, 0x13, 0xb5, 0x00,
	0x04, 0x02, 0x15, 0x03, 0xb5, 0x13, 0xb5, 0x13, 0xb5, 0x1f, 0xff, 0x1f,
	0xff, 0x1f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
	0x0b, 0x14, 0x28, 0x0f, 0x2c, 0x55, 0x8c, 0x06, 0x0b, 0x14, 0x28, 0x0f,
	0x2c, 0x55, 0x8c, 0x28, 0x19, 0x0a, 0x19, 0x0a, 0x02, 0xc8, 0x8c, 0x0a,
	0xb4, 0xfa, 0x00, 0x8c, 0x0a, 0x00, 0x64, 0x06, 0x01, 0xb3, 0x35, 0x0a,
	0x3c, 0x1e, 0x3c, 0x55, 0x03, 0xe8, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x12, 0x92, 0x1e, 0x05, 0x00, 0x00, 0x6e, 0x00, 0x5a, 0x00, 0xeb,
	0x1f, 0x05, 0x01, 0x03, 0x08, 0x06, 0x04, 0x00, 0x3a, 0x20, 0x0b, 0xa5,
	0x8c, 0x00, 0x78, 0x01, 0x2c, 0x01, 0x7c, 0x01, 0x86, 0x0a, 0x03, 0x0f,
	0x23, 0x10, 0x03, 0x01, 0x80, 0x00, 0x80, 0x00, 0x1c, 0x07, 0xd0, 0x05,
	0x0a, 0x50, 0x00, 0x0a, 0x00, 0x00, 0x02, 0x93, 0x27, 0x1a, 0x05, 0xf6,
	0x0c, 0x0a, 0x14, 0x46, 0x08, 0x24, 0x18, 0x00, 0x00, 0x3a, 0x3a, 0x30,
	0x78, 0x30, 0x30, 0x52, 0x52, 0x78, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x05, 0x00, 0x28, 0x04, 0x16, 0x55, 0x4a, 0xb0, 0x01, 0x91, 0x29, 0x0a,
	0x00, 0xbe, 0x00, 0x2b, 0x00, 0x32, 0x00, 0x30, 0x00, 0x2b, 0x01, 0xa9,
};

static bool goodix_gtx8_checksum_valid_normandy(const u8 *data, int size)
{
	u8 cal_checksum = 0;
	int i;

	if (size < GOODIX_GTX8_CHECKSUM_SIZE)
		return false;

	for (i = 0; i < size; i++)
		cal_checksum += data[i];

	return cal_checksum == 0;
}

static bool goodix_gtx8_checksum_valid_yellowstone(const u8 *data, int size)
{
	u16 cal_checksum = 0;
	u16 r_checksum;
	int i;

	if (size < GOODIX_GTX8_CHECKSUM_SIZE)
		return false;

	for (i = 0; i < size - GOODIX_GTX8_CHECKSUM_SIZE; i++)
		cal_checksum += data[i];

	r_checksum = get_unaligned_be16(&data[i]);

	return cal_checksum == r_checksum;
}

static u8 goodix_gtx8_checksum_u8(const u8 *data, int size)
{
	u8 checksum = 0;
	int i;

	for (i = 0; i < size; i++)
		checksum += data[i];

	return checksum;
}

static const char *goodix_gtx8_ic_type_name(enum goodix_gtx8_ic_type ic_type)
{
	switch (ic_type) {
	case IC_TYPE_NORMANDY:
		return "normandy";
	case IC_TYPE_YELLOWSTONE:
		return "yellowstone";
	default:
		return NULL;
	}
}

static u16 goodix_gtx8_cfg_reg_addr(const struct goodix_gtx8_cfg_pkg_reg *reg)
{
	return le16_to_cpu(reg->addr);
}

static int goodix_gtx8_raw_write(struct goodix_gtx8_core *cd, u32 addr,
				 const void *data, size_t len)
{
	const u8 *buf = data;
	size_t xfer_len;
	int error;

	while (len) {
		xfer_len = min_t(size_t, len,
				 I2C_MAX_TRANSFER_SIZE - GOODIX_GTX8_REG_ADDR_SIZE);

		error = regmap_raw_write(cd->regmap, addr, buf, xfer_len);
		if (error)
			return error;

		addr += xfer_len;
		buf += xfer_len;
		len -= xfer_len;
	}

	return 0;
}

static bool goodix_gtx8_config_matches(struct goodix_gtx8_core *cd,
				       const struct goodix_gtx8_cfg_pkg_const_info *const_info,
				       const struct goodix_gtx8_cfg_pkg_reg_info *reg_info)
{
	const char *ic_type_name;
	u16 addr;
	u8 sensor_id;
	u8 buf[GOODIX_GTX8_CFG_BLOCK_FW_MASK_LEN];
	int error;

	ic_type_name = goodix_gtx8_ic_type_name(cd->ic_data->ic_type);
	if (!ic_type_name ||
	    strncmp((const char *)const_info->ic_type, ic_type_name,
		    strlen(ic_type_name)))
		return false;

	if (const_info->cfg_type != GOODIX_GTX8_CFG_TYPE_NORMAL)
		return false;

	addr = goodix_gtx8_cfg_reg_addr(&reg_info->sensor_id);
	if (addr) {
		error = regmap_raw_read(cd->regmap, addr, &sensor_id, 1);
		if (error)
			return false;

		if (reg_info->sensor_id.reserved1)
			sensor_id &= reg_info->sensor_id.reserved1;

		if (sensor_id != const_info->sensor_id)
			return false;
	}

	addr = goodix_gtx8_cfg_reg_addr(&reg_info->pid);
	if (addr && reg_info->pid.reserved1 &&
	    reg_info->pid.reserved1 <= GOODIX_GTX8_CFG_BLOCK_PID_LEN) {
		error = regmap_raw_read(cd->regmap, addr, buf,
					reg_info->pid.reserved1);
		if (error)
			return false;

		if (memcmp(buf, const_info->hw_pid, reg_info->pid.reserved1))
			return false;
	}

	addr = goodix_gtx8_cfg_reg_addr(&reg_info->fw_mask);
	if (addr && const_info->fw_mask[0]) {
		error = regmap_raw_read(cd->regmap, addr, buf,
					sizeof(const_info->fw_mask));
		if (error)
			return false;

		if (memcmp(buf, const_info->fw_mask, sizeof(const_info->fw_mask)))
			return false;
	}

	return true;
}

static int goodix_gtx8_apply_config_package(struct goodix_gtx8_core *cd,
					    const struct goodix_gtx8_cfg_pkg_const_info *const_info,
					    const struct goodix_gtx8_cfg_pkg_reg_info *reg_info,
					    const u8 *config, size_t config_len)
{
	if (!config_len || config_len > GOODIX_GTX8_CFG_MAX_SIZE)
		return -EINVAL;

	cd->config = devm_kmemdup(cd->dev, config, config_len, GFP_KERNEL);
	if (!cd->config)
		return -ENOMEM;

	cd->config_len = config_len;
	cd->config_addr = goodix_gtx8_cfg_reg_addr(&reg_info->cfg_addr);
	cd->command_addr = goodix_gtx8_cfg_reg_addr(&reg_info->command);
	cd->esd_addr = goodix_gtx8_cfg_reg_addr(&reg_info->esd);
	cd->fw_request_addr = goodix_gtx8_cfg_reg_addr(&reg_info->fw_request);

	if (goodix_gtx8_cfg_reg_addr(&reg_info->coor))
		cd->touch_data_addr = goodix_gtx8_cfg_reg_addr(&reg_info->coor);

	dev_dbg(cd->dev, "loaded %.*s config v%02x len %zu\n",
		GOODIX_GTX8_CFG_IC_TYPE_NAME_MAX_LEN,
		(const char *)const_info->ic_type, cd->config[0],
		cd->config_len);

	return 0;
}

static int goodix_gtx8_parse_config_bin(struct goodix_gtx8_core *cd,
					const u8 *data, size_t size)
{
	const struct goodix_gtx8_cfg_pkg_const_info *const_info;
	const struct goodix_gtx8_cfg_pkg_const_info *fallback_const = NULL;
	const struct goodix_gtx8_cfg_pkg_reg_info *fallback_reg = NULL;
	const struct goodix_gtx8_cfg_pkg_reg_info *reg_info;
	const struct goodix_gtx8_cfg_bin_head *head;
	const u8 *config, *fallback_config = NULL;
	size_t config_len, fallback_config_len = 0;
	u16 offset;
	u32 pkg_len;
	int i;

	if (size < GOODIX_GTX8_CFG_BIN_HEAD_LEN)
		return -EINVAL;

	head = (const struct goodix_gtx8_cfg_bin_head *)data;
	if (le32_to_cpu(head->bin_len) != size)
		return -EINVAL;

	if (goodix_gtx8_checksum_u8(&data[GOODIX_GTX8_CFG_BIN_VERSION_START],
				    size - GOODIX_GTX8_CFG_BIN_VERSION_START) !=
	    head->checksum)
		return -EINVAL;

	if (size < GOODIX_GTX8_CFG_BIN_HEAD_LEN +
	    head->pkg_num * GOODIX_GTX8_CFG_OFFSET_LEN)
		return -EINVAL;

	for (i = 0; i < head->pkg_num; i++) {
		offset = get_unaligned_le16(&data[GOODIX_GTX8_CFG_BIN_HEAD_LEN +
						  i * GOODIX_GTX8_CFG_OFFSET_LEN]);
		if (offset + GOODIX_GTX8_CFG_PKG_HEAD_LEN > size)
			return -EINVAL;

		const_info = (const struct goodix_gtx8_cfg_pkg_const_info *)
			     &data[offset];
		reg_info = (const struct goodix_gtx8_cfg_pkg_reg_info *)
			   &data[offset + GOODIX_GTX8_CFG_PKG_CONST_INFO_LEN];
		pkg_len = le32_to_cpu(const_info->pkg_len);

		if (pkg_len < GOODIX_GTX8_CFG_PKG_HEAD_LEN ||
		    offset + pkg_len > size)
			return -EINVAL;

		if (const_info->cfg_type != GOODIX_GTX8_CFG_TYPE_NORMAL)
			continue;

		config = &data[offset + GOODIX_GTX8_CFG_PKG_HEAD_LEN];
		config_len = pkg_len - GOODIX_GTX8_CFG_PKG_HEAD_LEN;

		if (!fallback_config) {
			fallback_const = const_info;
			fallback_reg = reg_info;
			fallback_config = config;
			fallback_config_len = config_len;
		}

		if (!goodix_gtx8_config_matches(cd, const_info, reg_info))
			continue;

		return goodix_gtx8_apply_config_package(cd, const_info,
							reg_info, config,
							config_len);
	}

	/*
	 * Lineage's downstream driver also falls back to package 0 when
	 * sensor matching fails, which is useful for replacement panels that
	 * report unexpected sensor IDs.
	 */
	if (fallback_config)
		return goodix_gtx8_apply_config_package(cd, fallback_const,
							fallback_reg,
							fallback_config,
							fallback_config_len);

	return -ENOENT;
}

static int goodix_gtx8_load_config(struct goodix_gtx8_core *cd)
{
	const struct firmware *fw = NULL;
	int error;

	if (!cd->ic_data->config_name)
		return 0;

	error = request_firmware_direct(&fw, cd->ic_data->config_name, cd->dev);
	if (!error) {
		error = goodix_gtx8_parse_config_bin(cd, fw->data, fw->size);
		release_firmware(fw);
		if (!error)
			return 0;

		dev_warn(cd->dev, "failed to parse %s: %d\n",
			 cd->ic_data->config_name, error);
	}

	if (!cd->ic_data->config_fallback)
		return 0;

	error = goodix_gtx8_parse_config_bin(cd, cd->ic_data->config_fallback,
					     cd->ic_data->config_fallback_size);
	if (error)
		dev_warn(cd->dev, "failed to parse fallback config: %d\n",
			 error);

	return error == -ENOMEM ? error : 0;
}

static int goodix_gtx8_send_command(struct goodix_gtx8_core *cd, u8 command,
				    u16 data)
{
	u8 buf[5];
	u16 checksum;
	size_t len;

	if (!cd->command_addr || !command)
		return -EINVAL;

	if (cd->ic_data->ic_type == IC_TYPE_YELLOWSTONE) {
		buf[0] = command;
		buf[1] = data >> 8;
		buf[2] = data;
		checksum = buf[0] + buf[1] + buf[2];
		buf[3] = checksum >> 8;
		buf[4] = checksum;
		len = 5;
	} else {
		buf[0] = command;
		buf[1] = data;
		buf[2] = 0 - command - data;
		len = 3;
	}

	return regmap_raw_write(cd->regmap, cd->command_addr, buf, len);
}

static int goodix_gtx8_wait_cfg_cmd_ready(struct goodix_gtx8_core *cd,
					  u8 ready_cmd, u8 resend_cmd)
{
	u8 cmd_buf[3];
	int error;
	int i;

	for (i = 0; i < GOODIX_GTX8_WAIT_CFG_READY_RETRY; i++) {
		error = regmap_raw_read(cd->regmap, cd->command_addr, cmd_buf,
					sizeof(cmd_buf));
		if (error)
			return error;

		if (cmd_buf[0] == ready_cmd)
			return 0;

		if (cmd_buf[0] != resend_cmd) {
			error = goodix_gtx8_send_command(cd, resend_cmd, 0);
			if (error)
				return error;
		}

		usleep_range(10000, 11000);
	}

	return -ETIMEDOUT;
}

static int goodix_gtx8_send_config(struct goodix_gtx8_core *cd)
{
	u8 buf[3] = { 0 };
	int error;
	int i;

	if (!cd->config)
		return 0;

	if (!cd->config_addr || !cd->command_addr)
		return -EINVAL;

	for (i = 0; i < GOODIX_GTX8_WAIT_CMD_FREE_RETRY; i++) {
		error = regmap_raw_read(cd->regmap, cd->command_addr, buf, 1);
		if (!error && buf[0] == GOODIX_GTX8_CMD_REG_READY)
			break;

		usleep_range(10000, 11000);
	}

	if (i == GOODIX_GTX8_WAIT_CMD_FREE_RETRY)
		return -ETIMEDOUT;

	error = goodix_gtx8_send_command(cd, GOODIX_GTX8_CMD_START_SEND_CFG,
					 0);
	if (error)
		return error;

	error = goodix_gtx8_wait_cfg_cmd_ready(cd,
					       GOODIX_GTX8_CMD_SEND_CFG_PREPARE_OK,
					       GOODIX_GTX8_CMD_START_SEND_CFG);
	if (error)
		return error;

	error = goodix_gtx8_raw_write(cd, cd->config_addr, cd->config,
				      cd->config_len);
	if (error)
		return error;

	error = goodix_gtx8_send_command(cd, GOODIX_GTX8_CMD_END_SEND_CFG, 0);
	if (error)
		return error;

	if (cd->ic_data->ic_type != IC_TYPE_YELLOWSTONE) {
		for (i = 0; i < GOODIX_GTX8_WAIT_CMD_FREE_RETRY; i++) {
			error = regmap_raw_read(cd->regmap, cd->command_addr,
						buf, 1);
			if (!error && buf[0] == GOODIX_GTX8_CMD_REG_READY)
				return 0;

			usleep_range(10000, 11000);
		}

		return -ETIMEDOUT;
	}

	for (i = 0; i < GOODIX_GTX8_WAIT_CMD_FREE_RETRY; i++) {
		error = regmap_raw_read(cd->regmap, cd->command_addr, buf,
					sizeof(buf));
		if (!error && (buf[0] == GOODIX_GTX8_CMD_CFG_ERR ||
			       buf[0] == GOODIX_GTX8_CMD_CFG_OK))
			break;

		usleep_range(10000, 11000);
	}

	error = goodix_gtx8_send_command(cd, GOODIX_GTX8_CMD_END_SEND_CFG_YS,
					 0);
	if (error)
		return error;

	if (i == GOODIX_GTX8_WAIT_CMD_FREE_RETRY)
		return -ETIMEDOUT;

	if (buf[0] == GOODIX_GTX8_CMD_CFG_ERR &&
	    buf[2] != GOODIX_GTX8_CFG_REPLY_DATA_EQU)
		return -EINVAL;

	return 0;
}

static void goodix_gtx8_init_config(struct goodix_gtx8_core *cd)
{
	u8 esd = 0;
	int error;

	error = goodix_gtx8_send_config(cd);
	if (error) {
		dev_warn(cd->dev, "failed to send config: %d\n", error);
		return;
	}

	if (cd->esd_addr) {
		error = regmap_raw_write(cd->regmap, cd->esd_addr, &esd, 1);
		if (error)
			dev_warn(cd->dev, "failed to init ESD: %d\n", error);
	}
}

static int goodix_gtx8_get_remaining_contacts(struct goodix_gtx8_core *cd,
					      int n)
{
	size_t offset = cd->ic_data->pre_read_size;
	u32 addr = cd->touch_data_addr + offset;
	int error;

	error = regmap_raw_read(cd->regmap, addr, &cd->event_buffer[offset],
				(n - 1) * GOODIX_GTX8_TOUCH_SIZE);
	if (error) {
		dev_err_ratelimited(cd->dev, "failed to get touch data, %d\n",
				    error);
		return error;
	}

	return 0;
}

static int goodix_gtx8_read_event_head(struct goodix_gtx8_core *cd)
{
	return regmap_raw_read(cd->regmap, cd->touch_data_addr,
			       cd->event_buffer, cd->ic_data->pre_read_size);
}

static void goodix_gtx8_report_state(struct goodix_gtx8_core *cd, u8 touch_num,
				     union goodix_gtx8_touch *touch_data)
{
	union goodix_gtx8_touch *t;
	int i;
	u8 finger_id;

	for (i = 0; i < touch_num; i++) {
		t = &touch_data[i];

		if (cd->ic_data->ic_type == IC_TYPE_NORMANDY) {
			input_mt_slot(cd->input_dev, t->normandy.finger_id);
			input_mt_report_slot_state(cd->input_dev,
						   MT_TOOL_FINGER, true);

			touchscreen_report_pos(cd->input_dev, &cd->props,
					       __le16_to_cpu(t->normandy.x),
					       __le16_to_cpu(t->normandy.y),
					       true);
			input_report_abs(cd->input_dev, ABS_MT_TOUCH_MAJOR,
					 t->normandy.w);
		} else {
			finger_id = FIELD_GET(
				GOODIX_GTX8_FINGER_ID_MASK_YELLOWSTONE,
				t->yellowstone.finger_id);
			input_mt_slot(cd->input_dev, finger_id);
			input_mt_report_slot_state(cd->input_dev,
						   MT_TOOL_FINGER, true);

			touchscreen_report_pos(cd->input_dev, &cd->props,
					       __be16_to_cpu(t->yellowstone.x),
					       __be16_to_cpu(t->yellowstone.y),
					       true);
			input_report_abs(cd->input_dev, ABS_MT_TOUCH_MAJOR,
					 t->yellowstone.w);
		}
	}

	input_mt_sync_frame(cd->input_dev);
	input_sync(cd->input_dev);
}

static void goodix_gtx8_touch_handler(struct goodix_gtx8_core *cd, u8 touch_num,
				      union goodix_gtx8_touch *touch_data)
{
	int error;

	touch_num = FIELD_GET(GOODIX_GTX8_TOUCH_COUNT_MASK, touch_num);

	if (touch_num > GOODIX_GTX8_MAX_TOUCH) {
		dev_warn(cd->dev, "invalid touch num %d\n", touch_num);
		return;
	}

	if (touch_num > 1) {
		/* read additional contact data if more than 1 touch event */
		error = goodix_gtx8_get_remaining_contacts(cd, touch_num);
		if (error)
			return;
	}

	if (touch_num) {
		/*
		 * Normandy checksum is for the entire read buffer,
		 * Yellowstone is only for the touch data (since header
		 * has a separate checksum)
		 */
		if (cd->ic_data->ic_type == IC_TYPE_NORMANDY) {
			int len = GOODIX_GTX8_HEADER_SIZE_NORMANDY +
				  touch_num * GOODIX_GTX8_TOUCH_SIZE +
				  GOODIX_GTX8_CHECKSUM_SIZE;
			if (!goodix_gtx8_checksum_valid_normandy(
				    cd->event_buffer, len)) {
				dev_err(cd->dev,
					"touch data checksum error: %*ph\n",
					len, cd->event_buffer);
				return;
			}
		} else {
			int len = touch_num * GOODIX_GTX8_TOUCH_SIZE +
				  GOODIX_GTX8_CHECKSUM_SIZE;
			if (!goodix_gtx8_checksum_valid_yellowstone(
				    (u8 *)touch_data, len)) {
				dev_err(cd->dev,
					"touch data checksum error: %*ph\n",
					len, (u8 *)touch_data);
				return;
			}
		}
	}

	goodix_gtx8_report_state(cd, touch_num, touch_data);
}

static int goodix_gtx8_handle_events(struct goodix_gtx8_core *cd)
{
	struct goodix_gtx8_event_normandy *ev_normandy;
	struct goodix_gtx8_event_yellowstone *ev_yellowstone;
	union goodix_gtx8_touch *touch_data;
	int error;
	u8 status, touch_num;

	error = goodix_gtx8_read_event_head(cd);
	if (error) {
		dev_warn_ratelimited(
			cd->dev, "failed to get event head data: %d\n", error);
		return error;
	}

	/*
	 * Both IC types have the same data in the header, just at different
	 * offsets
	 */
	if (cd->ic_data->ic_type == IC_TYPE_NORMANDY) {
		ev_normandy =
			(struct goodix_gtx8_event_normandy *)cd->event_buffer;
		status = ev_normandy->hdr.status;
		touch_num = ev_normandy->hdr.touch_num;
		touch_data = (union goodix_gtx8_touch *)ev_normandy->data;
	} else {
		ev_yellowstone = (struct goodix_gtx8_event_yellowstone *)
					 cd->event_buffer;
		status = ev_yellowstone->hdr.status;
		touch_num = ev_yellowstone->hdr.touch_num;
		touch_data = (union goodix_gtx8_touch *)ev_yellowstone->data;
	}

	if (status == 0)
		return 0;

	/* Yellowstone ICs have a checksum for the header */
	if (cd->ic_data->ic_type == IC_TYPE_YELLOWSTONE &&
	    !goodix_gtx8_checksum_valid_yellowstone(
		    cd->event_buffer, GOODIX_GTX8_HEADER_SIZE_YELLOWSTONE)) {
		dev_warn_ratelimited(cd->dev,
				     "touch head checksum error: %*ph\n",
				     (int)GOODIX_GTX8_HEADER_SIZE_YELLOWSTONE,
				     cd->event_buffer);
		goto out_clear;
	}

	if ((status & GOODIX_GTX8_TOUCH_EVENT) && cd->poll_attempts_left &&
	    FIELD_GET(GOODIX_GTX8_TOUCH_COUNT_MASK, touch_num)) {
		cd->poll_attempts_left = 0;
		WRITE_ONCE(cd->poll_interval_ms, 0);
	}

	if (status & GOODIX_GTX8_TOUCH_EVENT)
		goodix_gtx8_touch_handler(cd, touch_num, touch_data);

	if (status & GOODIX_GTX8_REQUEST_EVENT) {
		error = goodix_gtx8_send_config(cd);
		if (error)
			dev_warn_ratelimited(cd->dev,
					     "failed to handle config request: %d\n",
					     error);
	}

out_clear:
	/* Clear up status field */
	regmap_write(cd->regmap, cd->touch_data_addr, 0);

	return 0;
}

static irqreturn_t goodix_gtx8_irq(int irq, void *data)
{
	struct goodix_gtx8_core *cd = data;

	mutex_lock(&cd->event_lock);
	goodix_gtx8_handle_events(cd);
	mutex_unlock(&cd->event_lock);

	return IRQ_HANDLED;
}

static void goodix_gtx8_queue_poll(struct goodix_gtx8_core *cd)
{
	unsigned int interval = READ_ONCE(cd->poll_interval_ms);

	if (interval)
		schedule_delayed_work(&cd->poll_work,
				      msecs_to_jiffies(interval));
}

static void goodix_gtx8_wake_probe_alt_addr(struct goodix_gtx8_core *cd)
{
	struct i2c_msg msgs[2];
	u8 rx_buf[8];
	size_t i;
	int error;

	for (i = 0; i < ARRAY_SIZE(goodix_gtx8_wake_regs); i++) {
		u8 reg = goodix_gtx8_wake_regs[i];

		msgs[0].addr = GOODIX_GTX8_WAKE_ADDR;
		msgs[0].flags = 0;
		msgs[0].len = 1;
		msgs[0].buf = &reg;
		msgs[1].addr = GOODIX_GTX8_WAKE_ADDR;
		msgs[1].flags = I2C_M_RD;
		msgs[1].len = sizeof(rx_buf);
		msgs[1].buf = rx_buf;

		error = i2c_transfer(cd->client->adapter, msgs, ARRAY_SIZE(msgs));
		if (error != ARRAY_SIZE(msgs))
			dev_dbg(cd->dev, "wake probe failed at 0x%02x: %d\n",
				goodix_gtx8_wake_regs[i], error);
	}
}

static void goodix_gtx8_start_wake_poll(struct goodix_gtx8_core *cd)
{
	cd->poll_error_count = 0;
	cd->poll_attempts_left = GOODIX_GTX8_WAKE_POLL_ATTEMPTS;
	goodix_gtx8_wake_probe_alt_addr(cd);
	WRITE_ONCE(cd->poll_interval_ms, GOODIX_GTX8_WAKE_POLL_INTERVAL_MS);
	goodix_gtx8_queue_poll(cd);
}

static void goodix_gtx8_poll_work(struct work_struct *work)
{
	struct goodix_gtx8_core *cd = container_of(to_delayed_work(work),
						  struct goodix_gtx8_core,
						  poll_work);
	int error;

	/*
	 * Some panels may need a few raw event-register reads before
	 * they start raising IRQs
	 */
	mutex_lock(&cd->event_lock);
	error = goodix_gtx8_read_event_head(cd);
	mutex_unlock(&cd->event_lock);

	/* The boot wake poll is bounded, so tolerate transient read failures. */
	if (error) {
		if (!cd->poll_attempts_left &&
		    ++cd->poll_error_count >= GOODIX_GTX8_POLL_MAX_ERRORS) {
			WRITE_ONCE(cd->poll_interval_ms, 0);
			dev_warn(cd->dev,
				 "disabling polling after repeated I2C errors\n");
			return;
		}
	} else {
		cd->poll_error_count = 0;
	}

	if (cd->poll_attempts_left) {
		cd->poll_attempts_left--;
		if (!cd->poll_attempts_left) {
			WRITE_ONCE(cd->poll_interval_ms, 0);
			return;
		}
	}

	goodix_gtx8_queue_poll(cd);
}

static ssize_t poll_interval_ms_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_gtx8_core *cd = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(cd->poll_interval_ms));
}

static ssize_t poll_interval_ms_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct goodix_gtx8_core *cd = dev_get_drvdata(dev);
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 0, &interval);
	if (error)
		return error;

	if (interval && (interval < GOODIX_GTX8_POLL_INTERVAL_MIN_MS ||
			 interval > GOODIX_GTX8_POLL_INTERVAL_MAX_MS))
		return -EINVAL;

	WRITE_ONCE(cd->poll_interval_ms, 0);
	cancel_delayed_work_sync(&cd->poll_work);
	cd->poll_error_count = 0;
	cd->poll_attempts_left = 0;
	WRITE_ONCE(cd->poll_interval_ms, interval);
	goodix_gtx8_queue_poll(cd);

	return count;
}

static DEVICE_ATTR_RW(poll_interval_ms);

static ssize_t wake_sequence_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct goodix_gtx8_core *cd = dev_get_drvdata(dev);
	bool enable;
	int error;

	error = kstrtobool(buf, &enable);
	if (error)
		return error;

	WRITE_ONCE(cd->poll_interval_ms, 0);
	cancel_delayed_work_sync(&cd->poll_work);
	cd->poll_error_count = 0;
	cd->poll_attempts_left = 0;

	if (!enable)
		return count;

	goodix_gtx8_start_wake_poll(cd);

	return count;
}

static DEVICE_ATTR_WO(wake_sequence);

static struct attribute *goodix_gtx8_attrs[] = {
	&dev_attr_poll_interval_ms.attr,
	&dev_attr_wake_sequence.attr,
	NULL
};

static const struct attribute_group goodix_gtx8_attr_group = {
	.attrs = goodix_gtx8_attrs,
};

static int goodix_gtx8_input_dev_config(struct goodix_gtx8_core *cd)
{
	struct input_dev *input_dev;
	int error;

	input_dev = devm_input_allocate_device(cd->dev);
	if (!input_dev)
		return -ENOMEM;

	cd->input_dev = input_dev;
	input_set_drvdata(input_dev, cd);

	input_dev->name = "Goodix GTX8 Capacitive TouchScreen";
	input_dev->phys = "input/ts";

	input_dev->id = goodix_gtx8_input_id;

	input_set_abs_params(cd->input_dev, ABS_MT_POSITION_X, 0, SZ_64K - 1, 0,
			     0);
	input_set_abs_params(cd->input_dev, ABS_MT_POSITION_Y, 0, SZ_64K - 1, 0,
			     0);
	input_set_abs_params(cd->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	touchscreen_parse_properties(cd->input_dev, true, &cd->props);

	error = input_mt_init_slots(cd->input_dev, GOODIX_GTX8_MAX_TOUCH,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	error = input_register_device(cd->input_dev);
	if (error)
		return error;

	return 0;
}

static int goodix_gtx8_read_version(struct goodix_gtx8_core *cd)
{
	int error;

	/*
	 * The vendor driver reads a whole lot more data to calculate and
	 * verify a checksum. Without documentation, we don't know what
	 * most of that data is, so we only read the parts we know about
	 * and instead ensure their values are as expected
	 */
	error = regmap_raw_read(cd->regmap, cd->ic_data->fw_version_addr,
				&cd->fw_version, sizeof(cd->fw_version));
	if (error) {
		dev_err(cd->dev, "error reading fw version, %d\n", error);
		return error;
	}

	/*
	 * Since we don't verify the checksum, do a basic check that the
	 * product ID meets expectations
	 */
	if (memcmp(cd->fw_version.product_id, cd->ic_data->product_id,
		   sizeof(cd->fw_version.product_id))) {
		dev_err(cd->dev, "unexpected product ID, got: %c%c%c%c\n",
			cd->fw_version.product_id[0],
			cd->fw_version.product_id[1],
			cd->fw_version.product_id[2],
			cd->fw_version.product_id[3]);
		return -EINVAL;
	}

	return 0;
}

static int goodix_gtx8_dev_confirm(struct goodix_gtx8_core *cd)
{
	u8 rx_buf[1];
	int retry = 3;
	int error;

	while (retry--) {
		/*
		 * test_addr appears to always be the touch_data_addr for
		 * Normandy, but it doesn't really matter since all we
		 * need is a valid address
		 */
		error = regmap_raw_read(cd->regmap,
					cd->touch_data_addr, rx_buf,
					sizeof(rx_buf));

		if (!error)
			return 0;

		usleep_range(5000, 5100);
	}

	dev_err(cd->dev, "device confirm failed\n");

	return -EINVAL;
}

static int goodix_gtx8_power_on(struct goodix_gtx8_core *cd)
{
	int error;

	error = regulator_enable(cd->vddio);
	if (error) {
		dev_err(cd->dev, "Failed to enable VDDIO: %d\n", error);
		return error;
	}

	error = regulator_enable(cd->avdd);
	if (error) {
		dev_err(cd->dev, "Failed to enable AVDD: %d\n", error);
		goto err_vddio_disable;
	}

	/* Vendors usually configure the power on delay as 300ms */
	msleep(GOODIX_GTX8_POWER_ON_DELAY_MS);

	gpiod_set_value_cansleep(cd->reset_gpio, 0);

	/* Vendor waits 5ms for firmware to initialize */
	usleep_range(5000, 5100);

	error = goodix_gtx8_dev_confirm(cd);
	if (error)
		goto err_dev_reset;

	/* Vendor waits 100ms for firmware to fully boot */
	msleep(GOODIX_GTX8_NORMAL_RESET_DELAY_MS);

	return 0;

err_dev_reset:
	gpiod_set_value_cansleep(cd->reset_gpio, 1);
	regulator_disable(cd->avdd);
err_vddio_disable:
	regulator_disable(cd->vddio);
	return error;
}

static void goodix_gtx8_power_off(struct goodix_gtx8_core *cd)
{
	gpiod_set_value_cansleep(cd->reset_gpio, 1);
	regulator_disable(cd->avdd);
	regulator_disable(cd->vddio);
}

static int goodix_gtx8_suspend(struct device *dev)
{
	struct goodix_gtx8_core *cd = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&cd->poll_work);
	disable_irq(cd->irq);
	goodix_gtx8_power_off(cd);

	return 0;
}

static int goodix_gtx8_resume(struct device *dev)
{
	struct goodix_gtx8_core *cd = dev_get_drvdata(dev);
	int error;

	error = goodix_gtx8_power_on(cd);
	if (error)
		return error;

	goodix_gtx8_init_config(cd);

	enable_irq(cd->irq);

	return 0;
}

EXPORT_GPL_SIMPLE_DEV_PM_OPS(goodix_gtx8_pm_ops, goodix_gtx8_suspend,
			     goodix_gtx8_resume);

static void goodix_gtx8_power_off_act(void *data)
{
	struct goodix_gtx8_core *cd = data;

	cancel_delayed_work_sync(&cd->poll_work);
	goodix_gtx8_power_off(cd);
}

static int goodix_gtx8_probe(struct i2c_client *client)
{
	struct goodix_gtx8_core *cd;
	struct regmap *regmap;
	int error;

	cd = devm_kzalloc(&client->dev, sizeof(*cd), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &goodix_gtx8_regmap_conf);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	cd->dev = &client->dev;
	cd->client = client;
	cd->irq = client->irq;
	cd->regmap = regmap;
	cd->ic_data = i2c_get_match_data(client);
	cd->touch_data_addr = cd->ic_data->touch_data_addr;
	mutex_init(&cd->event_lock);
	INIT_DELAYED_WORK(&cd->poll_work, goodix_gtx8_poll_work);

	cd->event_buffer =
		devm_kzalloc(cd->dev, cd->ic_data->event_size, GFP_KERNEL);
	if (!cd->event_buffer)
		return -ENOMEM;

	cd->reset_gpio =
		devm_gpiod_get_optional(cd->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(cd->reset_gpio))
		return dev_err_probe(cd->dev, PTR_ERR(cd->reset_gpio),
				     "Failed to request reset GPIO\n");

	cd->avdd = devm_regulator_get(cd->dev, "avdd");
	if (IS_ERR(cd->avdd))
		return dev_err_probe(cd->dev, PTR_ERR(cd->avdd),
				     "Failed to request AVDD regulator\n");

	cd->vddio = devm_regulator_get(cd->dev, "vddio");
	if (IS_ERR(cd->vddio))
		return dev_err_probe(cd->dev, PTR_ERR(cd->vddio),
				     "Failed to request VDDIO regulator\n");

	error = goodix_gtx8_power_on(cd);
	if (error) {
		dev_err(cd->dev, "failed power on");
		return error;
	}

	error = devm_add_action_or_reset(cd->dev, goodix_gtx8_power_off_act,
					 cd);
	if (error)
		return error;

	error = goodix_gtx8_read_version(cd);
	if (error) {
		dev_err(cd->dev, "failed to get version info");
		return error;
	}

	error = goodix_gtx8_load_config(cd);
	if (error)
		return error;

	goodix_gtx8_init_config(cd);

	error = goodix_gtx8_input_dev_config(cd);
	if (error) {
		dev_err(cd->dev, "failed to set input device");
		return error;
	}

	error = devm_request_threaded_irq(cd->dev, cd->irq, NULL,
					  goodix_gtx8_irq, IRQF_ONESHOT,
					  "goodix-gtx8", cd);
	if (error) {
		dev_err(cd->dev, "request threaded IRQ failed: %d\n", error);
		return error;
	}

	/*
	 * Prime the event path once after config upload without clearing
	 * status.
	 */
	mutex_lock(&cd->event_lock);
	error = goodix_gtx8_read_event_head(cd);
	mutex_unlock(&cd->event_lock);
	if (error)
		dev_warn(cd->dev, "failed to prime event path: %d\n", error);

	dev_set_drvdata(cd->dev, cd);

	error = devm_device_add_group(cd->dev, &goodix_gtx8_attr_group);
	if (error)
		return error;

	dev_dbg(cd->dev,
		"Goodix GT%c%c%c%c Touchscreen Controller, Version %d.%d.%d.%d\n",
		cd->fw_version.product_id[0], cd->fw_version.product_id[1],
		cd->fw_version.product_id[2], cd->fw_version.product_id[3],
		cd->fw_version.fw_version[0], cd->fw_version.fw_version[1],
		cd->fw_version.fw_version[2], cd->fw_version.fw_version[3]);

	return 0;
}

static const struct goodix_gtx8_ic_data gt9886_data = {
	.event_size = GOODIX_GTX8_EVENT_SIZE_NORMANDY,
	.fw_version_addr = GOODIX_GTX8_FW_VERSION_ADDR_NORMANDY,
	.header_size = GOODIX_GTX8_HEADER_SIZE_NORMANDY,
	.ic_type = IC_TYPE_NORMANDY,
	.pre_read_size = GOODIX_GTX8_HEADER_SIZE_NORMANDY +
			 GOODIX_GTX8_TOUCH_SIZE +
			 GOODIX_GTX8_CHECKSUM_SIZE,
	.product_id = { '9', '8', '8', '6' },
	.touch_data_addr = GOODIX_GTX8_TOUCH_DATA_ADDR_NORMANDY,
};

static const struct goodix_gtx8_ic_data gt9896_data = {
	.config_fallback = goodix_gt9896_cfg_bin,
	.config_fallback_size = sizeof(goodix_gt9896_cfg_bin),
	.config_name = "goodix_gt9896_cfg.bin",
	.event_size = GOODIX_GTX8_EVENT_SIZE_YELLOWSTONE,
	.fw_version_addr = GOODIX_GTX8_FW_VERSION_ADDR_YELLOWSTONE,
	.header_size = GOODIX_GTX8_HEADER_SIZE_YELLOWSTONE,
	.ic_type = IC_TYPE_YELLOWSTONE,
	.pre_read_size = GOODIX_GTX8_HEADER_SIZE_YELLOWSTONE +
			 GOODIX_GTX8_TOUCH_SIZE +
			 GOODIX_GTX8_CHECKSUM_SIZE + 1,
	.product_id = { '9', '8', '9', '6' },
	.touch_data_addr = GOODIX_GTX8_TOUCH_DATA_ADDR_YELLOWSTONE,
};

static const struct i2c_device_id goodix_gtx8_i2c_id[] = {
	{ .name = "gt9886", .driver_data = (long)&gt9886_data },
	{ .name = "gt9896", .driver_data = (long)&gt9896_data },
	{},
};
MODULE_DEVICE_TABLE(i2c, goodix_gtx8_i2c_id);

static const struct of_device_id goodix_gtx8_of_match[] = {
	{ .compatible = "goodix,gt9886", .data = &gt9886_data },
	{ .compatible = "goodix,gt9896", .data = &gt9896_data },
	{},
};
MODULE_DEVICE_TABLE(of, goodix_gtx8_of_match);

static struct i2c_driver goodix_gtx8_driver = {
	.probe = goodix_gtx8_probe,
	.id_table = goodix_gtx8_i2c_id,
	.driver = {
		.name = "goodix-gtx8",
		.of_match_table = of_match_ptr(goodix_gtx8_of_match),
		.pm = pm_sleep_ptr(&goodix_gtx8_pm_ops),
	},
};
module_i2c_driver(goodix_gtx8_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Goodix GTX8 Touchscreen driver");
MODULE_AUTHOR("Jens Reidel <adrian@mainlining.org>");
