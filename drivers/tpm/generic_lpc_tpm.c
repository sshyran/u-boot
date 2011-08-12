/* Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * The code in this file has been heavily based on the article "Writing a TPM
 * Device Driver" published on http://ptgmedia.pearsoncmg.com and the
 * submission by Stefan Berger on Qemu-devel mailing list.
 *
 * One principal difference is that in the simplest config the other than 0
 * TPM localities do not get mapped by some devices (for instance, by
 * Infineon slb9635), so this driver provides access to locality 0 only.
 */

/* #define DEBUG */
#include <common.h>
#include <asm/io.h>
#include <tpm.h>

#ifdef DEBUG
#define TPM_DEBUG_ON	1
#else
#define TPM_DEBUG_ON	0
#endif

#define PREFIX "lpc_tpm: "
#define	TPM_DEBUG(fmt, args...)		\
	if (TPM_DEBUG_ON) {		\
		printf(PREFIX);		\
		printf(fmt , ##args);	\
	}

#ifndef CONFIG_TPM_TIS_BASE_ADDRESS
/* Base TPM address standard for x86 systems */
#define CONFIG_TPM_TIS_BASE_ADDRESS        0xfed40000
#endif

/* the macro accepts the locality value, but only locality 0 is operational */
#define TIS_REG(LOCALITY, REG) \
    (void *)(CONFIG_TPM_TIS_BASE_ADDRESS + (LOCALITY << 12) + REG)

/* hardware registers' offsets */
#define TIS_REG_ACCESS                 0x0
#define TIS_REG_INT_ENABLE             0x8
#define TIS_REG_INT_VECTOR             0xc
#define TIS_REG_INT_STATUS             0x10
#define TIS_REG_INTF_CAPABILITY        0x14
#define TIS_REG_STS                    0x18
#define TIS_REG_DATA_FIFO              0x24
#define TIS_REG_DID_VID                0xf00
#define TIS_REG_RID                    0xf04

/* some registers' bit field definitions */
#define TIS_STS_VALID                  (1 << 7) /* 0x80 */
#define TIS_STS_COMMAND_READY          (1 << 6) /* 0x40 */
#define TIS_STS_TPM_GO                 (1 << 5) /* 0x20 */
#define TIS_STS_DATA_AVAILABLE         (1 << 4) /* 0x10 */
#define TIS_STS_EXPECT                 (1 << 3) /* 0x08 */
#define TIS_STS_RESPONSE_RETRY         (1 << 1) /* 0x02 */

#define TIS_ACCESS_TPM_REG_VALID_STS   (1 << 7) /* 0x80 */
#define TIS_ACCESS_ACTIVE_LOCALITY     (1 << 5) /* 0x20 */
#define TIS_ACCESS_BEEN_SEIZED         (1 << 4) /* 0x10 */
#define TIS_ACCESS_SEIZE               (1 << 3) /* 0x08 */
#define TIS_ACCESS_PENDING_REQUEST     (1 << 2) /* 0x04 */
#define TIS_ACCESS_REQUEST_USE         (1 << 1) /* 0x02 */
#define TIS_ACCESS_TPM_ESTABLISHMENT   (1 << 0) /* 0x01 */

/* Maximal time to wait for the TPM chip to complete an operation */
#define TPM_MAX_EXECUTION_DELAY_MS 1000
/*
 * Structures defined below allow creating descriptions of TPM vendor/device
 * ID information for run time discovery. The only device the system knows
 * about at this time is Infineon slb9635
 */
struct device_name {
	u16 dev_id;
	const char * const dev_name;
};

struct vendor_name {
	u16 vendor_id;
	const char * vendor_name;
	struct device_name* dev_names;
};

static struct device_name infineon_devices[] = {
	{0xb, "SLB9635 TT 1.2"},
	{0}
};

static const struct vendor_name vendor_names[] = {
	{0x15d1, "Infineon", infineon_devices},
};

/*
 * Cached vendor/device ID pair to indicate that the device has been already
 * discovered
 */
static u32 vendor_dev_id;

static int is_byte_reg(u32 reg)
{
	/*
	 * These TPM registers are 8 bits wide and as such require byte access
	 * on writes and truncated value on reads.
	 */
	return ((reg == TIS_REG_ACCESS)	||
		(reg == TIS_REG_INT_VECTOR) ||
		(reg == TIS_REG_DATA_FIFO));
}

/* TPM access functions are carved out to make tracing easier. */
static u32 tpm_read(int locality, u32 reg)
{
	u32 value;
	value = readl(TIS_REG(locality, reg));
	if (is_byte_reg(reg))
		value &= 0xff;
	TPM_DEBUG("Read reg 0x%x returns 0x%x\n", reg, value);
	return value;
}

static void tpm_write(u32 value, int locality,  u32 reg)
{
	TPM_DEBUG("Write reg 0x%x with 0x%x\n", reg, value);

	if (is_byte_reg(reg))
		writeb(value & 0xff, TIS_REG(locality, reg));
	else
		writel(value, TIS_REG(locality, reg));
}

/*
 * tis_wait_reg()
 *
 * Wait for a register to change its state to match the expected state.
 *
 * @reg - the TPM register offset
 * @locality - locality
 * @time_ms - max time to wait, in ms
 * @mask - bitmask for the bitfield(s) to watch
 * @expected - value the field(s) are supposed to be set to
 *
 * Returns 0 in case the xpected value was present in the appropriate register
 * bits, or ~0 on timeout.
 */
static u32 tis_wait_reg(u8 reg, u8 locality, u32 time_ms, u8 mask, u8 expected)
{
	while (time_ms > 0) {
		u32 value = tpm_read(locality, reg);
		if ((value & mask) == expected)
			return 0;
		udelay(1000); /* 1 ms */
		time_ms--;
	}
	return ~0;
}

/*
 * Probe the TPM device and try determining its manufacturer/device name.
 *
 * Returns 0 on success (the device is found or was found during an earlier
 * invocation) or ~0 if the device is not found.
 */
static u32 tis_probe(void)
{
	u32 didvid = tpm_read(0, TIS_REG_DID_VID);
	int i;
	const char *device_name = "unknown";
	const char *vendor_name = device_name;
	u16 vid, did;

	if (vendor_dev_id)
		return 0;  /* Already probed. */

	if (!didvid || (didvid == 0xffffffff)) {
		printf("%s: No TPM device found\n", __FUNCTION__);
		return ~0;
	}

	vendor_dev_id = didvid;

	vid = didvid & 0xffff;
	did = (didvid >> 16) & 0xffff;
	for (i = 0; i < ARRAY_SIZE(vendor_names); i++) {
		int j = 0;
		u16 known_did;
		if (vid == vendor_names[i].vendor_id) {
			vendor_name = vendor_names[i].vendor_name;
		}
		while ((known_did = vendor_names[i].dev_names[j].dev_id) != 0) {
			if (known_did == did) {
				device_name =
					vendor_names[i].dev_names[j].dev_name;
				break;
			}
			j++;
		}
		break;
	}
	/* this will have to be converted into debug printout */
	TPM_DEBUG("Found TPM %s by %s\n", device_name, vendor_name);
	return 0;
}

/*
 * tis_senddata()
 *
 * send the passed in data to the TPM device.
 *
 * @data - address of the data to send, byte by byte
 * @len - length of the data to send
 *
 * Returns 0 on success, ~0 on error (in case the device does not accept the
 * entire comand).
 */
static u32 tis_senddata(const u8 * const data, u32 len)
{
	u32 rc = 0;
	u32 offset = 0;
	u16 burst = 0;
	u32 ctr = 0;
	u8 locality = 0;

	while (1) {
		while (!burst && (ctr < 2000)) {
			burst = (u16) (tpm_read(locality, TIS_REG_STS) >> 8);
			if (!burst) {
				udelay(100);
				ctr++;
			}
		}

		if (!burst) {
			rc = ~0;
			break;
		}

		while (1) {
			tpm_write(data[offset++], locality, TIS_REG_DATA_FIFO);
			burst--;

			if (burst == 0 || offset == len)
				break;
		}

		if (offset == len)
			break;
	}
	return rc;
}

/*
 * tis_readresponse()
 *
 * read the TPM device response after a command was issued.
 *
 * @buffer - address where to read the response, byte by byte.
 * @len - pointer to the size of buffer
 *
 * Does not yet generate any errors, always returns 0. len is used to report
 * the number of actually read response bytes.
 */
static u32 tis_readresponse(u8 *buffer, u32 *len)
{
	u32 offset = 0;
	u8 locality = 0;

	while ((tpm_read(locality, TIS_REG_STS) & TIS_STS_DATA_AVAILABLE) &&
		(offset < *len))
		buffer[offset++] = (u8) tpm_read(locality, TIS_REG_DATA_FIFO);

	*len = offset;
	return 0;
}

/*
 * tis_init()
 *
 * Initialize the TPM device. Returns 0 on success or ~0 on failure (in case
 * device probing did not succeed).
 */
int tis_init(void)
{
	if (tis_probe())
		return ~0;
	return 0;
}

/*
 * tis_open()
 *
 * Requests access to locality 0 for the caller. After all commands have been
 * completed the caller is supposed to call tis_close().
 *
 * Returns 0 on success, ~0 on failure.
 */
int tis_open(void)
{
	u8 locality = 0; /* we use locality zero for everything */

	if (tis_close())
		return ~0;

	/* now request access to locality */
	tpm_write(TIS_ACCESS_REQUEST_USE, locality, TIS_REG_ACCESS);

	/* did we get a lock? */
	if (tis_wait_reg(TIS_REG_ACCESS, locality, TPM_MAX_EXECUTION_DELAY_MS,
			 TIS_ACCESS_ACTIVE_LOCALITY,
			 TIS_ACCESS_ACTIVE_LOCALITY)) {
		printf("%s:%d - failed to lock locality %d\n",
		       __FILE__, __LINE__, locality);
		return ~0;
	}

	tpm_write(TIS_STS_COMMAND_READY, locality, TIS_REG_STS);
	if (tis_wait_reg(TIS_REG_STS, locality, TPM_MAX_EXECUTION_DELAY_MS,
			 TIS_STS_COMMAND_READY, TIS_STS_COMMAND_READY)) {
		printf("%s:%d - failed to get 'command_ready' status\n",
		       __FILE__, __LINE__);
		return ~0;
	}
	return 0;
}

/*
 * tis_close()
 *
 * terminate the currect session with the TPM bu releasing the locked
 * locality. Returns 0 on success of ~0 on failure (in case lock removal did
 * not succeed).
 */
int tis_close(void)
{
	u8 locality = 0;
	if (tpm_read(locality, TIS_REG_ACCESS) &
	    TIS_ACCESS_ACTIVE_LOCALITY) {
		tpm_write(TIS_ACCESS_ACTIVE_LOCALITY, locality, TIS_REG_ACCESS);

		if (tis_wait_reg(TIS_REG_ACCESS, locality,
				 TPM_MAX_EXECUTION_DELAY_MS,
				 TIS_ACCESS_ACTIVE_LOCALITY, 0)) {
			printf("%s:%d - failed to release locality %d\n",
			       __FILE__, __LINE__, locality);
			return ~0;
		}
	}
	return 0;
}

/*
 * tis_sendrecv()
 *
 * Send the requested data to the TPM and then try to get its response
 *
 * @sendbuf - buffer of the data to send
 * @send_size size of the data to send
 * @recvbuf - memory to save the response to
 * @recv_len - pointer to the size of the response buffer
 *
 * Returns 0 on success (and places the number of response bytes at recv_len)
 * or ~0 on failure.
 */
int tis_sendrecv(const uint8_t *sendbuf, size_t send_size,
		 uint8_t *recvbuf, size_t *recv_len)
{
	if (tis_senddata(sendbuf, send_size)) {
		printf("%s:%d failed sending data to TPM\n",
		       __FILE__, __LINE__);
		return ~0;
	}

	tis_readresponse(recvbuf, recv_len);
	return 0;
}
