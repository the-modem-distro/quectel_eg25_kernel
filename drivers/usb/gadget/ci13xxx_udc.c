/*
 * ci13xxx_udc.c - MIPS USB IP core family device controller
 *
 * Copyright (C) 2008 Chipidea - MIPS Technologies, Inc. All rights reserved.
 *
 * Author: David Lopo
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * Description: MIPS USB IP core family device controller
 *              Currently it only supports IP part number CI13412
 *
 * This driver is composed of several blocks:
 * - HW:     hardware interface
 * - DBG:    debug facilities (optional)
 * - UTIL:   utilities
 * - ISR:    interrupts handling
 * - ENDPT:  endpoint operations (Gadget API)
 * - GADGET: gadget operations (Gadget API)
 * - BUS:    bus glue code, bus abstraction layer
 *
 * Compile Options
 * - CONFIG_USB_GADGET_DEBUG_FILES: enable debug facilities
 * - STALL_IN:  non-empty bulk-in pipes cannot be halted
 *              if defined mass storage compliance succeeds but with warnings
 *              => case 4: Hi >  Dn
 *              => case 5: Hi >  Di
 *              => case 8: Hi <> Do
 *              if undefined usbtest 13 fails
 * - TRACE:     enable function tracing (depends on DEBUG)
 *
 * Main Features
 * - Chapter 9 & Mass Storage Compliance with Gadget File Storage
 * - Chapter 9 Compliance with Gadget Zero (STALL_IN undefined)
 * - Normal & LPM support
 *
 * USBTEST Report
 * - OK: 0-12, 13 (STALL_IN defined) & 14
 * - Not Supported: 15 & 16 (ISO)
 *
 * TODO List
 * - OTG
 * - Isochronous & Interrupt Traffic
 * - Handle requests which spawns into several TDs
 * - GET_STATUS(device) - always reports 0
 * - Gadget API (majority of optional features)
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/msm_hsusb.h>

#include "ci13xxx_udc.h"

/******************************************************************************
 * DEFINE
 *****************************************************************************/

#define USB_MAX_TIMEOUT		25 /* 25msec timeout */
#define EP_PRIME_CHECK_DELAY	(jiffies + msecs_to_jiffies(1000))
#define MAX_PRIME_CHECK_RETRY	3 /*Wait for 3sec for EP prime failure */
#define EXTRA_ALLOCATION_SIZE	256

/* ctrl register bank access */
static DEFINE_SPINLOCK(udc_lock);

/* control endpoint description */
static const struct usb_endpoint_descriptor
ctrl_endpt_out_desc = {
	.bLength         = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes    = USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize  = cpu_to_le16(CTRL_PAYLOAD_MAX),
};

static const struct usb_endpoint_descriptor
ctrl_endpt_in_desc = {
	.bLength         = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes    = USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize  = cpu_to_le16(CTRL_PAYLOAD_MAX),
};

/* UDC descriptor */
static struct ci13xxx *_udc;


/**
 * ffs_nr: find first (least significant) bit set
 * @x: the word to search
 *
 * This function returns bit number (instead of position)
 */
static int ffs_nr(u32 x)
{
	int n = ffs(x);

	return n ? n-1 : 32;
}

/******************************************************************************
 * HW block
 *****************************************************************************/
/* register bank descriptor */
static struct {
	unsigned int  lpm;    /* is LPM? */
	void __iomem *abs;    /* bus map offset */
	void __iomem *cap;    /* bus map offset + CAP offset + CAP data */
	size_t        size;   /* bank size */
} hw_bank;

/* MSM specific */
#define ABS_AHBBURST        (0x0090UL)
#define ABS_AHBMODE         (0x0098UL)
/* UDC register map */
#define ABS_CAPLENGTH       (0x100UL)
#define ABS_HCCPARAMS       (0x108UL)
#define ABS_DCCPARAMS       (0x124UL)
#define ABS_TESTMODE        (hw_bank.lpm ? 0x0FCUL : 0x138UL)
/* offset to CAPLENTGH (addr + data) */
#define CAP_USBCMD          (0x000UL)
#define CAP_USBSTS          (0x004UL)
#define CAP_USBINTR         (0x008UL)
#define CAP_DEVICEADDR      (0x014UL)
#define CAP_ENDPTLISTADDR   (0x018UL)
#define CAP_PORTSC          (0x044UL)
#define CAP_DEVLC           (0x084UL)
#define CAP_ENDPTPIPEID     (0x0BCUL)
#define CAP_USBMODE         (hw_bank.lpm ? 0x0C8UL : 0x068UL)
#define CAP_ENDPTSETUPSTAT  (hw_bank.lpm ? 0x0D8UL : 0x06CUL)
#define CAP_ENDPTPRIME      (hw_bank.lpm ? 0x0DCUL : 0x070UL)
#define CAP_ENDPTFLUSH      (hw_bank.lpm ? 0x0E0UL : 0x074UL)
#define CAP_ENDPTSTAT       (hw_bank.lpm ? 0x0E4UL : 0x078UL)
#define CAP_ENDPTCOMPLETE   (hw_bank.lpm ? 0x0E8UL : 0x07CUL)
#define CAP_ENDPTCTRL       (hw_bank.lpm ? 0x0ECUL : 0x080UL)
#define CAP_LAST            (hw_bank.lpm ? 0x12CUL : 0x0C0UL)

#define REMOTE_WAKEUP_DELAY	msecs_to_jiffies(200)

/* maximum number of enpoints: valid only after hw_device_reset() */
static unsigned int hw_ep_max;

/**
 * hw_ep_bit: calculates the bit number
 * @num: endpoint number
 * @dir: endpoint direction
 *
 * This function returns bit number
 */
static inline int hw_ep_bit(int num, int dir)
{
	return num + (dir ? 16 : 0);
}

static int ep_to_bit(int n)
{
	int fill = 16 - hw_ep_max / 2;

	if (n >= hw_ep_max / 2)
		n += fill;

	return n;
}

/**
 * hw_aread: reads from register bitfield
 * @addr: address relative to bus map
 * @mask: bitfield mask
 *
 * This function returns register bitfield data
 */
static u32 hw_aread(u32 addr, u32 mask)
{
	return ioread32(addr + hw_bank.abs) & mask;
}

/**
 * hw_awrite: writes to register bitfield
 * @addr: address relative to bus map
 * @mask: bitfield mask
 * @data: new data
 */
static void hw_awrite(u32 addr, u32 mask, u32 data)
{
	iowrite32(hw_aread(addr, ~mask) | (data & mask),
		  addr + hw_bank.abs);
}

/**
 * hw_cread: reads from register bitfield
 * @addr: address relative to CAP offset plus content
 * @mask: bitfield mask
 *
 * This function returns register bitfield data
 */
static u32 hw_cread(u32 addr, u32 mask)
{
	return ioread32(addr + hw_bank.cap) & mask;
}

/**
 * hw_cwrite: writes to register bitfield
 * @addr: address relative to CAP offset plus content
 * @mask: bitfield mask
 * @data: new data
 */
static void hw_cwrite(u32 addr, u32 mask, u32 data)
{
	iowrite32(hw_cread(addr, ~mask) | (data & mask),
		  addr + hw_bank.cap);
}

/**
 * hw_ctest_and_clear: tests & clears register bitfield
 * @addr: address relative to CAP offset plus content
 * @mask: bitfield mask
 *
 * This function returns register bitfield data
 */
static u32 hw_ctest_and_clear(u32 addr, u32 mask)
{
	u32 reg = hw_cread(addr, mask);

	iowrite32(reg, addr + hw_bank.cap);
	return reg;
}

/**
 * hw_ctest_and_write: tests & writes register bitfield
 * @addr: address relative to CAP offset plus content
 * @mask: bitfield mask
 * @data: new data
 *
 * This function returns register bitfield data
 */
static u32 hw_ctest_and_write(u32 addr, u32 mask, u32 data)
{
	u32 reg = hw_cread(addr, ~0);

	iowrite32((reg & ~mask) | (data & mask), addr + hw_bank.cap);
	return (reg & mask) >> ffs_nr(mask);
}

static int hw_device_init(void __iomem *base)
{
	u32 reg;

	/* bank is a module variable */
	hw_bank.abs = base;

	hw_bank.cap = hw_bank.abs;
	hw_bank.cap += ABS_CAPLENGTH;
	hw_bank.cap += ioread8(hw_bank.cap);

	reg = hw_aread(ABS_HCCPARAMS, HCCPARAMS_LEN) >> ffs_nr(HCCPARAMS_LEN);
	hw_bank.lpm  = reg;
	hw_bank.size = hw_bank.cap - hw_bank.abs;
	hw_bank.size += CAP_LAST;
	hw_bank.size /= sizeof(u32);

	reg = hw_aread(ABS_DCCPARAMS, DCCPARAMS_DEN) >> ffs_nr(DCCPARAMS_DEN);
	hw_ep_max = reg * 2;   /* cache hw ENDPT_MAX */

	if (hw_ep_max == 0 || hw_ep_max > ENDPT_MAX)
		return -ENODEV;

	/* setup lock mode ? */

	/* ENDPTSETUPSTAT is '0' by default */

	/* HCSPARAMS.bf.ppc SHOULD BE zero for device */

	return 0;
}
/**
 * hw_device_reset: resets chip (execute without interruption)
 * @base: register base address
 *
 * This function returns an error code
 */
static int hw_device_reset(struct ci13xxx *udc)
{
	int delay_count = 25; /* 250 usec */

	/* should flush & stop before reset */
	hw_cwrite(CAP_ENDPTFLUSH, ~0, ~0);
	hw_cwrite(CAP_USBCMD, USBCMD_RS, 0);

	hw_cwrite(CAP_USBCMD, USBCMD_RST, USBCMD_RST);
	while (delay_count--  && hw_cread(CAP_USBCMD, USBCMD_RST))
		udelay(10);
	if (delay_count < 0)
		pr_err("USB controller reset failed\n");

	if (udc->udc_driver->notify_event)
		udc->udc_driver->notify_event(udc,
			CI13XXX_CONTROLLER_RESET_EVENT);

	/* USBMODE should be configured step by step */
	hw_cwrite(CAP_USBMODE, USBMODE_CM, USBMODE_CM_IDLE);
	hw_cwrite(CAP_USBMODE, USBMODE_CM, USBMODE_CM_DEVICE);
	hw_cwrite(CAP_USBMODE, USBMODE_SLOM, USBMODE_SLOM);  /* HW >= 2.3 */

	/*
	 * ITC (Interrupt Threshold Control) field is to set the maximum
	 * rate at which the device controller will issue interrupts.
	 * The maximum interrupt interval measured in micro frames.
	 * Valid values are 0, 1, 2, 4, 8, 16, 32, 64. The default value is
	 * 8 micro frames. If CPU can handle interrupts at faster rate, ITC
	 * can be set to lesser value to gain performance.
	 */
	if (udc->udc_driver->nz_itc)
		hw_cwrite(CAP_USBCMD, USBCMD_ITC_MASK,
			USBCMD_ITC(udc->udc_driver->nz_itc));
	else if (udc->udc_driver->flags & CI13XXX_ZERO_ITC)
		hw_cwrite(CAP_USBCMD, USBCMD_ITC_MASK, USBCMD_ITC(0));

	if (hw_cread(CAP_USBMODE, USBMODE_CM) != USBMODE_CM_DEVICE) {
		pr_err("cannot enter in device mode");
		pr_err("lpm = %i", hw_bank.lpm);
		return -ENODEV;
	}

	return 0;
}

/**
 * hw_device_state: enables/disables interrupts & starts/stops device (execute
 *                  without interruption)
 * @dma: 0 => disable, !0 => enable and set dma engine
 *
 * This function returns an error code
 */
static int hw_device_state(u32 dma)
{
	struct ci13xxx *udc = _udc;

	if (dma) {
		if (!(udc->udc_driver->flags & CI13XXX_DISABLE_STREAMING)) {
			hw_cwrite(CAP_USBMODE, USBMODE_SDIS, 0);
			pr_debug("%s(): streaming mode is enabled. USBMODE:%x\n",
				 __func__, hw_cread(CAP_USBMODE, ~0));

		} else {
			hw_cwrite(CAP_USBMODE, USBMODE_SDIS, USBMODE_SDIS);
			pr_debug("%s(): streaming mode is disabled. USBMODE:%x\n",
				__func__, hw_cread(CAP_USBMODE, ~0));
		}

		hw_cwrite(CAP_ENDPTLISTADDR, ~0, dma);


		/* Set BIT(31) to enable AHB2AHB Bypass functionality */
		if (udc->udc_driver->flags & CI13XXX_ENABLE_AHB2AHB_BYPASS) {
			hw_awrite(ABS_AHBMODE, AHB2AHB_BYPASS, AHB2AHB_BYPASS);
			pr_debug("%s(): ByPass Mode is enabled. AHBMODE:%x\n",
					__func__, hw_aread(ABS_AHBMODE, ~0));
		}

		/* interrupt, error, port change, reset, sleep/suspend */
		hw_cwrite(CAP_USBINTR, ~0,
			     USBi_UI|USBi_UEI|USBi_PCI|USBi_URI|USBi_SLI);
		hw_cwrite(CAP_USBCMD, USBCMD_RS, USBCMD_RS);
		udc->transceiver->flags |= PHY_SOFT_CONNECT;
	} else {
		udc->transceiver->flags &= ~PHY_SOFT_CONNECT;
		hw_cwrite(CAP_USBCMD, USBCMD_RS, 0);
		hw_cwrite(CAP_USBINTR, ~0, 0);
		/* Clear BIT(31) to disable AHB2AHB Bypass functionality */
		if (udc->udc_driver->flags & CI13XXX_ENABLE_AHB2AHB_BYPASS) {
			hw_awrite(ABS_AHBMODE, AHB2AHB_BYPASS, 0);
			pr_debug("%s(): ByPass Mode is disabled. AHBMODE:%x\n",
					__func__, hw_aread(ABS_AHBMODE, ~0));
		}
	}
	return 0;
}

static void debug_ept_flush_info(int ep_num, int dir)
{
	struct ci13xxx *udc = _udc;
	struct ci13xxx_ep *mep;

	if (dir)
		mep = &udc->ci13xxx_ep[ep_num + hw_ep_max/2];
	else
		mep = &udc->ci13xxx_ep[ep_num];

	pr_err_ratelimited("USB Registers\n");
	pr_err_ratelimited("USBCMD:%x\n", hw_cread(CAP_USBCMD, ~0));
	pr_err_ratelimited("USBSTS:%x\n", hw_cread(CAP_USBSTS, ~0));
	pr_err_ratelimited("ENDPTLISTADDR:%x\n",
			hw_cread(CAP_ENDPTLISTADDR, ~0));
	pr_err_ratelimited("PORTSC:%x\n", hw_cread(CAP_PORTSC, ~0));
	pr_err_ratelimited("USBMODE:%x\n", hw_cread(CAP_USBMODE, ~0));
	pr_err_ratelimited("ENDPTSTAT:%x\n", hw_cread(CAP_ENDPTSTAT, ~0));

}
/**
 * hw_ep_flush: flush endpoint fifo (execute without interruption)
 * @num: endpoint number
 * @dir: endpoint direction
 *
 * This function returns an error code
 */
static int hw_ep_flush(int num, int dir)
{
	ktime_t start, diff;
	int n = hw_ep_bit(num, dir);
	struct ci13xxx_ep *mEp = &_udc->ci13xxx_ep[n];

	/* Flush ep0 even when queue is empty */
	if (_udc->skip_flush || (num && list_empty(&mEp->qh.queue)))
		return 0;

	start = ktime_get();
	do {
		/* flush any pending transfer */
		hw_cwrite(CAP_ENDPTFLUSH, BIT(n), BIT(n));
		while (hw_cread(CAP_ENDPTFLUSH, BIT(n))) {
			cpu_relax();
			diff = ktime_sub(ktime_get(), start);
			if (ktime_to_ms(diff) > USB_MAX_TIMEOUT) {
				printk_ratelimited(KERN_ERR
					"%s: Failed to flush ep#%d %s\n",
					__func__, num,
					dir ? "IN" : "OUT");
				debug_ept_flush_info(num, dir);
				_udc->skip_flush = true;
				/* Notify to trigger h/w reset recovery later */
				if (_udc->udc_driver->notify_event)
					_udc->udc_driver->notify_event(_udc,
						CI13XXX_CONTROLLER_ERROR_EVENT);
				return 0;
			}
		}
	} while (hw_cread(CAP_ENDPTSTAT, BIT(n)));

	return 0;
}

/**
 * hw_ep_disable: disables endpoint (execute without interruption)
 * @num: endpoint number
 * @dir: endpoint direction
 *
 * This function returns an error code
 */
static int hw_ep_disable(int num, int dir)
{
	hw_cwrite(CAP_ENDPTCTRL + num * sizeof(u32),
		  dir ? ENDPTCTRL_TXE : ENDPTCTRL_RXE, 0);
	return 0;
}

/**
 * hw_ep_enable: enables endpoint (execute without interruption)
 * @num:  endpoint number
 * @dir:  endpoint direction
 * @type: endpoint type
 *
 * This function returns an error code
 */
static int hw_ep_enable(int num, int dir, int type)
{
	u32 mask, data;

	if (dir) {
		mask  = ENDPTCTRL_TXT;  /* type    */
		data  = type << ffs_nr(mask);

		mask |= ENDPTCTRL_TXS;  /* unstall */
		mask |= ENDPTCTRL_TXR;  /* reset data toggle */
		data |= ENDPTCTRL_TXR;
		mask |= ENDPTCTRL_TXE;  /* enable  */
		data |= ENDPTCTRL_TXE;
	} else {
		mask  = ENDPTCTRL_RXT;  /* type    */
		data  = type << ffs_nr(mask);

		mask |= ENDPTCTRL_RXS;  /* unstall */
		mask |= ENDPTCTRL_RXR;  /* reset data toggle */
		data |= ENDPTCTRL_RXR;
		mask |= ENDPTCTRL_RXE;  /* enable  */
		data |= ENDPTCTRL_RXE;
	}
	hw_cwrite(CAP_ENDPTCTRL + num * sizeof(u32), mask, data);

	/* make sure endpoint is enabled before returning */
	mb();

	return 0;
}

/**
 * hw_ep_get_halt: return endpoint halt status
 * @num: endpoint number
 * @dir: endpoint direction
 *
 * This function returns 1 if endpoint halted
 */
static int hw_ep_get_halt(int num, int dir)
{
	u32 mask = dir ? ENDPTCTRL_TXS : ENDPTCTRL_RXS;

	return hw_cread(CAP_ENDPTCTRL + num * sizeof(u32), mask) ? 1 : 0;
}

/**
 * hw_test_and_clear_setup_status: test & clear setup status (execute without
 *                                 interruption)
 * @n: endpoint number
 *
 * This function returns setup status
 */
static int hw_test_and_clear_setup_status(int n)
{
	n = ep_to_bit(n);
	return hw_ctest_and_clear(CAP_ENDPTSETUPSTAT, BIT(n));
}

/**
 * hw_ep_prime: primes endpoint (execute without interruption)
 * @num:     endpoint number
 * @dir:     endpoint direction
 * @is_ctrl: true if control endpoint
 *
 * This function returns an error code
 */
static int hw_ep_prime(int num, int dir, int is_ctrl)
{
	int n = hw_ep_bit(num, dir);

	if (is_ctrl && dir == RX && hw_cread(CAP_ENDPTSETUPSTAT, BIT(num)))
		return -EAGAIN;

	hw_cwrite(CAP_ENDPTPRIME, BIT(n), BIT(n));

	if (is_ctrl && dir == RX  && hw_cread(CAP_ENDPTSETUPSTAT, BIT(num)))
		return -EAGAIN;

	/* status shoult be tested according with manual but it doesn't work */
	return 0;
}

/**
 * hw_ep_set_halt: configures ep halt & resets data toggle after clear (execute
 *                 without interruption)
 * @num:   endpoint number
 * @dir:   endpoint direction
 * @value: true => stall, false => unstall
 *
 * This function returns an error code
 */
static int hw_ep_set_halt(int num, int dir, int value)
{
	u32 addr, mask_xs, mask_xr;

	if (value != 0 && value != 1)
		return -EINVAL;

	do {
		if (hw_cread(CAP_ENDPTSETUPSTAT, BIT(num)))
			return 0;

		addr = CAP_ENDPTCTRL + num * sizeof(u32);
		mask_xs = dir ? ENDPTCTRL_TXS : ENDPTCTRL_RXS;
		mask_xr = dir ? ENDPTCTRL_TXR : ENDPTCTRL_RXR;

		/* data toggle - reserved for EP0 but it's in ESS */
		hw_cwrite(addr, mask_xs|mask_xr, value ? mask_xs : mask_xr);

	} while (value != hw_ep_get_halt(num, dir));

	return 0;
}

/**
 * hw_is_port_high_speed: test if port is high speed
 *
 * This function returns true if high speed port
 */
static int hw_port_is_high_speed(void)
{
	return hw_bank.lpm ? hw_cread(CAP_DEVLC, DEVLC_PSPD) :
		hw_cread(CAP_PORTSC, PORTSC_HSP);
}


/**
 * hw_port_test_set: writes port test mode (execute without interruption)
 * @mode: new value
 *
 * This function returns an error code
 */
static int hw_port_test_set(u8 mode)
{
	const u8 TEST_MODE_MAX = 7;

	if (mode > TEST_MODE_MAX)
		return -EINVAL;

	hw_cwrite(CAP_PORTSC, PORTSC_PTC, mode << ffs_nr(PORTSC_PTC));
	return 0;
}

/**
 * hw_read_intr_enable: returns interrupt enable register
 *
 * This function returns register data
 */
static u32 hw_read_intr_enable(void)
{
	return hw_cread(CAP_USBINTR, ~0);
}

/**
 * hw_read_intr_status: returns interrupt status register
 *
 * This function returns register data
 */
static u32 hw_read_intr_status(void)
{
	return hw_cread(CAP_USBSTS, ~0);
}

/**
 * hw_test_and_clear_complete: test & clear complete status (execute without
 *                             interruption)
 * @n: endpoint number
 *
 * This function returns complete status
 */
static int hw_test_and_clear_complete(int n)
{
	n = ep_to_bit(n);
	return hw_ctest_and_clear(CAP_ENDPTCOMPLETE, BIT(n));
}

/**
 * hw_test_and_clear_intr_active: test & clear active interrupts (execute
 *                                without interruption)
 *
 * This function returns active interrutps
 */
static u32 hw_test_and_clear_intr_active(void)
{
	u32 reg = hw_read_intr_status() & hw_read_intr_enable();

	hw_cwrite(CAP_USBSTS, ~0, reg);
	return reg;
}

/**
 * hw_test_and_clear_setup_guard: test & clear setup guard (execute without
 *                                interruption)
 *
 * This function returns guard value
 */
static int hw_test_and_clear_setup_guard(void)
{
	return hw_ctest_and_write(CAP_USBCMD, USBCMD_SUTW, 0);
}

/**
 * hw_test_and_set_setup_guard: test & set setup guard (execute without
 *                              interruption)
 *
 * This function returns guard value
 */
static int hw_test_and_set_setup_guard(void)
{
	return hw_ctest_and_write(CAP_USBCMD, USBCMD_SUTW, USBCMD_SUTW);
}

/**
 * hw_usb_set_address: configures USB address (execute without interruption)
 * @value: new USB address
 *
 * This function returns an error code
 */
static int hw_usb_set_address(u8 value)
{
	/* advance */
	hw_cwrite(CAP_DEVICEADDR, DEVICEADDR_USBADR | DEVICEADDR_USBADRA,
		  value << ffs_nr(DEVICEADDR_USBADR) | DEVICEADDR_USBADRA);
	return 0;
}

/**
 * hw_usb_reset: restart device after a bus reset (execute without
 *               interruption)
 *
 * This function returns an error code
 */
static int hw_usb_reset(void)
{
	int delay_count = 10; /* 100 usec delay */

	hw_usb_set_address(0);

	/* ESS flushes only at end?!? */
	hw_cwrite(CAP_ENDPTFLUSH,    ~0, ~0);   /* flush all EPs */

	/* clear complete status */
	hw_cwrite(CAP_ENDPTCOMPLETE,  0,  0);   /* writes its content */

	/* wait until all bits cleared */
	while (delay_count-- && hw_cread(CAP_ENDPTPRIME, ~0))
		udelay(10);
	if (delay_count < 0)
		pr_err("ENDPTPRIME is not cleared during bus reset\n");

	/* reset all endpoints ? */

	/*
	 * reset internal status and wait for further instructions
	 * no need to verify the port reset status (ESS does it)
	 */

	return 0;
}


#define CI_PM_RESUME_RETRIES	5    /* Max Number of retries */

static int ci13xxx_wakeup(struct usb_gadget *_gadget)
{
	struct ci13xxx *udc = container_of(_gadget, struct ci13xxx, gadget);
	unsigned long flags;
	int ret = 0;
	static int retry_count;
	pr_info("%s: Remote wakeup called\n", __func__);
	
	spin_lock_irqsave(udc->lock, flags);
	if (!udc->gadget.remote_wakeup) {
		pr_err("%s: Remote wakeup disabled\n", __func__);
		spin_unlock_irqrestore(udc->lock, flags);
		return -EOPNOTSUPP;
	}

	spin_unlock_irqrestore(udc->lock, flags);

	if (!udc->suspended) {
		pr_err("%s: Was already awake, get out\n", __func__);
		return 0;
	}

	ret = pm_runtime_get_sync(&_gadget->dev);
	if (ret == -EACCES) {
		pr_err("%s: PM runtime get sync failed, ret %d (%i retries)\n", __func__, ret, retry_count);
		pm_runtime_put_noidle(&_gadget->dev);
		if (retry_count == CI_PM_RESUME_RETRIES) {
			pr_err("pm_runtime_get_sync timed out\n");
			retry_count = 0;
			return 0;
		}
		retry_count++;
		schedule_delayed_work(&udc->rw_work, REMOTE_WAKEUP_DELAY);
		return 0;
	}
	if (ret == 1) {
		pr_err("%s: Already up\n", __func__);

	} else if (ret == 0) {
		pr_err("%s: Woke up\n", __func__);
	}

	retry_count = 0;
	udc->udc_driver->notify_event(udc,
		CI13XXX_CONTROLLER_REMOTE_WAKEUP_EVENT);

	if (udc->transceiver)
		usb_phy_set_suspend(udc->transceiver, 0);

	spin_lock_irqsave(udc->lock, flags);
	if (!hw_cread(CAP_PORTSC, PORTSC_SUSP)) {
		printk("%s: PORTSC_SUSP is 0, we're already up!\n", __func__);
		pm_runtime_put(&_gadget->dev);
		spin_unlock_irqrestore(udc->lock, flags);
		return 0;
	}

	printk("%s: Writing to port\n", __func__);
	hw_cwrite(CAP_PORTSC, PORTSC_FPR, PORTSC_FPR);

	pm_runtime_mark_last_busy(&_gadget->dev);
	pm_runtime_put_autosuspend(&_gadget->dev);

	printk("%s: Finished\n", __func__);
	spin_unlock_irqrestore(udc->lock, flags);
	return 0;
}

static void usb_do_remote_wakeup(struct work_struct *w)
{
	struct ci13xxx *udc = _udc;
	unsigned long flags;
	bool do_wake;

	/*
	 * This work can not be canceled from interrupt handler. Check
	 * if wakeup conditions are still met.
	 */
	spin_lock_irqsave(udc->lock, flags);
	do_wake = udc->suspended && udc->gadget.remote_wakeup;
	spin_unlock_irqrestore(udc->lock, flags);

	if (do_wake)
		ci13xxx_wakeup(&udc->gadget);
}

/******************************************************************************
 * UTIL block
 *****************************************************************************/
/**
 * _usb_addr: calculates endpoint address from direction & number
 * @ep:  endpoint
 */
static inline u8 _usb_addr(struct ci13xxx_ep *ep)
{
	return ((ep->dir == TX) ? USB_ENDPOINT_DIR_MASK : 0) | ep->num;
}

static void ep_prime_timer_func(unsigned long data)
{
	struct ci13xxx_ep *mep = (struct ci13xxx_ep *)data;
	struct ci13xxx_req *req;
	struct list_head *ptr = NULL;
	int n = hw_ep_bit(mep->num, mep->dir);
	unsigned long flags;


	spin_lock_irqsave(mep->lock, flags);

	if (_udc && (!_udc->vbus_active || _udc->suspended)) {
		pr_debug("ep%d %s prime timer when vbus_active=%d,suspend=%d\n",
			mep->num, mep->dir ? "IN" : "OUT",
			_udc->vbus_active, _udc->suspended);
		goto out;
	}

	if (!hw_cread(CAP_ENDPTPRIME, BIT(n)))
		goto out;

	if (list_empty(&mep->qh.queue))
		goto out;

	req = list_entry(mep->qh.queue.next, struct ci13xxx_req, queue);

	mb();
	if (!(TD_STATUS_ACTIVE & req->ptr->token))
		goto out;

	mep->prime_timer_count++;
	if (mep->prime_timer_count == MAX_PRIME_CHECK_RETRY) {
		mep->prime_timer_count = 0;
		pr_info("ep%d dir:%s QH:cap:%08x cur:%08x next:%08x tkn:%08x\n",
				mep->num, mep->dir ? "IN" : "OUT",
				mep->qh.ptr->cap, mep->qh.ptr->curr,
				mep->qh.ptr->td.next, mep->qh.ptr->td.token);
		list_for_each(ptr, &mep->qh.queue) {
			req = list_entry(ptr, struct ci13xxx_req, queue);
			pr_info("\treq:%pKa:%08xtkn:%08xpage0:%08xsts:%d\n",
					&req->dma, req->ptr->next,
					req->ptr->token, req->ptr->page[0],
					req->req.status);
		}
		mep->prime_fail_count++;
	} else {
		mod_timer(&mep->prime_timer, EP_PRIME_CHECK_DELAY);
	}

	spin_unlock_irqrestore(mep->lock, flags);
	return;

out:
	mep->prime_timer_count = 0;
	spin_unlock_irqrestore(mep->lock, flags);

}

/**
 * _hardware_queue: configures a request at hardware level
 * @gadget: gadget
 * @mEp:    endpoint
 *
 * This function returns an error code
 */
static int _hardware_enqueue(struct ci13xxx_ep *mEp, struct ci13xxx_req *mReq)
{
	unsigned int i;
	int ret = 0;
	unsigned int length = mReq->req.length;
	struct ci13xxx *udc = _udc;


	/* don't queue twice */
	if (mReq->req.status == -EALREADY)
		return -EALREADY;

	mReq->req.status = -EALREADY;
	if (length && mReq->req.dma == DMA_ERROR_CODE) {
		mReq->req.dma = \
			dma_map_single(mEp->device, mReq->req.buf,
				       length, mEp->dir ? DMA_TO_DEVICE :
				       DMA_FROM_DEVICE);
		if (mReq->req.dma == 0)
			return -ENOMEM;

		mReq->map = 1;
	}

	if (mReq->req.zero && length && (length % mEp->ep.maxpacket == 0)) {
		mReq->zptr = dma_pool_alloc(mEp->td_pool, GFP_ATOMIC,
					   &mReq->zdma);
		if (mReq->zptr == NULL) {
			if (mReq->map) {
				dma_unmap_single(mEp->device, mReq->req.dma,
					length, mEp->dir ? DMA_TO_DEVICE :
					DMA_FROM_DEVICE);
				mReq->req.dma = DMA_ERROR_CODE;
				mReq->map     = 0;
			}
			return -ENOMEM;
		}
		memset(mReq->zptr, 0, sizeof(*mReq->zptr));
		mReq->zptr->next    = TD_TERMINATE;
		mReq->zptr->token   = TD_STATUS_ACTIVE;
		if (!mReq->req.no_interrupt)
			mReq->zptr->token   |= TD_IOC;
	}

	/*
	 * TD configuration
	 * TODO - handle requests which spawns into several TDs
	 */
	memset(mReq->ptr, 0, sizeof(*mReq->ptr));
	mReq->ptr->token    = length << ffs_nr(TD_TOTAL_BYTES);
	mReq->ptr->token   &= TD_TOTAL_BYTES;
	mReq->ptr->token   |= TD_STATUS_ACTIVE;
	if (mReq->zptr) {
		mReq->ptr->next    = mReq->zdma;
	} else {
		mReq->ptr->next    = TD_TERMINATE;
		if (!mReq->req.no_interrupt)
			mReq->ptr->token  |= TD_IOC;
	}

	/* MSM Specific: updating the request as required for
	 * SPS mode. Enable MSM DMA engine according
	 * to the UDC private data in the request.
	 */
	if (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID) {
		if (mReq->req.udc_priv & MSM_SPS_MODE) {
			mReq->ptr->token = TD_STATUS_ACTIVE;
			if (mReq->req.udc_priv & MSM_IS_FINITE_TRANSFER)
				mReq->ptr->next = TD_TERMINATE;
			else
				mReq->ptr->next = MSM_ETD_TYPE | mReq->dma;
			if (!mReq->req.no_interrupt)
				mReq->ptr->token |= MSM_ETD_IOC;
		}
		mReq->req.dma = 0;
	}

	mReq->ptr->page[0]  = mReq->req.dma;
	for (i = 1; i < 5; i++)
		mReq->ptr->page[i] = (mReq->req.dma + i * CI13XXX_PAGE_SIZE) &
							~TD_RESERVED_MASK;
	wmb();

	/* Remote Wakeup */
	if (udc->suspended) {
		if (!udc->gadget.remote_wakeup) {
			mReq->req.status = -EAGAIN;

			dev_info(mEp->device, "%s: queue failed (suspend).",
					__func__);
			dev_info(mEp->device, "%s: Remote wakeup is not supported. ept #%d, but doing it anyway\n",
					__func__, mEp->num);

			return -EAGAIN;
		}

		usb_phy_set_suspend(udc->transceiver, 0);
		schedule_delayed_work(&udc->rw_work, REMOTE_WAKEUP_DELAY);
	}

	if (!list_empty(&mEp->qh.queue)) {
		struct ci13xxx_req *mReqPrev;
		int n = hw_ep_bit(mEp->num, mEp->dir);
		int tmp_stat;
		ktime_t start, diff;

		mReqPrev = list_entry(mEp->qh.queue.prev,
				struct ci13xxx_req, queue);
		if (mReqPrev->zptr)
			mReqPrev->zptr->next = mReq->dma & TD_ADDR_MASK;
		else
			mReqPrev->ptr->next = mReq->dma & TD_ADDR_MASK;
		wmb();
		if (hw_cread(CAP_ENDPTPRIME, BIT(n)))
			goto done;
		start = ktime_get();
		do {
			hw_cwrite(CAP_USBCMD, USBCMD_ATDTW, USBCMD_ATDTW);
			tmp_stat = hw_cread(CAP_ENDPTSTAT, BIT(n));
			diff = ktime_sub(ktime_get(), start);
			/* poll for max. 100ms */
			if (ktime_to_ms(diff) > USB_MAX_TIMEOUT) {
				if (hw_cread(CAP_USBCMD, USBCMD_ATDTW))
					break;
				printk_ratelimited(KERN_ERR
				"%s:queue failed ep#%d %s\n",
				 __func__, mEp->num, mEp->dir ? "IN" : "OUT");
				return -EAGAIN;
			}
		} while (!hw_cread(CAP_USBCMD, USBCMD_ATDTW));
		hw_cwrite(CAP_USBCMD, USBCMD_ATDTW, 0);
		if (tmp_stat)
			goto done;
	}

	/* Hardware may leave few TDs unprocessed, check and reprime with 1st */
	if (!list_empty(&mEp->qh.queue)) {
		struct ci13xxx_req *mReq_active, *mReq_next;
		u32 i = 0;

		/* Nothing to be done if hardware already finished this TD */
		if ((TD_STATUS_ACTIVE & mReq->ptr->token) == 0)
			goto done;

		/* Iterate forward to find first TD with ACTIVE bit set */
		mReq_active = mReq;
		list_for_each_entry(mReq_next, &mEp->qh.queue, queue) {
			i++;
			mEp->dTD_active_re_q_count++;
			if (TD_STATUS_ACTIVE & mReq_next->ptr->token) {
				mReq_active = mReq_next;
				pr_debug("!!ReQ(%u-%u-%x)-%u!!\n", mEp->num,
					 mEp->dir, mReq_next->ptr->token, i);
				break;
			}
		}

		/*  QH configuration */
		mEp->qh.ptr->td.next = mReq_active->dma;
		mEp->qh.ptr->td.token &= ~TD_STATUS;
		goto prime;
	}

	/*  QH configuration */
	mEp->qh.ptr->td.next   = mReq->dma;    /* TERMINATE = 0 */

	if (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID) {
		if (mReq->req.udc_priv & MSM_SPS_MODE) {
			mEp->qh.ptr->td.next   |= MSM_ETD_TYPE;
			i = hw_cread(CAP_ENDPTPIPEID +
						 mEp->num * sizeof(u32), ~0);
			/* Read current value of this EPs pipe id */
			i = (mEp->dir == TX) ?
				((i >> MSM_TX_PIPE_ID_OFS) & MSM_PIPE_ID_MASK) :
					(i & MSM_PIPE_ID_MASK);
			/* If requested pipe id is different from current,
			   then write it */
			if (i != (mReq->req.udc_priv & MSM_PIPE_ID_MASK)) {
				if (mEp->dir == TX)
					hw_cwrite(
						CAP_ENDPTPIPEID +
							mEp->num * sizeof(u32),
						MSM_PIPE_ID_MASK <<
							MSM_TX_PIPE_ID_OFS,
						(mReq->req.udc_priv &
						 MSM_PIPE_ID_MASK)
							<< MSM_TX_PIPE_ID_OFS);
				else
					hw_cwrite(
						CAP_ENDPTPIPEID +
							mEp->num * sizeof(u32),
						MSM_PIPE_ID_MASK,
						mReq->req.udc_priv &
							MSM_PIPE_ID_MASK);
			}
		}
	}

	mEp->qh.ptr->td.token &= ~TD_STATUS;   /* clear status */
	mEp->qh.ptr->cap |=  QH_ZLT;

prime:
	wmb();   /* synchronize before ep prime */

	ret = hw_ep_prime(mEp->num, mEp->dir,
			   mEp->type == USB_ENDPOINT_XFER_CONTROL);
	if (!ret)
		mod_timer(&mEp->prime_timer, EP_PRIME_CHECK_DELAY);

done:
	return ret;
}

/**
 * _hardware_dequeue: handles a request at hardware level
 * @gadget: gadget
 * @mEp:    endpoint
 *
 * This function returns an error code
 */
static int _hardware_dequeue(struct ci13xxx_ep *mEp, struct ci13xxx_req *mReq)
{

	if (mReq->req.status != -EALREADY)
		return -EINVAL;

	/* clean speculative fetches on req->ptr->token */
	mb();

	if ((TD_STATUS_ACTIVE & mReq->ptr->token) != 0)
		return -EBUSY;

	if (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID)
		if ((mReq->req.udc_priv & MSM_SPS_MODE) &&
			(mReq->req.udc_priv & MSM_IS_FINITE_TRANSFER))
			return -EBUSY;
	if (mReq->zptr) {
		if ((TD_STATUS_ACTIVE & mReq->zptr->token) != 0)
			return -EBUSY;

		/* The controller may access this dTD one more time.
		 * Defer freeing this to next zero length dTD completion.
		 * It is safe to assume that controller will no longer
		 * access the previous dTD after next dTD completion.
		 */
		if (mEp->last_zptr)
			dma_pool_free(mEp->td_pool, mEp->last_zptr,
					mEp->last_zdma);
		mEp->last_zptr = mReq->zptr;
		mEp->last_zdma = mReq->zdma;

		mReq->zptr = NULL;
	}

	mReq->req.status = 0;

	if (mReq->map) {
		dma_unmap_single(mEp->device, mReq->req.dma, mReq->req.length,
				 mEp->dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		mReq->req.dma = DMA_ERROR_CODE;
		mReq->map     = 0;
	}

	mReq->req.status = mReq->ptr->token & TD_STATUS;
	if ((TD_STATUS_HALTED & mReq->req.status) != 0)
		mReq->req.status = -1;
	else if ((TD_STATUS_DT_ERR & mReq->req.status) != 0)
		mReq->req.status = -1;
	else if ((TD_STATUS_TR_ERR & mReq->req.status) != 0)
		mReq->req.status = -1;

	mReq->req.actual   = mReq->ptr->token & TD_TOTAL_BYTES;
	mReq->req.actual >>= ffs_nr(TD_TOTAL_BYTES);
	mReq->req.actual   = mReq->req.length - mReq->req.actual;
	mReq->req.actual   = mReq->req.status ? 0 : mReq->req.actual;

	return mReq->req.actual;
}

/**
 * purge_rw_queue: Purge requests pending at the remote-wakeup
 * queue and send them to the HW.
 *
 * Go over all of the endpoints and push any pending requests to
 * the HW queue.
 */
static void purge_rw_queue(struct ci13xxx *udc)
{
	int i;
	struct ci13xxx_ep  *mEp  = NULL;
	struct ci13xxx_req *mReq = NULL;

	/*
	 * Go over all of the endpoints and push any pending requests to
	 * the HW queue.
	 */
	for (i = 0; i < hw_ep_max; i++) {
		mEp = &udc->ci13xxx_ep[i];

		while (!list_empty(&udc->ci13xxx_ep[i].rw_queue)) {
			int retval;

			/* pop oldest request */
			mReq = list_entry(udc->ci13xxx_ep[i].rw_queue.next,
					  struct ci13xxx_req, queue);

			list_del_init(&mReq->queue);

			retval = _hardware_enqueue(mEp, mReq);

			if (retval != 0) {
				mReq->req.status = retval;
				if (mReq->req.complete != NULL) {
					if (mEp->type ==
					    USB_ENDPOINT_XFER_CONTROL)
						mReq->req.complete(
							&(_udc->ep0in.ep),
							&mReq->req);
					else
						mReq->req.complete(
							&mEp->ep,
							&mReq->req);
				}
				retval = 0;
			}

			if (!retval)
				list_add_tail(&mReq->queue, &mEp->qh.queue);
			else if (mEp->multi_req)
				mEp->multi_req = false;

		}
	}

	udc->rw_pending = false;
}

/**
 * restore_original_req: Restore original req's attributes
 * @mReq: Request
 *
 * This function restores original req's attributes.  Call
 * this function before completing the large req (>16K).
 */
static void restore_original_req(struct ci13xxx_req *mReq)
{
	mReq->req.buf = mReq->multi.buf;
	mReq->req.length = mReq->multi.len;
	if (!mReq->req.status)
		mReq->req.actual = mReq->multi.actual;

	mReq->multi.len = 0;
	mReq->multi.actual = 0;
	mReq->multi.buf = NULL;
}

/**
 * release_ep_request: Free and endpoint request and release
 * resources
 * @mReq: request
 * @mEp: endpoint
 *
 */
static void release_ep_request(struct ci13xxx_ep  *mEp,
			       struct ci13xxx_req *mReq)
{
	struct ci13xxx_ep *mEpTemp = mEp;

	unsigned int val;

	/* MSM Specific: Clear end point specific register */
	if (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID) {
		if (mReq->req.udc_priv & MSM_SPS_MODE) {
			val = hw_cread(CAP_ENDPTPIPEID +
				mEp->num * sizeof(u32),
				~0);

			if (val != MSM_EP_PIPE_ID_RESET_VAL)
				hw_cwrite(
					CAP_ENDPTPIPEID +
					 mEp->num * sizeof(u32),
					~0, MSM_EP_PIPE_ID_RESET_VAL);
		}
	}
	mReq->req.status = -ESHUTDOWN;

	if (mReq->map) {
		dma_unmap_single(mEp->device, mReq->req.dma,
			mReq->req.length,
			mEp->dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		mReq->req.dma = DMA_ERROR_CODE;
		mReq->map     = 0;
	}

	if (mReq->zptr) {
		dma_pool_free(mEp->td_pool, mReq->zptr, mReq->zdma);
		mReq->zptr = NULL;
		mReq->zdma = 0;
	}

	if (mEp->multi_req) {
		restore_original_req(mReq);
		mEp->multi_req = false;
	}

	if (mReq->req.complete != NULL) {
		spin_unlock(mEp->lock);
		if ((mEp->type == USB_ENDPOINT_XFER_CONTROL) &&
			mReq->req.length)
			mEpTemp = &_udc->ep0in;
		mReq->req.complete(&mEpTemp->ep, &mReq->req);
		if (mEp->type == USB_ENDPOINT_XFER_CONTROL)
			mReq->req.complete = NULL;
		spin_lock(mEp->lock);
	}
}

/**
 * _ep_nuke: dequeues all endpoint requests
 * @mEp: endpoint
 *
 * This function returns an error code
 * Caller must hold lock
 */
static int _ep_nuke(struct ci13xxx_ep *mEp)
__releases(mEp->lock)
__acquires(mEp->lock)
{

	if (mEp == NULL)
		return -EINVAL;

	del_timer(&mEp->prime_timer);
	mEp->prime_timer_count = 0;

	hw_ep_flush(mEp->num, mEp->dir);

	while (!list_empty(&mEp->qh.queue)) {
		/* pop oldest request */
		struct ci13xxx_req *mReq =
			list_entry(mEp->qh.queue.next,
				   struct ci13xxx_req, queue);
		list_del_init(&mReq->queue);

		release_ep_request(mEp, mReq);
	}

	/* Clear the requests pending at the remote-wakeup queue */
	while (!list_empty(&mEp->rw_queue)) {

		/* pop oldest request */
		struct ci13xxx_req *mReq =
			list_entry(mEp->rw_queue.next,
				   struct ci13xxx_req, queue);

		list_del_init(&mReq->queue);

		release_ep_request(mEp, mReq);
	}

	if (mEp->last_zptr) {
		dma_pool_free(mEp->td_pool, mEp->last_zptr, mEp->last_zdma);
		mEp->last_zptr = NULL;
		mEp->last_zdma = 0;
	}

	return 0;
}

/**
 * _gadget_stop_activity: stops all USB activity, flushes & disables all endpts
 * @gadget: gadget
 *
 * This function returns an error code
 */
static int _gadget_stop_activity(struct usb_gadget *gadget)
{
	struct ci13xxx    *udc = container_of(gadget, struct ci13xxx, gadget);
	unsigned long flags;


	if (gadget == NULL)
		return -EINVAL;

	spin_lock_irqsave(udc->lock, flags);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.remote_wakeup = 0;
	udc->suspended = 0;
	udc->configured = 0;
	spin_unlock_irqrestore(udc->lock, flags);

	udc->driver->disconnect(gadget);

	spin_lock_irqsave(udc->lock, flags);
	_ep_nuke(&udc->ep0out);
	_ep_nuke(&udc->ep0in);
	spin_unlock_irqrestore(udc->lock, flags);

	return 0;
}

/******************************************************************************
 * ISR block
 *****************************************************************************/
/**
 * isr_reset_handler: USB reset interrupt handler
 * @udc: UDC device
 *
 * This function resets USB engine after a bus reset occurred
 */
static void isr_reset_handler(struct ci13xxx *udc)
__releases(udc->lock)
__acquires(udc->lock)
{
	int retval;

	if (udc == NULL) {
		err("EINVAL");
		return;
	}

//	spin_unlock(udc->lock);

	if (udc->suspended) {
		printk("%s: Bus reset while suspended\n", __func__);
		if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
			CI13XXX_CONTROLLER_RESUME_EVENT);
		if (udc->transceiver)
			usb_phy_set_suspend(udc->transceiver, 0);
		udc->driver->resume(&udc->gadget);
		udc->suspended = 0;
	}
	printk("%s: Bus reset in progress\n", __func__);

	/*stop charging upon reset */
	if (udc->transceiver)
		usb_phy_set_power(udc->transceiver, 100);

	retval = _gadget_stop_activity(&udc->gadget);
	if (retval)
		goto done;

	if (udc->rw_pending)
		purge_rw_queue(udc);

	_udc->skip_flush = false;
	retval = hw_usb_reset();
	if (retval)
		goto done;

//	spin_lock(udc->lock);

 done:
	if (retval)
		err("error: %i", retval);
}

/**
 * isr_resume_handler: USB PCI interrupt handler
 * @udc: UDC device
 *
 */
static void isr_resume_handler(struct ci13xxx *udc)
{
	printk("%s: Resume begin... ", __func__);
	udc->gadget.speed = hw_port_is_high_speed() ?
		USB_SPEED_HIGH : USB_SPEED_FULL;
	if (udc->suspended) {
		printk(" ...Was suspended\n");
//		spin_unlock(udc->lock);
		if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
			  CI13XXX_CONTROLLER_RESUME_EVENT);
		if (udc->transceiver)
			usb_phy_set_suspend(udc->transceiver, 0);
		udc->suspended = 0;
		udc->driver->resume(&udc->gadget);
//		spin_lock(udc->lock);

		if (udc->rw_pending)
			purge_rw_queue(udc);

	} else {
		printk(" ...Was already resumed!\n");
	}
	
	printk("%s: Resume finished \n", __func__);
}

/**
 * isr_resume_handler: USB SLI interrupt handler
 * @udc: UDC device
 *
 */
static void isr_suspend_handler(struct ci13xxx *udc)
{
	printk("%s: Suspend begin ", __func__);
	if(udc->vbus_active) {
		if(udc->gadget.speed == USB_SPEED_UNKNOWN) {
			udc->gadget.speed = hw_port_is_high_speed() ?
				USB_SPEED_HIGH : USB_SPEED_FULL;
		}
	}
	if (udc->suspended == 0) {
		printk(" ...Suspending now!\n");

//		spin_unlock(udc->lock);
		udc->driver->suspend(&udc->gadget);
		if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
			CI13XXX_CONTROLLER_SUSPEND_EVENT);
		if (udc->transceiver)
			usb_phy_set_suspend(udc->transceiver, 1);
//		spin_lock(udc->lock);
		udc->suspended = 1;
	} else if(udc->suspended == 1) {
		printk("%s: Already suspended!\n", __func__);
	}
	printk("%s: Suspend finished \n", __func__);
}

/**
 * isr_get_status_complete: get_status request complete function
 * @ep:  endpoint
 * @req: request handled
 *
 * Caller must release lock
 */
static void isr_get_status_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (ep == NULL || req == NULL) {
		err("EINVAL");
		return;
	}

	if (req->status)
		err("GET_STATUS failed");
}

/**
 * isr_get_status_response: get_status request response
 * @udc: udc struct
 * @setup: setup request packet
 *
 * This function returns an error code
 */
static int isr_get_status_response(struct ci13xxx *udc,
				   struct usb_ctrlrequest *setup)
__releases(mEp->lock)
__acquires(mEp->lock)
{
	struct ci13xxx_ep *mEp = &udc->ep0in;
	struct usb_request *req = udc->status;
	int dir, num, retval;

	if (mEp == NULL || setup == NULL)
		return -EINVAL;

	req->complete = isr_get_status_complete;
	req->length   = 2;
	req->buf      = udc->status_buf;

	if ((setup->bRequestType & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		/* Assume that device is bus powered for now. */
		*((u16 *)req->buf) = _udc->gadget.remote_wakeup << 1;
		retval = 0;
	} else if ((setup->bRequestType & USB_RECIP_MASK) \
		   == USB_RECIP_ENDPOINT) {
		dir = (le16_to_cpu(setup->wIndex) & USB_ENDPOINT_DIR_MASK) ?
			TX : RX;
		num =  le16_to_cpu(setup->wIndex) & USB_ENDPOINT_NUMBER_MASK;
		*((u16 *)req->buf) = hw_ep_get_halt(num, dir);
	}
	/* else do nothing; reserved for future use */

	spin_unlock(mEp->lock);
	retval = usb_ep_queue(&mEp->ep, req, GFP_ATOMIC);
	spin_lock(mEp->lock);
	return retval;
}

/**
 * isr_setup_status_complete: setup_status request complete function
 * @ep:  endpoint
 * @req: request handled
 *
 * Caller must release lock. Put the port in test mode if test mode
 * feature is selected.
 */
static void
isr_setup_status_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct ci13xxx *udc = req->context;
	unsigned long flags;

	spin_lock_irqsave(udc->lock, flags);
	if (udc->test_mode)
		hw_port_test_set(udc->test_mode);
	spin_unlock_irqrestore(udc->lock, flags);
}

/**
 * isr_setup_status_phase: queues the status phase of a setup transation
 * @udc: udc struct
 *
 * This function returns an error code
 */
static int isr_setup_status_phase(struct ci13xxx *udc)
__releases(mEp->lock)
__acquires(mEp->lock)
{
	int retval;
	struct ci13xxx_ep *mEp;

	mEp = (udc->ep0_dir == TX) ? &udc->ep0out : &udc->ep0in;
	udc->status->context = udc;
	udc->status->complete = isr_setup_status_complete;
	udc->status->length = 0;

	spin_unlock(mEp->lock);
	retval = usb_ep_queue(&mEp->ep, udc->status, GFP_ATOMIC);
	spin_lock(mEp->lock);

	return retval;
}

/**
 * isr_tr_complete_low: transaction complete low level handler
 * @mEp: endpoint
 *
 * This function returns an error code
 * Caller must hold lock
 */
static int isr_tr_complete_low(struct ci13xxx_ep *mEp)
__releases(mEp->lock)
__acquires(mEp->lock)
{
	struct ci13xxx_req *mReq, *mReqTemp;
	struct ci13xxx_ep *mEpTemp = mEp;
	int retval = 0;
	int req_dequeue = 1;
	struct ci13xxx *udc = _udc;

	if (list_empty(&mEp->qh.queue))
		return 0;

	del_timer(&mEp->prime_timer);
	mEp->prime_timer_count = 0;
	list_for_each_entry_safe(mReq, mReqTemp, &mEp->qh.queue,
			queue) {
dequeue:
		retval = _hardware_dequeue(mEp, mReq);
		if (retval < 0) {
			/*
			 * FIXME: don't know exact delay
			 * required for HW to update dTD status
			 * bits. This is a temporary workaround till
			 * HW designers come back on this.
			 */
			if (retval == -EBUSY && req_dequeue &&
				(mEp->dir == 0 || mEp->num == 0)) {
				req_dequeue = 0;
				udc->dTD_update_fail_count++;
				mEp->dTD_update_fail_count++;
				udelay(10);
				goto dequeue;
			}
			break;
		}
		req_dequeue = 0;

		if (mEp->multi_req) { /* Large request in progress */
			unsigned int remain_len;

			mReq->multi.actual += mReq->req.actual;
			remain_len = mReq->multi.len - mReq->multi.actual;
			if (mReq->req.status || !remain_len ||
				(mReq->req.actual != mReq->req.length)) {
				restore_original_req(mReq);
				mEp->multi_req = false;
			} else {
				mReq->req.buf = mReq->multi.buf +
						mReq->multi.actual;
				mReq->req.length = min_t(unsigned int, remain_len,
						(4 * CI13XXX_PAGE_SIZE));

				mReq->req.status = -EINPROGRESS;
				mReq->req.actual = 0;
				list_del_init(&mReq->queue);
				retval = _hardware_enqueue(mEp, mReq);
				if (retval) {
					err("Large req failed in middle");
					mReq->req.status = retval;
					restore_original_req(mReq);
					mEp->multi_req = false;
					goto done;
				} else {
					list_add_tail(&mReq->queue,
						&mEp->qh.queue);
					return 0;
				}
			}
		}
		list_del_init(&mReq->queue);
done:

		if (mReq->req.complete != NULL) {
			spin_unlock(mEp->lock);
			if ((mEp->type == USB_ENDPOINT_XFER_CONTROL) &&
					mReq->req.length)
				mEpTemp = &_udc->ep0in;
			mReq->req.complete(&mEpTemp->ep, &mReq->req);
			spin_lock(mEp->lock);
		}
	}

	if (retval == -EBUSY)
		retval = 0;

	return retval;
}

/**
 * isr_tr_complete_handler: transaction complete interrupt handler
 * @udc: UDC descriptor
 *
 * This function handles traffic events
 */
static void isr_tr_complete_handler(struct ci13xxx *udc)
__releases(udc->lock)
__acquires(udc->lock)
{
	unsigned int i;
	u8 tmode = 0;

	if (udc == NULL) {
		err("EINVAL");
		return;
	}

	for (i = 0; i < hw_ep_max; i++) {
		struct ci13xxx_ep *mEp  = &udc->ci13xxx_ep[i];
		int type, num, dir, err = -EINVAL;
		struct usb_ctrlrequest req;

		if (mEp->desc == NULL)
			continue;   /* not configured */

		if (hw_test_and_clear_complete(i)) {
			err = isr_tr_complete_low(mEp);
			if (mEp->type == USB_ENDPOINT_XFER_CONTROL) {
				if (err > 0)   /* needs status phase */
					err = isr_setup_status_phase(udc);
				if (err < 0) {
					spin_unlock(udc->lock);
					if (usb_ep_set_halt(&mEp->ep))
						err("error: ep_set_halt");
					spin_lock(udc->lock);
				}
			}
		}

		if (mEp->type != USB_ENDPOINT_XFER_CONTROL ||
		    !hw_test_and_clear_setup_status(i))
			continue;

		if (i != 0) {
			warn("ctrl traffic received at endpoint");
			continue;
		}

		/*
		 * Flush data and handshake transactions of previous
		 * setup packet.
		 */
		_ep_nuke(&udc->ep0out);
		_ep_nuke(&udc->ep0in);

		/* read_setup_packet */
		do {
			hw_test_and_set_setup_guard();
			memcpy(&req, &mEp->qh.ptr->setup, sizeof(req));
			/* Ensure buffer is read before acknowledging to h/w */
			mb();
		} while (!hw_test_and_clear_setup_guard());

		type = req.bRequestType;

		udc->ep0_dir = (type & USB_DIR_IN) ? TX : RX;

		switch (req.bRequest) {
		case USB_REQ_CLEAR_FEATURE:
			if (type == (USB_DIR_OUT|USB_RECIP_ENDPOINT) &&
					le16_to_cpu(req.wValue) ==
					USB_ENDPOINT_HALT) {
				if (req.wLength != 0)
					break;
				num  = le16_to_cpu(req.wIndex);
				dir = num & USB_ENDPOINT_DIR_MASK;
				num &= USB_ENDPOINT_NUMBER_MASK;
				if (dir) /* TX */
					num += hw_ep_max/2;
				if (!udc->ci13xxx_ep[num].wedge) {
					spin_unlock(udc->lock);
					err = usb_ep_clear_halt(
						&udc->ci13xxx_ep[num].ep);
					spin_lock(udc->lock);
					if (err)
						break;
				}
				err = isr_setup_status_phase(udc);
			} else if (type == (USB_DIR_OUT|USB_RECIP_DEVICE) &&
					le16_to_cpu(req.wValue) ==
					USB_DEVICE_REMOTE_WAKEUP) {
				if (req.wLength != 0)
					break;
				udc->gadget.remote_wakeup = 0;
				err = isr_setup_status_phase(udc);
			} else {
				goto delegate;
			}
			break;
		case USB_REQ_GET_STATUS:
			if (type != (USB_DIR_IN|USB_RECIP_DEVICE)   &&
			    type != (USB_DIR_IN|USB_RECIP_ENDPOINT) &&
			    type != (USB_DIR_IN|USB_RECIP_INTERFACE))
				goto delegate;
			if (le16_to_cpu(req.wLength) != 2 ||
			    le16_to_cpu(req.wValue)  != 0)
				break;
			err = isr_get_status_response(udc, &req);
			break;
		case USB_REQ_SET_ADDRESS:
			if (type != (USB_DIR_OUT|USB_RECIP_DEVICE))
				goto delegate;
			if (le16_to_cpu(req.wLength) != 0 ||
			    le16_to_cpu(req.wIndex)  != 0)
				break;
			err = hw_usb_set_address((u8)le16_to_cpu(req.wValue));
			if (err)
				break;
			err = isr_setup_status_phase(udc);
			break;
		case USB_REQ_SET_CONFIGURATION:
			if (type == (USB_DIR_OUT|USB_TYPE_STANDARD))
				udc->configured = !!req.wValue;
			goto delegate;
		case USB_REQ_SET_FEATURE:
			if (type == (USB_DIR_OUT|USB_RECIP_ENDPOINT) &&
					le16_to_cpu(req.wValue) ==
					USB_ENDPOINT_HALT) {
				if (req.wLength != 0)
					break;
				num  = le16_to_cpu(req.wIndex);
				dir = num & USB_ENDPOINT_DIR_MASK;
				num &= USB_ENDPOINT_NUMBER_MASK;
				if (dir) /* TX */
					num += hw_ep_max/2;

				spin_unlock(udc->lock);
				err = usb_ep_set_halt(&udc->ci13xxx_ep[num].ep);
				spin_lock(udc->lock);
				if (!err)
					isr_setup_status_phase(udc);
			} else if (type == (USB_DIR_OUT|USB_RECIP_DEVICE)) {
				if (req.wLength != 0)
					break;
				switch (le16_to_cpu(req.wValue)) {
				case USB_DEVICE_REMOTE_WAKEUP:
					udc->gadget.remote_wakeup = 1;
					err = isr_setup_status_phase(udc);
					break;
				case USB_DEVICE_TEST_MODE:
					tmode = le16_to_cpu(req.wIndex) >> 8;
					switch (tmode) {
					case TEST_J:
					case TEST_K:
					case TEST_SE0_NAK:
					case TEST_PACKET:
					case TEST_FORCE_EN:
						udc->test_mode = tmode;
						err = isr_setup_status_phase(
								udc);
						break;
					default:
						break;
					}
				default:
					goto delegate;
				}
			} else {
				goto delegate;
			}
			break;
		default:
delegate:
			if (req.wLength == 0)   /* no data phase */
				udc->ep0_dir = TX;

			spin_unlock(udc->lock);
			err = udc->driver->setup(&udc->gadget, &req);
			spin_lock(udc->lock);
			break;
		}

		if (err < 0) {
			spin_unlock(udc->lock);
			if (usb_ep_set_halt(&mEp->ep))
				err("error: ep_set_halt");
			spin_lock(udc->lock);
		}
	}
}

/******************************************************************************
 * ENDPT block
 *****************************************************************************/
/**
 * ep_enable: configure endpoint, making it usable
 *
 * Check usb_ep_enable() at "usb_gadget.h" for details
 */
static int ep_enable(struct usb_ep *ep,
		     const struct usb_endpoint_descriptor *desc)
{
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	int retval = 0;
	unsigned long flags;
	unsigned int mult = 0;

	if (ep == NULL || desc == NULL)
		return -EINVAL;

	spin_lock_irqsave(mEp->lock, flags);

	/* only internal SW should enable ctrl endpts */

	mEp->desc = desc;

	if (!list_empty(&mEp->qh.queue))
		warn("enabling a non-empty endpoint!");

	mEp->dir  = usb_endpoint_dir_in(desc) ? TX : RX;
	mEp->num  = usb_endpoint_num(desc);
	mEp->type = usb_endpoint_type(desc);

	mEp->ep.maxpacket = usb_endpoint_maxp(desc);

	mEp->qh.ptr->cap = 0;

	if (mEp->type == USB_ENDPOINT_XFER_CONTROL) {
		mEp->qh.ptr->cap |=  QH_IOS;
	} else if (mEp->type == USB_ENDPOINT_XFER_ISOC) {
		mEp->qh.ptr->cap &= ~QH_MULT;
		mult = ((mEp->ep.maxpacket >> QH_MULT_SHIFT) + 1) & 0x03;
		mEp->qh.ptr->cap |= (mult << ffs_nr(QH_MULT));
	} else {
		mEp->qh.ptr->cap |= QH_ZLT;
	}

	mEp->qh.ptr->cap |=
		(mEp->ep.maxpacket << ffs_nr(QH_MAX_PKT)) & QH_MAX_PKT;
	mEp->qh.ptr->td.next |= TD_TERMINATE;   /* needed? */

	/* complete all the updates to ept->head before enabling endpoint*/
	mb();

	/*
	 * Enable endpoints in the HW other than ep0 as ep0
	 * is always enabled
	 */
	if (mEp->num)
		retval |= hw_ep_enable(mEp->num, mEp->dir, mEp->type);

	spin_unlock_irqrestore(mEp->lock, flags);
	return retval;
}

/**
 * ep_disable: endpoint is no longer usable
 *
 * Check usb_ep_disable() at "usb_gadget.h" for details
 */
static int ep_disable(struct usb_ep *ep)
{
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	int direction, retval = 0;
	unsigned long flags;

	if (ep == NULL)
		return -EINVAL;
	else if (mEp->desc == NULL)
		return -EBUSY;

	spin_lock_irqsave(mEp->lock, flags);

	/* only internal SW should disable ctrl endpts */

	direction = mEp->dir;
	do {
		retval |= _ep_nuke(mEp);
		retval |= hw_ep_disable(mEp->num, mEp->dir);

		if (mEp->type == USB_ENDPOINT_XFER_CONTROL)
			mEp->dir = (mEp->dir == TX) ? RX : TX;

	} while (mEp->dir != direction);

	mEp->desc = NULL;
	mEp->ep.desc = NULL;
	mEp->ep.maxpacket = USHRT_MAX;

	spin_unlock_irqrestore(mEp->lock, flags);
	return retval;
}

/**
 * ep_alloc_request: allocate a request object to use with this endpoint
 *
 * Check usb_ep_alloc_request() at "usb_gadget.h" for details
 */
static struct usb_request *ep_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
	struct ci13xxx_ep  *mEp  = container_of(ep, struct ci13xxx_ep, ep);
	struct ci13xxx_req *mReq = NULL;

	if (ep == NULL) {
		err("EINVAL");
		return NULL;
	}

	mReq = kzalloc(sizeof(struct ci13xxx_req), gfp_flags);
	if (mReq != NULL) {
		INIT_LIST_HEAD(&mReq->queue);
		mReq->req.dma = DMA_ERROR_CODE;

		mReq->ptr = dma_pool_alloc(mEp->td_pool, gfp_flags,
					   &mReq->dma);
		if (mReq->ptr == NULL) {
			kfree(mReq);
			mReq = NULL;
		}
	}

	return (mReq == NULL) ? NULL : &mReq->req;
}

/**
 * ep_free_request: frees a request object
 *
 * Check usb_ep_free_request() at "usb_gadget.h" for details
 */
static void ep_free_request(struct usb_ep *ep, struct usb_request *req)
{
	struct ci13xxx_ep  *mEp  = container_of(ep,  struct ci13xxx_ep, ep);
	struct ci13xxx_req *mReq = container_of(req, struct ci13xxx_req, req);
	unsigned long flags;

	if (ep == NULL || req == NULL) {
		err("EINVAL");
		return;
	} else if (!list_empty(&mReq->queue)) {
		err("EBUSY");
		return;
	}

	spin_lock_irqsave(mEp->lock, flags);

	if (mReq->ptr)
		dma_pool_free(mEp->td_pool, mReq->ptr, mReq->dma);
	kfree(mReq);

	spin_unlock_irqrestore(mEp->lock, flags);
}

/**
 * ep_queue: queues (submits) an I/O request to an endpoint
 *
 * Check usb_ep_queue()* at usb_gadget.h" for details
 */
static int ep_queue(struct usb_ep *ep, struct usb_request *req,
		    gfp_t __maybe_unused gfp_flags)
{
	struct ci13xxx_ep  *mEp  = container_of(ep,  struct ci13xxx_ep, ep);
	struct ci13xxx_req *mReq = container_of(req, struct ci13xxx_req, req);
	int retval = 0;
	unsigned long flags;
	struct ci13xxx *udc = _udc;

	if (ep == NULL)
		return -EINVAL;

	spin_lock_irqsave(mEp->lock, flags);
	if (req == NULL || mEp->desc == NULL) {
		retval = -EINVAL;
		goto done;
	}

	if (!udc->softconnect) {
		retval = -ENODEV;
		goto done;
	}

	if (!udc->configured && mEp->type !=
		USB_ENDPOINT_XFER_CONTROL) {
		retval = -ESHUTDOWN;
		goto done;
	}

	if (mEp->type == USB_ENDPOINT_XFER_CONTROL) {
		if (req->length)
			mEp = (_udc->ep0_dir == RX) ?
				&_udc->ep0out : &_udc->ep0in;
		if (!list_empty(&mEp->qh.queue)) {
			_ep_nuke(mEp);
			retval = -EOVERFLOW;
			warn("endpoint ctrl %X nuked", _usb_addr(mEp));
		}
	}

	if (ep->endless && udc->gadget.speed == USB_SPEED_FULL) {
		err("Queueing endless req is not supported for FS");
		retval = -EINVAL;
		goto done;
	}

	/* first nuke then test link, e.g. previous status has not sent */
	if (!list_empty(&mReq->queue)) {
		retval = -EBUSY;
		err("request already in queue");
		goto done;
	}
	if (mEp->multi_req) {
		retval = -EAGAIN;
		err("Large request is in progress. come again");
		goto done;
	}

	if (req->length > (4 * CI13XXX_PAGE_SIZE)) {
		if (!list_empty(&mEp->qh.queue)) {
			retval = -EAGAIN;
			err("Queue is busy. Large req is not allowed");
			goto done;
		}
		if ((mEp->type != USB_ENDPOINT_XFER_BULK) ||
				(mEp->dir != RX)) {
			retval = -EINVAL;
			err("Larger req is supported only for Bulk OUT");
			goto done;
		}
		mEp->multi_req = true;
		mReq->multi.len = req->length;
		mReq->multi.buf = req->buf;
		req->length = (4 * CI13XXX_PAGE_SIZE);
	}

	/* push request */
	mReq->req.status = -EINPROGRESS;
	mReq->req.actual = 0;

	if (udc->rw_pending) {
		list_add_tail(&mReq->queue, &mEp->rw_queue);
		retval = 0;
		goto done;
	}

	if (udc->suspended) {
		/* Remote Wakeup */
		if (!udc->gadget.remote_wakeup) {

			dev_info(mEp->device, "%s: queue failed (suspend).",
					__func__);
			dev_info(mEp->device, "%s: Remote wakeup is not supported. ept #%d\n",
					__func__, mEp->num);
			mEp->multi_req = false;

			retval = -EAGAIN;
			goto done;
		}

		list_add_tail(&mReq->queue, &mEp->rw_queue);

		udc->rw_pending = true;
		schedule_delayed_work(&udc->rw_work,
				      REMOTE_WAKEUP_DELAY);

		retval = 0;
		goto done;
	}

	retval = _hardware_enqueue(mEp, mReq);

	if (retval == -EALREADY) {
		retval = 0;
	}
	if (!retval)
		list_add_tail(&mReq->queue, &mEp->qh.queue);
	else if (mEp->multi_req)
		mEp->multi_req = false;

 done:
	spin_unlock_irqrestore(mEp->lock, flags);
	return retval;
}

/**
 * ep_dequeue: dequeues (cancels, unlinks) an I/O request from an endpoint
 *
 * Check usb_ep_dequeue() at "usb_gadget.h" for details
 */
static int ep_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	struct ci13xxx_ep  *mEp  = container_of(ep,  struct ci13xxx_ep, ep);
	struct ci13xxx_ep *mEpTemp = mEp;
	struct ci13xxx_req *mReq = container_of(req, struct ci13xxx_req, req);
	struct ci13xxx *udc = _udc;
	unsigned long flags;

	if (udc->udc_driver->in_lpm && udc->udc_driver->in_lpm(udc)) {
		dev_err(udc->transceiver->dev,
				"%s: Unable to dequeue while in LPM\n",
				__func__);
		return -EAGAIN;
	}

	if (ep == NULL)
		return -EINVAL;

	spin_lock_irqsave(mEp->lock, flags);
	/*
	 * Only ep0 IN is exposed to composite.  When a req is dequeued
	 * on ep0, check both ep0 IN and ep0 OUT queues.
	 */
	if (req == NULL || mReq->req.status != -EALREADY ||
		mEp->desc == NULL || list_empty(&mReq->queue) ||
		(list_empty(&mEp->qh.queue) && ((mEp->type !=
			USB_ENDPOINT_XFER_CONTROL) ||
			list_empty(&_udc->ep0out.qh.queue)))) {
		spin_unlock_irqrestore(mEp->lock, flags);
		return -EINVAL;
	}

	if (mEp->type == USB_ENDPOINT_XFER_CONTROL) {
		hw_ep_flush(_udc->ep0out.num, RX);
		hw_ep_flush(_udc->ep0in.num, TX);
	} else {
		hw_ep_flush(mEp->num, mEp->dir);
	}

	/* pop request */
	list_del_init(&mReq->queue);
	if (mReq->map) {
		dma_unmap_single(mEp->device, mReq->req.dma, mReq->req.length,
				 mEp->dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		mReq->req.dma = DMA_ERROR_CODE;
		mReq->map     = 0;
	}
	req->status = -ECONNRESET;

	if (mEp->last_zptr) {
		dma_pool_free(mEp->td_pool, mEp->last_zptr, mEp->last_zdma);
		mEp->last_zptr = NULL;
		mEp->last_zdma = 0;
	}

	if (mReq->zptr) {
		dma_pool_free(mEp->td_pool, mReq->zptr, mReq->zdma);
		mReq->zptr = NULL;
		mReq->zdma = 0;
	}

	if (mEp->multi_req) {
		restore_original_req(mReq);
		mEp->multi_req = false;
	}

	if (mReq->req.complete != NULL) {
		spin_unlock(mEp->lock);
		if ((mEp->type == USB_ENDPOINT_XFER_CONTROL) &&
				mReq->req.length)
			mEpTemp = &_udc->ep0in;
		mReq->req.complete(&mEpTemp->ep, &mReq->req);
		if (mEp->type == USB_ENDPOINT_XFER_CONTROL)
			mReq->req.complete = NULL;
		spin_lock(mEp->lock);
	}

	spin_unlock_irqrestore(mEp->lock, flags);
	return 0;
}

static int is_sps_req(struct ci13xxx_req *mReq)
{
	return (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID &&
			mReq->req.udc_priv & MSM_SPS_MODE);
}

/**
 * ep_set_halt: sets the endpoint halt feature
 *
 * Check usb_ep_set_halt() at "usb_gadget.h" for details
 */
static int ep_set_halt(struct usb_ep *ep, int value)
{
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	struct ci13xxx *udc = _udc;
	int direction, retval = 0;
	unsigned long flags;

	if (ep == NULL || mEp->desc == NULL)
		return -EINVAL;

	if (udc->suspended) {
		dev_err(udc->transceiver->dev,
			"%s: Unable to halt EP while suspended\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(mEp->lock, flags);

#ifndef STALL_IN
	/* g_file_storage MS compliant but g_zero fails chapter 9 compliance */
	if (value && mEp->type == USB_ENDPOINT_XFER_BULK && mEp->dir == TX &&
		!list_empty(&mEp->qh.queue) &&
		!is_sps_req(list_entry(mEp->qh.queue.next, struct ci13xxx_req,
							   queue))){
		spin_unlock_irqrestore(mEp->lock, flags);
		return -EAGAIN;
	}
#endif

	direction = mEp->dir;
	do {
		retval |= hw_ep_set_halt(mEp->num, mEp->dir, value);

		if (!value)
			mEp->wedge = 0;

		if (mEp->type == USB_ENDPOINT_XFER_CONTROL)
			mEp->dir = (mEp->dir == TX) ? RX : TX;

	} while (mEp->dir != direction);

	spin_unlock_irqrestore(mEp->lock, flags);
	return retval;
}

/**
 * ep_set_wedge: sets the halt feature and ignores clear requests
 *
 * Check usb_ep_set_wedge() at "usb_gadget.h" for details
 */
static int ep_set_wedge(struct usb_ep *ep)
{
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	unsigned long flags;

	if (ep == NULL || mEp->desc == NULL)
		return -EINVAL;

	spin_lock_irqsave(mEp->lock, flags);

	mEp->wedge = 1;
	
	spin_unlock_irqrestore(mEp->lock, flags);

	return usb_ep_set_halt(ep);
}

/**
 * ep_fifo_flush: flushes contents of a fifo
 *
 * Check usb_ep_fifo_flush() at "usb_gadget.h" for details
 */
static void ep_fifo_flush(struct usb_ep *ep)
{
	struct ci13xxx *udc = _udc;
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	unsigned long flags;

	if (ep == NULL) {
		return;
	}

	if (udc->udc_driver->in_lpm && udc->udc_driver->in_lpm(udc)) {
		dev_err(udc->transceiver->dev,
				"%s: Unable to fifo_flush while in LPM\n",
				__func__);
		return;
	}

	spin_lock_irqsave(mEp->lock, flags);

	/*
	 * _ep_nuke() takes care of flushing the endpoint.
	 * some function drivers expect udc to retire all
	 * pending requests upon flushing an endpoint.  There
	 * is no harm in doing it.
	 */
	_ep_nuke(mEp);

	spin_unlock_irqrestore(mEp->lock, flags);
}

/**
 * Endpoint-specific part of the API to the USB controller hardware
 * Check "usb_gadget.h" for details
 */
static const struct usb_ep_ops usb_ep_ops = {
	.enable	       = ep_enable,
	.disable       = ep_disable,
	.alloc_request = ep_alloc_request,
	.free_request  = ep_free_request,
	.queue	       = ep_queue,
	.dequeue       = ep_dequeue,
	.set_halt      = ep_set_halt,
	.set_wedge     = ep_set_wedge,
	.fifo_flush    = ep_fifo_flush,
};

/******************************************************************************
 * GADGET block
 *****************************************************************************/
static int ci13xxx_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	struct ci13xxx *udc = container_of(_gadget, struct ci13xxx, gadget);
	unsigned long flags;
	int gadget_ready = 0;

	if (!(udc->udc_driver->flags & CI13XXX_PULLUP_ON_VBUS))
		return -EOPNOTSUPP;

	spin_lock_irqsave(udc->lock, flags);
	udc->vbus_active = is_active;
	if (udc->driver)
		gadget_ready = 1;
	spin_unlock_irqrestore(udc->lock, flags);

	if (!gadget_ready)
		return 0;

	if (is_active) {
		hw_device_reset(udc);
		if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
				CI13XXX_CONTROLLER_CONNECT_EVENT);
		/* Enable BAM (if needed) before starting controller */
		if (udc->softconnect) {
			msm_usb_bam_enable(CI_CTRL,
				_gadget->bam2bam_func_enabled);
			hw_device_state(udc->ep0out.qh.dma);
		}
	} else {
		hw_device_state(0);
		_gadget_stop_activity(&udc->gadget);
		if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
				CI13XXX_CONTROLLER_DISCONNECT_EVENT);
//		usb_gadget_set_state(&udc->gadget, USB_STATE_NOTATTACHED);
	}

	return 0;
}

#define VBUS_DRAW_BUF_LEN 10
#define MAX_OVERRIDE_VBUS_ALLOWED 900	/* 900 mA */
static char vbus_draw_mA[VBUS_DRAW_BUF_LEN];
module_param_string(vbus_draw_mA, vbus_draw_mA, VBUS_DRAW_BUF_LEN,
			S_IRUGO | S_IWUSR);

static int ci13xxx_vbus_draw(struct usb_gadget *_gadget, unsigned int mA)
{
	struct ci13xxx *udc = container_of(_gadget, struct ci13xxx, gadget);
	unsigned int override_mA = 0;

	/* override param to draw more current if battery draining faster */
	if ((mA == CONFIG_USB_GADGET_VBUS_DRAW) &&
		(vbus_draw_mA[0] != '\0')) {
		if ((!kstrtoint(vbus_draw_mA, 10, &override_mA)) &&
				(override_mA <= MAX_OVERRIDE_VBUS_ALLOWED)) {
			mA = override_mA;
		}
	}

	if (udc->transceiver)
		return usb_phy_set_power(udc->transceiver, mA);
	return -ENOTSUPP;
}

static int ci13xxx_pullup(struct usb_gadget *_gadget, int is_active)
{
	struct ci13xxx *udc = container_of(_gadget, struct ci13xxx, gadget);
	unsigned long flags;

	spin_lock_irqsave(udc->lock, flags);
	udc->softconnect = is_active;
	if (((udc->udc_driver->flags & CI13XXX_PULLUP_ON_VBUS) &&
			!udc->vbus_active) || !udc->driver) {
		spin_unlock_irqrestore(udc->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(udc->lock, flags);

	pm_runtime_get_sync(&_gadget->dev);

	/* Enable BAM (if needed) before starting controller */
	if (is_active) {
		msm_usb_bam_enable(CI_CTRL, _gadget->bam2bam_func_enabled);
	}

	spin_lock_irqsave(udc->lock, flags);
	if (!udc->vbus_active) {
		spin_unlock_irqrestore(udc->lock, flags);
		pm_runtime_put_sync(&_gadget->dev);
		return 0;
	}
	if (is_active) {
		spin_unlock(udc->lock);
		if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
				CI13XXX_CONTROLLER_CONNECT_EVENT);
		spin_lock(udc->lock);
		hw_device_state(udc->ep0out.qh.dma);
	} else {
		hw_device_state(0);
	}
	spin_unlock_irqrestore(udc->lock, flags);

	pm_runtime_mark_last_busy(&_gadget->dev);
	pm_runtime_put_autosuspend(&_gadget->dev);

	return 0;
}

static int ci13xxx_start(struct usb_gadget *gadget,
			 struct usb_gadget_driver *driver);
static int ci13xxx_stop(struct usb_gadget *gadget,
			struct usb_gadget_driver *driver);

/**
 * Device operations part of the API to the USB controller hardware,
 * which don't involve endpoints (or i/o)
 * Check  "usb_gadget.h" for details
 */
static const struct usb_gadget_ops usb_gadget_ops = {
	.vbus_session	= ci13xxx_vbus_session,
	.wakeup		= ci13xxx_wakeup,
	.vbus_draw	= ci13xxx_vbus_draw,
	.pullup		= ci13xxx_pullup,
	.udc_start	= ci13xxx_start,
	.udc_stop	= ci13xxx_stop,
};

/**
 * ci13xxx_start: register a gadget driver
 * @gadget: our gadget
 * @driver: the driver being registered
 *
 * Interrupts are enabled here.
 */
static int ci13xxx_start(struct usb_gadget *gadget,
			 struct usb_gadget_driver *driver)
{
	struct ci13xxx *udc = _udc;
	unsigned long flags;
	int retval = -ENOMEM;

	if (driver             == NULL ||
	    driver->setup      == NULL ||
	    driver->disconnect == NULL)
		return -EINVAL;
	else if (udc         == NULL)
		return -ENODEV;
	else if (udc->driver != NULL)
		return -EBUSY;

	spin_lock_irqsave(udc->lock, flags);

	info("hw_ep_max = %d", hw_ep_max);

	udc->gadget.dev.driver = NULL;

	spin_unlock_irqrestore(udc->lock, flags);

	pm_runtime_get_sync(&udc->gadget.dev);

	udc->ep0out.ep.desc = &ctrl_endpt_out_desc;
	retval = usb_ep_enable(&udc->ep0out.ep);
	if (retval)
		goto pm_put;

	udc->ep0in.ep.desc = &ctrl_endpt_in_desc;
	retval = usb_ep_enable(&udc->ep0in.ep);
	if (retval)
		goto pm_put;
	udc->status = usb_ep_alloc_request(&udc->ep0in.ep, GFP_KERNEL);
	if (!udc->status) {
		retval = -ENOMEM;
		goto pm_put;
	}

	udc->status_buf = kzalloc(2 + udc->gadget.extra_buf_alloc,
				GFP_KERNEL); /* for GET_STATUS */
	if (!udc->status_buf) {
		usb_ep_free_request(&udc->ep0in.ep, udc->status);
		retval = -ENOMEM;
		goto pm_put;
	}
	spin_lock_irqsave(udc->lock, flags);

	udc->gadget.ep0 = &udc->ep0in.ep;
	/* bind gadget */
	driver->driver.bus     = NULL;
	udc->gadget.dev.driver = &driver->driver;

	udc->driver = driver;
	if (udc->udc_driver->flags & CI13XXX_PULLUP_ON_VBUS) {
		if (udc->vbus_active) {
			if (udc->udc_driver->flags & CI13XXX_REGS_SHARED)
				hw_device_reset(udc);
		} else {
			goto done;
		}
	}

	if (!udc->softconnect)
		goto done;

	retval = hw_device_state(udc->ep0out.qh.dma);

done:
	spin_unlock_irqrestore(udc->lock, flags);

	if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
				CI13XXX_CONTROLLER_UDC_STARTED_EVENT);
pm_put:
	pm_runtime_put(&udc->gadget.dev);

	return retval;
}

/**
 * ci13xxx_stop: unregister a gadget driver
 *
 * Check usb_gadget_unregister_driver() at "usb_gadget.h" for details
 */
static int ci13xxx_stop(struct usb_gadget *gadget,
			struct usb_gadget_driver *driver)
{
	struct ci13xxx *udc = _udc;
	unsigned long flags;

	if (driver             == NULL ||
	    driver->unbind     == NULL ||
	    driver->setup      == NULL ||
	    driver->disconnect == NULL ||
	    driver             != udc->driver)
		return -EINVAL;

	spin_lock_irqsave(udc->lock, flags);

	if (!(udc->udc_driver->flags & CI13XXX_PULLUP_ON_VBUS) ||
			udc->vbus_active) {
		hw_device_state(0);
		spin_unlock_irqrestore(udc->lock, flags);
		_gadget_stop_activity(&udc->gadget);
		spin_lock_irqsave(udc->lock, flags);
	}

	spin_unlock_irqrestore(udc->lock, flags);

	usb_ep_free_request(&udc->ep0in.ep, udc->status);
	kfree(udc->status_buf);

	return 0;
}

/******************************************************************************
 * BUS block
 *****************************************************************************/
/**
 * udc_irq: global interrupt handler
 *
 * This function returns IRQ_HANDLED if the IRQ has been handled
 * It locks access to registers
 */
static irqreturn_t udc_irq(void)
{
	struct ci13xxx *udc = _udc;
	irqreturn_t retval;
	u32 intr;
	if (udc == NULL) {
		return IRQ_HANDLED;
	}

	spin_lock(udc->lock);

	if (udc->udc_driver->in_lpm && udc->udc_driver->in_lpm(udc)) {
		spin_unlock(udc->lock);
		return IRQ_NONE;
	}

	if (udc->udc_driver->flags & CI13XXX_REGS_SHARED) {
		if (hw_cread(CAP_USBMODE, USBMODE_CM) !=
				USBMODE_CM_DEVICE) {
			spin_unlock(udc->lock);
			return IRQ_NONE;
		}
	}
	intr = hw_test_and_clear_intr_active();
	if (intr) {
		/* order defines priority - do NOT change it */
		if (USBi_URI & intr) {
			printk("%s: Interrupt: Reset\n", __func__);
			if (!hw_cread(CAP_PORTSC, PORTSC_PR)) 
				pr_info("%s: USB reset interrupt is delayed\n",
								__func__);
			isr_reset_handler(udc);
		}
		if (USBi_PCI & intr) {
			printk("%s: Interrupt: Resume\n", __func__);
			isr_resume_handler(udc);
		}
		if (USBi_UI & intr) {
			isr_tr_complete_handler(udc);
		}
		if (USBi_SLI & intr) {
			printk("%s: Interrupt: Suspend\n", __func__);
			isr_suspend_handler(udc);
		}
		retval = IRQ_HANDLED;
	} else {
		retval = IRQ_NONE;
	}
	spin_unlock(udc->lock);

	return retval;
}

static void destroy_eps(struct ci13xxx *ci)
{
	int i;

	for (i = 0; i < hw_ep_max; i++) {
		struct ci13xxx_ep *mEp = &ci->ci13xxx_ep[i];

		dma_pool_free(ci->qh_pool, mEp->qh.ptr, mEp->qh.dma);
	}
}

/**
 * udc_probe: parent probe must call this to initialize UDC
 * @dev:  parent device
 * @regs: registers base address
 * @name: driver name
 *
 * This function returns an error code
 * No interrupts active, the IRQ has not been requested yet
 * Kernel assumes 32-bit DMA operations by default, no need to dma_set_mask
 */
static int udc_probe(struct ci13xxx_udc_driver *driver, struct device *dev,
		void __iomem *regs)
{
	struct ci13xxx *udc;
	struct ci13xxx_platform_data *pdata;
	int retval = 0, i, j;

	if (dev == NULL || regs == NULL || driver == NULL ||
			driver->name == NULL)
		return -EINVAL;

	udc = kzalloc(sizeof(struct ci13xxx), GFP_KERNEL);
	if (udc == NULL)
		return -ENOMEM;

	udc->lock = &udc_lock;
	udc->regs = regs;
	udc->udc_driver = driver;

	udc->gadget.ops          = &usb_gadget_ops;
	udc->gadget.speed        = USB_SPEED_UNKNOWN;
	udc->gadget.max_speed    = USB_SPEED_HIGH;
	udc->gadget.is_otg       = 0;
	udc->gadget.name         = driver->name;

	/* alloc resources */
	udc->qh_pool = dma_pool_create("ci13xxx_qh", dev,
				       sizeof(struct ci13xxx_qh),
				       64, CI13XXX_PAGE_SIZE);
	if (udc->qh_pool == NULL) {
		retval = -ENOMEM;
		goto free_udc;
	}

	udc->td_pool = dma_pool_create("ci13xxx_td", dev,
				       sizeof(struct ci13xxx_td),
				       64, CI13XXX_PAGE_SIZE);
	if (udc->td_pool == NULL) {
		retval = -ENOMEM;
		goto free_qh_pool;
	}

	INIT_DELAYED_WORK(&udc->rw_work, usb_do_remote_wakeup);

	retval = hw_device_init(regs);
	if (retval < 0)
		goto free_qh_pool;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	for (i = 0; i < hw_ep_max; i++) {
		struct ci13xxx_ep *mEp = &udc->ci13xxx_ep[i];
		INIT_LIST_HEAD(&mEp->ep.ep_list);
		INIT_LIST_HEAD(&mEp->rw_queue);
		setup_timer(&mEp->prime_timer, ep_prime_timer_func,
			(unsigned long) mEp);
	}

	arch_setup_dma_ops(&udc->gadget.dev, 0, DMA_BIT_MASK(32), NULL, false);
	for (i = 0; i < hw_ep_max/2; i++) {
		for (j = RX; j <= TX; j++) {
			int k = i + j * hw_ep_max/2;
			struct ci13xxx_ep *mEp = &udc->ci13xxx_ep[k];

			scnprintf(mEp->name, sizeof(mEp->name), "ep%i%s", i,
					(j == TX)  ? "in" : "out");

			mEp->lock         = udc->lock;
			mEp->device       = &udc->gadget.dev;
			mEp->td_pool      = udc->td_pool;

			mEp->ep.name      = mEp->name;
			mEp->ep.ops       = &usb_ep_ops;
			usb_ep_set_maxpacket_limit(&mEp->ep,
				k ? USHRT_MAX : CTRL_PAYLOAD_MAX);

			INIT_LIST_HEAD(&mEp->qh.queue);
			mEp->qh.ptr = dma_pool_alloc(udc->qh_pool, GFP_KERNEL,
					&mEp->qh.dma);
			if (mEp->qh.ptr == NULL)
				retval = -ENOMEM;
			else
				memset(mEp->qh.ptr, 0, sizeof(*mEp->qh.ptr));

			/* skip ep0 out and in endpoints  */
			if (i == 0)
				continue;

			list_add_tail(&mEp->ep.ep_list, &udc->gadget.ep_list);
		}
	}

	if (retval)
		goto free_dma_pools;

	udc->gadget.ep0 = &udc->ep0in.ep;

	pdata = dev->platform_data;
	if (pdata) {
		udc->gadget.usb_core_id = pdata->usb_core_id;
		if (pdata->enable_axi_prefetch)
			udc->gadget.extra_buf_alloc = EXTRA_ALLOCATION_SIZE;
	}

	if (udc->udc_driver->flags & CI13XXX_REQUIRE_TRANSCEIVER) {
		udc->transceiver = usb_get_phy(USB_PHY_TYPE_USB2);
		if (udc->transceiver == NULL) {
			retval = -ENODEV;
			goto destroy_eps;
		}
	}

	if (!(udc->udc_driver->flags & CI13XXX_REGS_SHARED)) {
		retval = hw_device_reset(udc);
		if (retval)
			goto put_transceiver;
	}

	if (udc->transceiver) {
		retval = otg_set_peripheral(udc->transceiver->otg,
						&udc->gadget);
		if (retval)
			goto put_transceiver;
	}

	retval = usb_add_gadget_udc(dev, &udc->gadget);
	if (retval)
		goto remove_trans;

	pm_runtime_no_callbacks(&udc->gadget.dev);
	pm_runtime_set_active(&udc->gadget.dev);
	pm_runtime_enable(&udc->gadget.dev);

	/* Use delayed LPM especially for composition-switch in LPM (suspend) */
	pm_runtime_set_autosuspend_delay(&udc->gadget.dev, 2000);
	pm_runtime_use_autosuspend(&udc->gadget.dev);

	_udc = udc;
	return retval;

remove_trans:
	if (udc->transceiver)
		otg_set_peripheral(udc->transceiver->otg, &udc->gadget);

	err("error = %i", retval);
put_transceiver:
	if (udc->transceiver)
		usb_put_phy(udc->transceiver);
destroy_eps:
	destroy_eps(udc);
free_dma_pools:
	dma_pool_destroy(udc->td_pool);
free_qh_pool:
	dma_pool_destroy(udc->qh_pool);
free_udc:
	kfree(udc);
	_udc = NULL;
	return retval;
}

/**
 * udc_remove: parent remove must call this to remove UDC
 *
 * No interrupts active, the IRQ has been released
 */
static void udc_remove(void)
{
	struct ci13xxx *udc = _udc;

	if (udc == NULL) {
		err("EINVAL");
		return;
	}

	usb_del_gadget_udc(&udc->gadget);

	if (udc->transceiver) {
		otg_set_peripheral(udc->transceiver->otg, &udc->gadget);
		usb_put_phy(udc->transceiver);
	}
	destroy_eps(udc);
	dma_pool_destroy(udc->td_pool);
	dma_pool_destroy(udc->qh_pool);

	kfree(udc);
	_udc = NULL;
}
