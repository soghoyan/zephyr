/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * USB device controller (UDC) driver skeleton
 *
 * This is a skeleton for a device controller driver using the UDC API.
 * Please use it as a starting point for a driver implementation for your
 * USB device controller. Maintaining a common style, terminology and
 * abbreviations will allow us to speed up reviews and reduce maintenance.
 * Copy UDC driver skeleton, remove all unrelated comments and replace the
 * copyright notice with your own.
 *
 * Typically, a driver implementation contains only a single source file,
 * but the large list of e.g. register definitions should be in a separate
 * .h file.
 *
 * If you want to define a helper macro, check if there is something similar
 * in include/zephyr/sys/util.h or include/zephyr/usb/usb_ch9.h that you can use.
 * Please keep all identifiers and logging messages concise and clear.
 */

#include "udc_common.h"

#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>

#include<soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_ch58x, CONFIG_UDC_DRIVER_LOG_LEVEL);

struct udc_ch58x_ep_reg_map {
	uint8_t	ctrl;
	uint8_t	tlen;
	uint8_t	dma;
	uint8_t	mod;
	uint8_t	en;
	uint16_t buf0;
	uint16_t buf1;
};

#define R16_PIN_ANALOG_IE   0x4000101A // RW, analog pin enable and digital input disable
#define  RB_PIN_ADC8_9_IE   0x01                      // RW, ADC/TouchKey channel 9/8 digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_ADC6_7_IE   0x02                      // RW, ADC/TouchKey channel 7/6 digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_ADC10_IE    0x04                      // RW, ADC/TouchKey channel 10 digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_ADC11_IE    0x08                      // RW, ADC/TouchKey channel 11 digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_USB2_DP_PU  0x10                      // RW, USB2 UDP internal pullup resistance enable: 0=enable/disable by RB_UC_DEV_PU_EN, 1=enable pullup, replace RB_UC_DEV_PU_EN under sleep mode
#define  RB_PIN_USB2_IE     0x20                      // RW, USB2 analog I/O enable: 0=analog I/O disable, 1=analog I/O enable
#define  RB_PIN_USB_DP_PU   0x40                      // RW, USB UDP internal pullup resistance enable: 0=enable/disable by RB_UC_DEV_PU_EN, 1=enable pullup, replace RB_UC_DEV_PU_EN under sleep mode
#define  RB_PIN_USB_IE      0x80                      // RW, USB analog I/O enable: 0=analog I/O disable, 1=analog I/O enable
#define  RB_PIN_ADC0_IE     0x0200                    // RW, ADC/TouchKey channel 0 digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_ADC1_IE     0x0400                    // RW, ADC/TouchKey channel 1 digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_ADC12_IE    0x0800                    // RW, ADC/TouchKey channel 12 digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_ADC13_IE    0x1000                    // RW, ADC/TouchKey channel 13 digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_XT32K_IE    0x2000                    // RW, external 32KHz oscillator digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_ADC2_3_IE   0x4000                    // RW, ADC/TouchKey channel 2/3 digital input disable: 0=digital input enable, 1=digital input disable
#define  RB_PIN_ADC4_5_IE   0x8000                    // RW, ADC/TouchKey channel 4/5 digital input disable: 0=digital input enable, 1=digital input disable

#define EP_MAX_PKT_SZ 64
#define DBL_BUF_CNT	3
#define SGL_BUF_CNT 4
#define CTRL_BUF_SZ 64
#define BIDIR_EP_MASK 0x03
#define EP_IS_BIDIR(n) (((n) & BIDIR_EP_MASK) && ((n) & BIDIR_EP_MASK) == (n))
#define BUF(i) (EP_MAX_PKT_SZ * (i))

#ifdef UDC_CH58X_DOUBLE_BUF
#define DMA_BUF_TOTAL_SZ(ep_cnt) (\
		BUF(3) + (               /* 1 buf for ep 0 and 2 for ep 4*/ \
			(ep_cnt) < 5 ? BUF(((ep_cnt) - 1) * 4): /*4 buf per ep 1~3*/\
			(ep_cnt) < 6 ? BUF(3 * 4):              /*skip ep 4 */\
			BUF(3 * 4 + ((ep_cnt) - 5) * 2)         /*2 buf per ep 5~7*/\
		)\
	)
#else
#define DMA_BUF_TOTAL_SZ(ep_cnt) (\
		BUF(3) + (                /* 1 buf for ep 0 and 2 for ep 4*/ \
			(ep_cnt) < 5 ? BUF(((ep_cnt) - 1) * 2): /*4 buf per ep 1~3*/\
			(ep_cnt) < 6 ? BUF(3 * 2):              /*skip ep 4 */\
			BUF(3 * 2 + ((ep_cnt) - 5) * 2)         /*2 buf per ep 5~7*/\
		)\
	)
#endif
/*
 * Structure for holding controller configuration items that can remain in
 * non-volatile memory. This is usually accessed as
 *   const struct udc_ch58x_config *config = dev->config;
 */
struct udc_ch58x_config {
	mem_addr_t reg;
	size_t num_of_eps;
	uint8_t *dma_buf;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	void (*udc_irq_enable)(const struct device *dev);
	void (*udc_irq_disable)(const struct device *dev);
	int speed_idx;
};

/*
 * Structure to hold driver private data.
 * Note that this is not accessible via dev->data, but as
 *   struct udc_ch58x_data *priv = udc_get_private(dev);
 */
struct udc_ch58x_data {
	const struct device *dev;
	struct k_msgq *msgq;
	struct k_work work;
	struct net_buf *setup;
	struct net_buf *ctrl_out;
};

enum udc_ch58x_ep_event_type {
	/* Trigger next transfer, must not be used for control OUT */
	CH58X_EP_EVT_XFER,
	/* Setup packet received */
	CH58X_EP_EVT_SETUP,
	/* OUT transaction for specific endpoint is finished */
	CH58X_EP_EVT_DOUT,
	/* IN transaction for specific endpoint is finished */
	CH58X_EP_EVT_DIN,
	/* Workaround for clear halt in ISR */
	CH58X_EP_EVT_CLEAR_HALT,
};

/* Structure for driver's endpoint events */
struct udc_ch58x_ep_event {
	enum udc_ch58x_ep_event_type event;
	uint8_t ep;
	uint8_t len;
};

static int udc_ch58x_ep_dequeue(const struct device *dev,
				   struct udc_ep_config *const cfg);

#ifdef UDC_CH58X_DOUBLE_BUF
static const struct udc_ch58x_ep_reg_map reg_map[] = {
	{R8_UEP0_CTRL, R8_UEP0_T_LEN, R16_UEP0_DMA, 0, 				0,					BUF(0),  BUF(0)},
	{R8_UEP1_CTRL, R8_UEP1_T_LEN, R16_UEP1_DMA, R8_UEP4_1_MOD,	RB_UEP1_EN_SHIFT,	BUF(3),  BUF(4)},
	{R8_UEP2_CTRL, R8_UEP2_T_LEN, R16_UEP2_DMA, R8_UEP2_3_MOD,	RB_UEP2_EN_SHIFT,	BUF(7),  BUF(8)},
	{R8_UEP3_CTRL, R8_UEP3_T_LEN, R16_UEP3_DMA, R8_UEP2_3_MOD,	RB_UEP3_EN_SHIFT,	BUF(11), BUF(12)},
	{R8_UEP4_CTRL, R8_UEP4_T_LEN, 0,			R8_UEP4_1_MOD,	RB_UEP4_EN_SHIFT,	BUF(1),  BUF(2)},
	{R8_UEP5_CTRL, R8_UEP5_T_LEN, R16_UEP5_DMA, R8_UEP567_MOD,	RB_UEP5_EN_SHIFT,	BUF(15), BUF(16)},
	{R8_UEP6_CTRL, R8_UEP6_T_LEN, R16_UEP6_DMA, R8_UEP567_MOD,	RB_UEP6_EN_SHIFT,	BUF(17), BUF(18)},
	{R8_UEP7_CTRL, R8_UEP7_T_LEN, R16_UEP7_DMA, R8_UEP567_MOD,	RB_UEP7_EN_SHIFT,	BUF(19), BUF(20)},
};
#else
static const struct udc_ch58x_ep_reg_map reg_map[] = {
	{R8_UEP0_CTRL, R8_UEP0_T_LEN, R16_UEP0_DMA, 0, 				0,					BUF(0),  BUF(0)},
	{R8_UEP1_CTRL, R8_UEP1_T_LEN, R16_UEP1_DMA, R8_UEP4_1_MOD,	RB_UEP1_EN_SHIFT,	BUF(3),  BUF(4)},
	{R8_UEP2_CTRL, R8_UEP2_T_LEN, R16_UEP2_DMA, R8_UEP2_3_MOD,	RB_UEP2_EN_SHIFT,	BUF(5),  BUF(6)},
	{R8_UEP3_CTRL, R8_UEP3_T_LEN, R16_UEP3_DMA, R8_UEP2_3_MOD,	RB_UEP3_EN_SHIFT,	BUF(7),  BUF(8)},
	{R8_UEP4_CTRL, R8_UEP4_T_LEN, 0,			R8_UEP4_1_MOD,	RB_UEP4_EN_SHIFT,	BUF(1),  BUF(2)},
	{R8_UEP5_CTRL, R8_UEP5_T_LEN, R16_UEP5_DMA, R8_UEP567_MOD,	RB_UEP5_EN_SHIFT,	BUF(9), BUF(10)},
	{R8_UEP6_CTRL, R8_UEP6_T_LEN, R16_UEP6_DMA, R8_UEP567_MOD,	RB_UEP6_EN_SHIFT,	BUF(11), BUF(12)},
	{R8_UEP7_CTRL, R8_UEP7_T_LEN, R16_UEP7_DMA, R8_UEP567_MOD,	RB_UEP7_EN_SHIFT,	BUF(13), BUF(14)},
};
#endif

static void ch58x_event_submit(const struct device *dev, 
	enum udc_ch58x_ep_event_type event, uint8_t ep, uint8_t len){
	struct udc_ch58x_ep_event evt = {
		.ep = ep,
		.event = event,
		.len = len
	};
	struct udc_ch58x_data *priv = udc_get_private(dev);
	if(k_msgq_put(priv->msgq, &evt, K_NO_WAIT)){
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOMSG);
		LOG_ERR("Failed to submit message, type %d", evt.event);
	}else{
		k_work_submit_to_queue(udc_get_work_q(), &priv->work);
	}
}

static inline int work_handler_ctrl_ep_buf_done(const struct device *dev,
				  struct net_buf *buf, struct udc_ch58x_ep_event *ev)
{
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf_out;
	int err = 0;

	switch (ev->event) {
	case CH58X_EP_EVT_SETUP:
		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);
		udc_ch58x_ep_dequeue(dev, ep_cfg);
		udc_ch58x_ep_dequeue(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
		udc_ep_set_busy(dev, USB_CONTROL_EP_IN, false);
	struct usb_setup_packet *s = (void *)buf->data;
	LOG_INF("Setup: ty %x rq %x vl %x ix %x ln %x", s->bmRequestType, s->bRequest, s->wValue, s->wIndex, s->wLength);

		if (udc_ctrl_stage_is_data_out(dev)) {
			/*  Allocate and feed buffer for data OUT stage */
			LOG_DBG("s:%p|feed for -out-", buf);
			buf_out = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, udc_data_stage_length(buf));
			if (buf_out) {
				udc_buf_put(ep_cfg, buf_out);
			}else{
				err = udc_submit_ep_event(dev, buf, -ENOMEM);
			}
		} else if (udc_ctrl_stage_is_data_in(dev)) {
			/*
			* Here we have to feed both descriptor tables so that
			* no setup packets are lost in case of successive
			* status OUT stage and next setup.
			*/
			LOG_DBG("s:%p|feed for -in-status >setup", buf);

			/* Finally alloc buffer for IN and submit to upper layer */
			if (err == 0) {
				err = udc_ctrl_submit_s_in_status(dev);
			}
		} else {
			LOG_DBG("s:%p|feed >setup", buf);
			/*
			* For all other cases we feed with a buffer
			* large enough for setup packet.
			*/
	//		err = usbfsotg_ctrl_feed_dout(dev, 8U, false, true);
			if (err == 0) {
				err = udc_ctrl_submit_s_status(dev);
			}
		}
		break;
	case CH58X_EP_EVT_DOUT:
		if (udc_ctrl_stage_is_status_out(dev)) {
			/* s-in-status finished, next bd is already fed */
			LOG_DBG("dout:%p|no feed", buf);
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		} else {
			/*
			 * For all other cases we feed with a buffer
			 * large enough for setup packet.
			 */
			LOG_DBG("dout:%p|feed >setup", buf);
//			err = usbfsotg_ctrl_feed_dout(dev, 8U, false, false);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_in(dev)) {
			ep_cfg->stat.data1 = 1;
			udc_get_ep_cfg(dev, USB_CONTROL_EP_IN)->stat.data1 = 1;
			err = udc_ctrl_submit_s_out_status(dev, buf);
		}
		break;
	case CH58X_EP_EVT_DIN:
		if (udc_ctrl_stage_is_status_in(dev) ||
		    udc_ctrl_stage_is_no_data(dev)) {
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_out(dev)) {
			ep_cfg->stat.data1 = 1;
			udc_get_ep_cfg(dev, USB_CONTROL_EP_IN)->stat.data1 = 1;
			/*
			 * IN transfer finished, release buffer,
			 * control OUT buffer should be already fed.
			 */
			LOG_INF("din:%p|feed for status-out-", buf);
			buf_out = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, udc_mps_ep_size(ep_cfg));
			if (buf_out) {
//			LOG_INF("din:%p|feed for status-out-", buf);
				udc_buf_put(ep_cfg, buf_out);
				ch58x_event_submit(dev, CH58X_EP_EVT_XFER, USB_CONTROL_EP_OUT, 0);
				net_buf_unref(buf);
			}else{
				err = udc_submit_ep_event(dev, buf, -ENOMEM);
			}
		}
		break;
	default:
	}
	return err;
}
static int ch58x_xfer_start(const struct device *dev, struct udc_ep_config *ep_cfg){
	const struct udc_ch58x_config *cfg = dev->config;
	const struct udc_ch58x_ep_reg_map *rmap = &reg_map[USB_EP_GET_IDX(ep_cfg->addr)];
	struct net_buf *buf;
	unsigned int lock_key;
	uint8_t val, len;
	if(ep_cfg->stat.halted){
//		LOG_WRN("Ep %02x is halted", ep_cfg->addr);
		return -ECONNRESET;
	}
	if ((buf = udc_buf_peek(dev, ep_cfg->addr)) == NULL) {
//		LOG_WRN("Ep %02x no buf", ep_cfg->addr);
		return -ENOBUFS;
	}
	if(USB_EP_DIR_IS_OUT(ep_cfg->addr)){
		if (net_buf_tailroom(buf) < udc_mps_ep_size(ep_cfg)){
			return -EBADMSG;
		}
		lock_key = irq_lock();
		val = sys_read8(cfg->reg + rmap->ctrl);
		val = (val & ~(MASK_UEP_R_RES | RB_UEP_R_TOG)) | UEP_R_RES_ACK;
		val |= ep_cfg->stat.data1 ? RB_UEP_R_TOG : 0;
		sys_write8(val, cfg->reg + rmap->ctrl);
		irq_unlock(lock_key);

//LOG_WRN("buf->len rx %x %02x",net_buf_tailroom(buf), sys_read8(cfg->reg + rmap->ctrl));
	}else{
		len = (uint8_t)MIN(buf->len, udc_mps_ep_size(ep_cfg));
		if(len /*|| udc_ep_buf_has_zlp(buf)*/){
			bool out_en = (sys_read8(cfg->reg + rmap->mod) & (RB_UEP_RX_EN << rmap->en)) != 0;
			memcpy(cfg->dma_buf + (out_en ? rmap->buf1 : rmap->buf0), buf->data, len);
		}else{
//		LOG_WRN("Ep %02x 0 len buf", ep_cfg->addr);
//			return -EBADMSG;
		}
		lock_key = irq_lock();
		sys_write8(len, cfg->reg + rmap->tlen);
		val = sys_read8(cfg->reg + rmap->ctrl);
		val = (val & ~(MASK_UEP_T_RES | RB_UEP_T_TOG)) | UEP_T_RES_ACK;
		val |= ep_cfg->stat.data1 ? RB_UEP_T_TOG : 0;
		sys_write8(val, cfg->reg + rmap->ctrl);
		irq_unlock(lock_key);
//LOG_WRN("buf->len tx %x %02x",buf->len, sys_read8(cfg->reg + rmap->ctrl));
	}
	return 0;
}
static int xfer_fifo_handler(const struct device *dev, struct udc_ch58x_ep_event *ev){
	const struct udc_ch58x_config *cfg = dev->config;
	const struct udc_ch58x_ep_reg_map *rmap = &reg_map[USB_EP_GET_IDX(ev->ep)];
	struct udc_ch58x_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ev->ep);
	struct net_buf *buf;
	unsigned int lock_key;
	uint8_t val, len;
	int err = 0;
	switch (ev->event) {
	case CH58X_EP_EVT_SETUP:
		buf = priv->setup;
		if(buf == NULL && (buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, USB_CONTROL_EP_MPS)) == NULL){
			err = -ENOBUFS;
			break;
		}
		priv->setup = buf;
		net_buf_add_mem(buf, cfg->dma_buf + rmap->buf0, ev->len);
	memset(cfg->dma_buf + rmap->buf0, 0, 8);
		ep_cfg->stat.data1 = 1;
		udc_ep_buf_set_setup(buf);
		udc_get_ep_cfg(dev, USB_CONTROL_EP_IN)->stat.data1 = 1;
		break;
	case CH58X_EP_EVT_DOUT:
		ep_cfg->stat.data1 ^= 1;
		if ((buf = udc_buf_peek(dev, ev->ep)) == NULL) {
			err = -ENOBUFS;
			break;
		}
		net_buf_add_mem(buf, cfg->dma_buf + rmap->buf0, ev->len);

		if (net_buf_tailroom(buf) >= udc_mps_ep_size(ep_cfg) &&
				ev->len == udc_mps_ep_size(ep_cfg)) {
			lock_key = irq_lock();
			val = sys_read8(cfg->reg + rmap->ctrl);
			val = (val & ~(MASK_UEP_R_RES | RB_UEP_R_TOG)) | UEP_R_RES_ACK;
			val |= ep_cfg->stat.data1 ? RB_UEP_R_TOG : 0;
			sys_write8(val, cfg->reg + rmap->ctrl);
			irq_unlock(lock_key);
			err = -EAGAIN;
		} else {
//			udc_ep_set_busy(dev, ev->ep, false);
		}
		break;
	case CH58X_EP_EVT_DIN:
		ep_cfg->stat.data1 ^= 1;
		buf = udc_buf_peek(dev, ep_cfg->addr);
		if ((buf = udc_buf_peek(dev, ev->ep)) == NULL) {
			err = -ENOBUFS;
			break;
		}

		net_buf_pull(buf, ev->len);
		len = (uint8_t)MIN(buf->len, udc_mps_ep_size(ep_cfg));
		if(len){
			bool out_en = (sys_read8(cfg->reg + rmap->mod) & (RB_UEP_RX_EN << rmap->en)) != 0;
			memcpy(cfg->dma_buf + (out_en ? rmap->buf1 : rmap->buf0), buf->data, len);
		}else if(udc_ep_buf_has_zlp(buf)){
			udc_ep_buf_clear_zlp(buf);
		}else{
//			udc_ep_set_busy(dev, ev->ep, false);
			break;
		}
		lock_key = irq_lock();
		sys_write8(len, cfg->reg + rmap->tlen);
		val = sys_read8(cfg->reg + rmap->ctrl);
		val = (val & ~(MASK_UEP_T_RES | RB_UEP_T_TOG)) | UEP_T_RES_ACK;
		val |= ep_cfg->stat.data1 ? RB_UEP_T_TOG : 0;
		sys_write8(val, cfg->reg + rmap->ctrl);
		irq_unlock(lock_key);
		err = -EAGAIN;
		break;
	// case USBFSOTG_EVT_CLEAR_HALT:
	// 	err = usbfsotg_ep_clear_halt(dev, ep_cfg);
	case CH58X_EP_EVT_XFER:
	default:
		break;
	}
	return err;
}
static void xfer_work_handler(struct k_work *item)
{
	struct udc_ch58x_ep_event ev;
	struct udc_ch58x_data *priv = CONTAINER_OF(item, struct udc_ch58x_data, work);
	const struct device *dev = priv->dev;

	while (!k_msgq_get(priv->msgq, &ev, K_NO_WAIT)) {
		struct udc_ep_config *ep_cfg;
		struct net_buf *buf;
		int err = 0;

		LOG_DBG("dev %p, ep 0x%02x, event %u", dev, ev.ep, ev.event);
		ep_cfg = udc_get_ep_cfg(dev, ev.ep);
		if (unlikely(ep_cfg == NULL)) {
			udc_submit_event(dev, UDC_EVT_ERROR, -ENODATA);
			goto xfer_work_error;
		}
		if(ev.event != CH58X_EP_EVT_XFER){
			err = xfer_fifo_handler(dev, &ev);
			if(err){
				if(err != -EAGAIN){
					LOG_ERR("No buf ep 0x%02x", ev.ep);
					udc_submit_event(dev, UDC_EVT_ERROR, err);
				}
				continue;
			}


			if(ev.event == CH58X_EP_EVT_SETUP){
				buf = priv->setup;
				priv->setup = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8);
				if (priv->setup == NULL) {
					err = -ENOBUFS;
				}
			}else{
				buf = udc_buf_get(dev, ev.ep);
			}
			if (buf == NULL) {// should never happen
				goto xfer_work_error;
			}
			if(USB_EP_GET_IDX(ev.ep) == 0){
				err = work_handler_ctrl_ep_buf_done(dev, buf, &ev);
			}else{
				err = udc_submit_ep_event(dev, buf, 0);
			}
			udc_ep_set_busy(dev, ev.ep, false);

		}
		else{
//LOG_INF("Xfer ep %02x busy %d", ep_cfg->addr, udc_ep_is_busy(dev, ev.ep));
		}
		if (unlikely(err)) {
			udc_submit_event(dev, UDC_EVT_ERROR, err);
		}

		/* Peek next transfer */
		if (!udc_ep_is_busy(dev, ev.ep)){
			if (ch58x_xfer_start(dev, ep_cfg) == 0) {
//LOG_INF("Xfer start ep %02x ctr %x len %x", ep_cfg->addr, sys_read8(0x40008022), sys_read8(0x40008020));
				udc_ep_set_busy(dev, ev.ep, true);
			}
		}

xfer_work_error:
	}
}

static void udc_ch58x_isr(const struct device *dev){
	const struct udc_ch58x_config *cfg = dev->config;
	uint8_t int_flg = sys_read8(cfg->reg + R8_USB_INT_FG);
	uint8_t int_stat = sys_read8(cfg->reg + R8_USB_INT_ST);
	uint8_t misc_stat;

	if(int_flg & RB_UIF_TRANSFER){
//		isr_handle_xfer_done(dev, istatus, status);
		uint8_t ep_ix = (int_stat & MASK_UIS_TOKEN) != UIS_TOKEN_SETUP ? 
						int_stat & MASK_UIS_ENDP : USB_CONTROL_EP_OUT;
		const struct udc_ch58x_ep_reg_map *rmap = &reg_map[ep_ix];
		uint8_t val = sys_read8(cfg->reg + rmap->ctrl);

		switch(int_stat & MASK_UIS_TOKEN){
		case UIS_TOKEN_OUT:
//	LOG_INF("IRQ OUT ep %02x", ep_ix);
			if(unlikely(!(int_stat & RB_UIS_TOG_OK))){
LOG_ERR("Toggle Err");
				break;
			}
//if(ep_ix) LOG_WRN("Rx %x", sys_read8(cfg->reg + R8_USB_RX_LEN));
			val = (val & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
			sys_write8(val, cfg->reg + rmap->ctrl);
			ch58x_event_submit(dev, CH58X_EP_EVT_DOUT, 
				USB_EP_GET_ADDR(ep_ix, USB_EP_DIR_OUT), sys_read8(cfg->reg + R8_USB_RX_LEN));
			break;
		case UIS_TOKEN_IN:
			val = (val & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
			sys_write8(val, cfg->reg + rmap->ctrl);
			ch58x_event_submit(dev, CH58X_EP_EVT_DIN, 
				USB_EP_GET_ADDR(ep_ix, USB_EP_DIR_IN), sys_read8(cfg->reg + rmap->tlen));
			break;
		case UIS_TOKEN_SOF:
			udc_submit_event(dev, UDC_EVT_SOF, 0);
			break;
		case UIS_TOKEN_SETUP:
			if(!(int_stat & RB_UIS_SETUP_ACT)){
				break;
			}
			// TODO: Handle clear halt when halted
			struct udc_ep_config *ep_cfg;

			ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
			ep_cfg->stat.halted = false;
			ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
			ep_cfg->stat.halted = false;

			val &= ~(MASK_UEP_R_RES | MASK_UEP_T_RES);
			val |= UEP_R_RES_NAK | UEP_T_RES_NAK;
			sys_write8(val, cfg->reg + rmap->ctrl);
			ch58x_event_submit(dev, CH58X_EP_EVT_SETUP, 
				USB_CONTROL_EP_OUT, sizeof(struct usb_setup_packet));
		}

		sys_write8(RB_UIF_TRANSFER, cfg->reg + R8_USB_INT_FG);
		return;
	}

	misc_stat = sys_read8(cfg->reg + R8_USB_MIS_ST);
	if (int_flg & RB_UIF_FIFO_OV) {
		LOG_DBG("FIFO Overflow IRQ");
		udc_submit_event(dev, UDC_EVT_ERROR, -EOVERFLOW);
	}

	if (int_flg & RB_UIF_BUS_RST) {
		udc_submit_event(dev, UDC_EVT_RESET, 0);
	}

	if(int_flg & RB_UIF_SUSPEND){
		bool suspend = !!(misc_stat & RB_UMS_SUSPEND);
		udc_set_suspended(dev, suspend);
		udc_submit_event(dev, suspend ? UDC_EVT_SUSPEND : UDC_EVT_RESUME, 0);
	}
	sys_write8(int_flg, cfg->reg + R8_USB_INT_FG);
}

/*
 * This is called in the context of udc_ep_enqueue() and must
 * not block. The driver can immediately claim the buffer if the queue is empty,
 * but usually it is offloaded to a thread or workqueue to handle transfers
 * in a single location. Please refer to existing driver implementations
 * for examples.
 */
static int udc_ch58x_ep_enqueue(const struct device *dev,
				   struct udc_ep_config *const cfg,
				   struct net_buf *buf)
{
	LOG_DBG("%p enqueue %p", dev, buf);
	udc_buf_put(cfg, buf);

	if (cfg->stat.halted) {
		/*
		 * It is fine to enqueue a transfer for a halted endpoint,
		 * you need to make sure that transfers are retriggered when
		 * the halt is cleared.
		 *
		 * Always use the abbreviation 'ep' for the endpoint address
		 * and 'ep_idx' or 'ep_num' for the endpoint number identifiers.
		 * Although struct udc_ep_config uses address to be unambiguous
		 * in its context.
		 */
		LOG_DBG("ep 0x%02x halted", cfg->addr);
		return 0;
	}else{
		ch58x_event_submit(dev, CH58X_EP_EVT_XFER, cfg->addr, 0);
	}

	return 0;
}

/*
 * This is called in the context of udc_ep_dequeue()
 * and must remove all requests from an endpoint queue
 * Successful removal should be reported to the higher level with
 * ECONNABORTED as the request result.
 * It is up to the request owner to clean up or reuse the buffer.
 */
static int udc_ch58x_ep_dequeue(const struct device *dev,
				   struct udc_ep_config *const cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;

	lock_key = irq_lock();

	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	irq_unlock(lock_key);

	return 0;
}

/*
 * Configure and make an endpoint ready for use.
 * This is called in the context of udc_ep_enable() or udc_ep_enable_internal(),
 * the latter of which may be used by the driver to enable control endpoints.
 */
static int udc_ch58x_ep_enable(const struct device *dev,
				  struct udc_ep_config *const epcfg)
{
	const struct udc_ch58x_config *cfg = dev->config;
	struct udc_ch58x_data *priv = udc_get_private(dev);
	const struct udc_ch58x_ep_reg_map *rmap = &reg_map[USB_EP_GET_IDX(epcfg->addr)];
	uint8_t val, valen;
	unsigned int lock_key;

	LOG_DBG("Enable ep 0x%02x", epcfg->addr);

	lock_key = irq_lock();

	val = sys_read8(cfg->reg + rmap->ctrl);
	val &= USB_EP_DIR_IS_IN(epcfg->addr) ? 
		~(RB_UEP_T_TOG | MASK_UEP_T_RES) : ~(RB_UEP_R_TOG | MASK_UEP_R_RES);
	if(USB_EP_GET_IDX(epcfg->addr) == 0){
		if (epcfg->addr == USB_CONTROL_EP_IN) {
			sys_write8(val | UEP_T_RES_NAK, cfg->reg + rmap->ctrl);
		}else if (epcfg->addr == USB_CONTROL_EP_OUT) {
			struct net_buf *buf;

			buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, USB_CONTROL_EP_MPS);
			if(buf == NULL)
				return -ENOBUFS;
			priv->setup = buf;
			//udc_buf_put(epcfg, buf);
			sys_write8(val | UEP_R_RES_NAK, cfg->reg + rmap->ctrl);
		}
		return 0;
	}

	valen = sys_read8(cfg->reg + rmap->mod);
	if(USB_EP_DIR_IS_IN(epcfg->addr)){
		val |= (epcfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_ISO ? UEP_T_RES_TOUT : UEP_T_RES_NAK;
		valen |= RB_UEP_TX_EN << rmap->en; 
	}else{
		val |= (epcfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_ISO ? UEP_R_RES_TOUT : UEP_R_RES_NAK;
		valen |= RB_UEP_RX_EN << rmap->en; 
	}
	//val |= epcfg->attributes & USB_EP_TRANSFER_TYPE_MASK == USB_EP_TYPE_ISO ? 0 : RB_UEP_AUTO_TOG;
	sys_write8(val, cfg->reg + rmap->ctrl);
	sys_write8(valen, cfg->reg + rmap->mod);

	irq_unlock(lock_key);

	return 0;
}

/*
 * Opposite function to udc_ch58x_ep_enable(). udc_ep_disable_internal()
 * may be used by the driver to disable control endpoints.
 */
static int udc_ch58x_ep_disable(const struct device *dev,
				   struct udc_ep_config *const epcfg)
{
	const struct udc_ch58x_config *cfg = dev->config;
	struct udc_ch58x_data *priv = udc_get_private(dev);
	const struct udc_ch58x_ep_reg_map *rmap = &reg_map[USB_EP_GET_IDX(epcfg->addr)];
	uint8_t valen;
	LOG_DBG("Disable ep 0x%02x", epcfg->addr);

	if(epcfg->addr == USB_CONTROL_EP_OUT){
		struct net_buf *buf;
		buf = udc_buf_get_all(dev, epcfg->addr);
		if (buf) {
			net_buf_unref(buf);
		}
		if(priv->setup){
			net_buf_unref(priv->setup);
			priv->setup = NULL;
		}
		return 0;
	}

	valen = sys_read8(cfg->reg + rmap->mod);
	if(USB_EP_DIR_IS_IN(epcfg->addr)){
		valen &= ~(RB_UEP_TX_EN << rmap->en); 
	}else{
		valen &= ~(RB_UEP_RX_EN << rmap->en); 
	}
	sys_write8(valen, cfg->reg + rmap->mod);

	return 0;
}

/* Halt endpoint. Halted endpoint should respond with a STALL handshake. */
static int udc_ch58x_ep_set_halt(const struct device *dev,
				    struct udc_ep_config *const ep_cfg)
{
	const struct udc_ch58x_config *cfg = dev->config;
	const struct udc_ch58x_ep_reg_map *rmap = &reg_map[USB_EP_GET_IDX(ep_cfg->addr)];
	uint8_t val;
	unsigned int lock_key;
	LOG_DBG("Set halt ep 0x%02x", ep_cfg->addr);

	lock_key = irq_lock();
	val = sys_read8(cfg->reg + rmap->ctrl);
	if(USB_EP_DIR_IS_IN(ep_cfg->addr)){
		val = (val & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_STALL;
	}else{
		val = (val & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_STALL;
	}
	sys_write8(val, cfg->reg + rmap->ctrl);
	ep_cfg->stat.halted = true;
	irq_unlock(lock_key);

	return 0;
}

/*
 * Opposite to halt endpoint. If there are requests in the endpoint queue,
 * the next transfer should be prepared.
 */
static int udc_ch58x_ep_clear_halt(const struct device *dev,
				      struct udc_ep_config *const ep_cfg)
{
	const struct udc_ch58x_config *cfg = dev->config;
	const struct udc_ch58x_ep_reg_map *rmap = &reg_map[USB_EP_GET_IDX(ep_cfg->addr)];
	uint8_t val;
	unsigned int lock_key;
	LOG_DBG("Clear halt ep 0x%02x", ep_cfg->addr);

	lock_key = irq_lock();
	val = sys_read8(cfg->reg + rmap->ctrl);
	if(USB_EP_DIR_IS_IN(ep_cfg->addr)){
		val = (val & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
	}else{
		val = (val & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
	}
	sys_write8(val, cfg->reg + rmap->ctrl);
	irq_unlock(lock_key);

	ch58x_event_submit(dev, CH58X_EP_EVT_XFER, ep_cfg->addr, 0);
	return 0;
}

static int udc_ch58x_set_address(const struct device *dev, const uint8_t addr)
{
	const struct udc_ch58x_config *cfg = dev->config;
	LOG_DBG("Set new address %u for %p", addr, dev);
	sys_write8(addr, cfg->reg + R8_USB_DEV_AD);

	return 0;
}

static int udc_ch58x_host_wakeup(const struct device *dev)
{
	LOG_DBG("Remote wakeup from %p", dev);

	return 0;
}

/* Return actual USB device speed */
static enum udc_bus_speed udc_ch58x_device_speed(const struct device *dev)
{
	struct udc_data *data = dev->data;

	return data->caps.hs ? UDC_BUS_SPEED_HS : UDC_BUS_SPEED_FS;
}

static int udc_ch58x_enable(const struct device *dev)
{
	const struct udc_ch58x_config *cfg = dev->config;
	mem_addr_t reg = cfg->reg;
	LOG_DBG("Enable device %p", dev);

	sys_write8(0xff, reg + R8_USB_INT_FG);
	sys_write8(sys_read8(reg + R8_USB_CTRL) | RB_UC_DEV_PU_EN, reg + R8_USB_CTRL);
	sys_write8(RB_UD_PD_DIS | RB_UD_PORT_EN, reg + R8_UDEV_CTRL);
	sys_write8(RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER, reg + R8_USB_INT_EN);
	cfg->udc_irq_enable(dev);

	return 0;
}

static int udc_ch58x_disable(const struct device *dev)
{
	const struct udc_ch58x_config *cfg = dev->config;
	mem_addr_t reg = cfg->reg;
	LOG_DBG("Disable device %p", dev);
	cfg->udc_irq_disable(dev);
	sys_write8(0, reg + R8_USB_INT_EN);
	sys_write8(0, reg + R8_UDEV_CTRL);
	sys_write8(sys_read8(reg + R8_USB_CTRL) & ~RB_UC_DEV_PU_EN, reg + R8_USB_CTRL);

	return 0;
}

/*
 * Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only udc_ch58x_enable() makes device visible to the host.
 */
static int udc_ch58x_init(const struct device *dev)
{
	const struct udc_ch58x_config *cfg = dev->config;
	const struct udc_ch58x_ep_reg_map *rmap = reg_map;
	uint16_t val;
	int i;

	sys_write8(0, cfg->reg + R8_USB_CTRL);
	sys_write8(0, cfg->reg + R8_USB_DEV_AD);
	// sys_write8(cfg->reg + R8_UEP4_1_MOD, RB_UEP1_BUF_MOD);
	// sys_write8(cfg->reg + R8_UEP2_3_MOD, RB_UEP2_BUF_MOD | RB_UEP3_BUF_MOD);
	sys_write8(0, cfg->reg + R8_UEP4_1_MOD);
	sys_write8(0, cfg->reg + R8_UEP2_3_MOD);
	sys_write8(0, cfg->reg + R8_UEP567_MOD);
	for(i = 0; i < cfg->num_of_eps; i++, rmap++){
		if(likely(rmap->dma)){
			sys_write16((uint16_t)(intptr_t)cfg->dma_buf + rmap->buf0, cfg->reg + rmap->dma);
//			sys_write8(cfg->reg + rmap->ctrl, RB_UEP_AUTO_TOG);
			sys_write8(0, cfg->reg + rmap->ctrl);
		}
	}
	val = sys_read16(R16_PIN_ANALOG_IE);
	val = (val & ~RB_PIN_USB_DP_PU) | RB_PIN_USB_IE;
	sys_write16(val, R16_PIN_ANALOG_IE);
	sys_write8(RB_UC_INT_BUSY | RB_UC_DMA_EN, cfg->reg + R8_USB_CTRL);

    // R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN; // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
    // R16_PIN_ANALOG_IE |= RB_PIN_USB_IE | RB_PIN_USB_DP_PU;         // 防止USB端口浮空及上拉电阻
    // R8_USB_INT_FG = 0xFF;                                          // 清中断标志
    // R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;                   // 允许USB端口
    // R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	return 0;
}

/* Shut down the controller completely */
static int udc_ch58x_shutdown(const struct device *dev)
{
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	return 0;
}

/*
 * This is called once to initialize the controller and endpoints
 * capabilities, and register endpoint structures.
 */
static int udc_ch58x_driver_preinit(const struct device *dev)
{
	const struct udc_ch58x_config *config = dev->config;
	struct udc_data *data = dev->data;
	uint16_t mps = EP_MAX_PKT_SZ;
	int err;

	/*
	 * You do not need to initialize it if your driver does not use
	 * udc_lock_internal() / udc_unlock_internal(), but implements its
	 * own mechanism.
	 */
	k_mutex_init(&data->mutex);
//	k_work_init(&priv->work, xfer_work_handler);

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;
	if (config->speed_idx == 2) {
		data->caps.hs = true;
		mps = 1024;
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = mps;
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = mps;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);

	return 0;
}

static int udc_ch58x_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int udc_ch58x_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

/*
 * UDC API structure.
 * Note, you do not need to implement basic checks, these audc_ch58x_dma_buf_##nre done by
 * the UDC common layer udc_common.c
 */
static const struct udc_api udc_ch58x_api = {
	.lock = udc_ch58x_lock,
	.unlock = udc_ch58x_unlock,
	.device_speed = udc_ch58x_device_speed,
	.init = udc_ch58x_init,
	.enable = udc_ch58x_enable,
	.disable = udc_ch58x_disable,
	.shutdown = udc_ch58x_shutdown,
	.set_address = udc_ch58x_set_address,
	.host_wakeup = udc_ch58x_host_wakeup,
	.ep_enable = udc_ch58x_ep_enable,
	.ep_disable = udc_ch58x_ep_disable,
	.ep_set_halt = udc_ch58x_ep_set_halt,
	.ep_clear_halt = udc_ch58x_ep_clear_halt,
	.ep_enqueue = udc_ch58x_ep_enqueue,
	.ep_dequeue = udc_ch58x_ep_dequeue,
};

#define DT_DRV_COMPAT wch_ch58x_udc

/*
 * A UDC driver should always be implemented as a multi-instance
 * driver, even if your platform does not require it.
 */
#define UDC_CH58X_DEVICE_DEFINE(n)						\
	static void udc_irq_enable_func##n(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    udc_ch58x_isr,				\
			    DEVICE_DT_INST_GET(n), 0);				\
										\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void udc_irq_disable_func##n(const struct device *dev)		\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
	K_MSGQ_DEFINE(udc_ch58x_msgq_##n, sizeof(struct udc_ch58x_ep_event),\
	      CONFIG_UDC_CH58X_MAX_EP_MSG, sizeof(uint32_t));	\
	static __noinit __aligned(4) uint8_t		\
		udc_ch58x_dma_buf_##n[					\
			DMA_BUF_TOTAL_SZ(DT_INST_PROP(n, num_bidir_endpoints))];	\
	static struct udc_ep_config						\
		ep_cfg_out_##n[DT_INST_PROP(n, num_bidir_endpoints)];		\
	static struct udc_ep_config						\
		ep_cfg_in_##n[DT_INST_PROP(n, num_bidir_endpoints)];		\
										\
	static const struct udc_ch58x_config udc_ch58x_config_##n = {	\
		.reg = DT_INST_REG_ADDR(n),							\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),		\
		.dma_buf = udc_ch58x_dma_buf_##n,						\
		.ep_cfg_in = ep_cfg_in_##n,					\
		.ep_cfg_out = ep_cfg_out_##n,					\
		.udc_irq_enable = udc_irq_enable_func##n,			\
		.udc_irq_disable = udc_irq_disable_func##n,			\
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),	\
	};									\
										\
	static struct udc_ch58x_data udc_priv_##n = {			\
		.dev = DEVICE_DT_INST_GET(n),						\
		.msgq = &udc_ch58x_msgq_##n,						\
		.work = Z_WORK_INITIALIZER(xfer_work_handler),		\
	};									\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_priv_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, udc_ch58x_driver_preinit, NULL,		\
			      &udc_data_##n, &udc_ch58x_config_##n,		\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &udc_ch58x_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_CH58X_DEVICE_DEFINE)
