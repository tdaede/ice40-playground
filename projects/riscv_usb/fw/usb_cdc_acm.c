/*
 * usb_cdc_acm.c
 *
 * Copyright (C) 2019 Sylvain Munaut
 * All rights reserved.
 *
 * LGPL v3+, see LICENSE.lgpl3
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "console.h"
#include "usb.h"
#include "usb_hw.h"
#include "usb_priv.h"


#define USB_RT_CDC_SEND_ENCAPSULATED_COMMAND	((0x00 << 8) | 0x21)
#define USB_RT_CDC_GET_ENCAPSULATED_RESPONSE	((0x01 << 8) | 0xa1)
#define USB_RT_CDC_SET_COMM_FEATURE		((0x02 << 8) | 0x21)
#define USB_RT_CDC_GET_COMM_FEATURE		((0x03 << 8) | 0xa1)
#define USB_RT_CDC_CLEAR_COMM_FEATURE		((0x04 << 8) | 0x21)
#define USB_RT_CDC_SET_LINE_CODING		((0x20 << 8) | 0x21)
#define USB_RT_CDC_GET_LINE_CODING		((0x21 << 8) | 0xa1)
#define USB_RT_CDC_SET_CONTROL_LINE_STATE	((0x22 << 8) | 0x21)
#define USB_RT_CDC_SEND_BREAK			((0x23 << 8) | 0x21)


// Buffer helpers
// --------------

#define UB_LEN	256

struct uart_buf
{
	char buf[UB_LEN];
	int  rd;
	int  wr;
};

static struct {
	struct uart_buf rx;
	struct uart_buf tx;
} g_uart;

static inline bool
ub_empty(const struct uart_buf *b)
{
	return b->rd == b->wr;
}

static inline bool
ub_full(const struct uart_buf *b)
{
	return b->rd == ((b->wr + 1) & (UB_LEN-1));
}

static inline int
ub_free(const struct uart_buf *b)
{
	return (b->rd - b->wr - 1) & (UB_LEN-1);
}

static inline int
ub_used(const struct uart_buf *b)
{
	return (b->wr - b->rd) & (UB_LEN-1);
}

static inline char
ub_pop(struct uart_buf *b)
{
	char c = b->buf[b->rd];
	b->rd = (b->rd + 1) & (UB_LEN-1);
	return c;
}

static inline void
ub_push(struct uart_buf *b, char c)
{
	b->buf[b->wr] = c;
	b->wr = (b->wr + 1) & (UB_LEN-1);
}

static inline void
ub_read(struct uart_buf *b, char *buf, int len)
{
	if ((b->rd + len) > UB_LEN) {
		int m = UB_LEN - b->rd;
		memcpy(&buf[0], &b->buf[b->rd], m);
		memcpy(&buf[m], &b->buf[0],     len-m);
	} else {
		memcpy(buf, &b->buf[b->rd], len);
	}

	b->rd = (b->rd + len) & (UB_LEN-1);
}

static inline void
ub_write(struct uart_buf *b, char *buf, int len)
{
	if ((b->wr + len) > UB_LEN) {
		int m = UB_LEN - b->rd;
		memcpy(&b->buf[b->wr], &buf[0], m);
		memcpy(&b->buf[0],     &buf[m], len-m);
	} else {
		memcpy(&b->buf[b->wr], buf, len);
	}

	b->wr = (b->wr + len) & (UB_LEN-1);
}


// Console implementation
// ----------------------

void console_init(void)
{
	memset(&g_uart, 0x00, sizeof(g_uart));
}

char getchar(void)
{
	while (ub_empty(&g_uart.rx))
		usb_poll();

	return ub_pop(&g_uart.rx);
}

int getchar_nowait(void)
{
	return ub_empty(&g_uart.rx) ? -1 : ub_pop(&g_uart.rx);
}

void putchar(char c)
{
	while (ub_full(&g_uart.tx))
		usb_poll();

	return ub_push(&g_uart.tx, c);
}

void puts(const char *p)
{
        char c;
        while ((c = *(p++)) != 0x00) {
                if (c == '\n')
			putchar('\r');
		putchar(c);
        }
}



// CDC ACM code
// ------------

static void
_cdc_acm_sof(void)
{
	uint32_t bds_out;
	uint32_t bds_in;
	char buf[64] __attribute__((aligned(4)));

	if (usb_get_state() != USB_DS_CONFIGURED)
		return;

	bds_out = usb_ep_regs[5].out.bd[0].csr;
	bds_in  = usb_ep_regs[5].in.bd[0].csr;

	/* Read data */
	if ((bds_out & USB_BD_STATE_MSK) == USB_BD_STATE_DONE_ERR) {
		USB_LOG_ERR("[1] Err %08x\n", bds_out);
		usb_ep_regs[5].out.bd[0].csr = USB_BD_STATE_RDY_DATA | USB_BD_LEN(64);
	} else if ((bds_out & USB_BD_STATE_MSK) == USB_BD_STATE_DONE_OK) {
		int len = (bds_out & USB_BD_LEN_MSK) - 2;

		if (ub_free(&g_uart.rx) >= len) {
			usb_data_read(buf, 128, len);
			ub_write(&g_uart.rx, buf, len);
			usb_ep_regs[5].out.bd[0].csr = USB_BD_STATE_RDY_DATA | USB_BD_LEN(64);
		}
	}

	/* Write data */
	if ((bds_in & USB_BD_STATE_MSK) != USB_BD_STATE_RDY_DATA) {
		int len = ub_used(&g_uart.tx);
		if (len > 0) {
			if (len > 64)
				len = 64;
			ub_read(&g_uart.tx, buf, len);
			usb_data_write(128, buf, len);
			usb_ep_regs[5].in.bd[0].csr = USB_BD_STATE_RDY_DATA | USB_BD_LEN(len);
		}
	}
}


static enum usb_fnd_resp
_cdc_acm_ctrl_req(struct usb_ctrl_req *req, struct usb_xfer *xfer)
{
	const struct usb_intf_desc *intf;

	/* Check this is a class request for a CDC/ACM data interface we handle */
	if (USB_REQ_TYPE_RCPT(req) != (USB_REQ_TYPE_CLASS | USB_REQ_RCPT_INTF))
		return USB_FND_CONTINUE;

	intf = usb_desc_find_intf(NULL, req->wIndex, 0, NULL);
	if (!intf)
		return USB_FND_ERROR;

        if ((intf->bInterfaceClass != 0x0a) ||
            (intf->bInterfaceSubClass != 0x00) ||
            (intf->bInterfaceProtocol != 0x00))
                return USB_FND_CONTINUE;

	/* Handle request */
	switch (req->wRequestAndType)
	{
	case USB_RT_CDC_SET_LINE_CODING:
	case USB_RT_CDC_GET_LINE_CODING:
	case USB_RT_CDC_SET_CONTROL_LINE_STATE:
		break;

	/* Unsupported */
	case USB_RT_CDC_SEND_ENCAPSULATED_COMMAND:
	case USB_RT_CDC_GET_ENCAPSULATED_RESPONSE:
	case USB_RT_CDC_SET_COMM_FEATURE:
	case USB_RT_CDC_GET_COMM_FEATURE:
	case USB_RT_CDC_CLEAR_COMM_FEATURE:
	case USB_RT_CDC_SEND_BREAK:
	default:
		return USB_FND_ERROR;
	}

	return USB_FND_SUCCESS;
}


static void
_cdc_acm_state_chg(enum usb_dev_state state)
{
	if (state == USB_DS_CONFIGURED) {
		/* EP4 IN: Interrupt, single buffer, and prepare one buffer */
		usb_ep_regs[4].in.status = USB_EP_TYPE_INT;
		usb_ep_regs[4].in.bd[0].ptr = 64;
		usb_ep_regs[4].in.bd[0].csr = 0;

		/* EP5 IN: Bulk, single buffer, and prepare one buffer */
		usb_ep_regs[5].in.status = USB_EP_TYPE_BULK;
		usb_ep_regs[5].in.bd[0].ptr = 128;
		usb_ep_regs[5].in.bd[0].csr = 0;

		/* EP5 OUT: Bulk, single buffer, and prepare one one buffer */
		usb_ep_regs[5].out.status = USB_EP_TYPE_BULK;
		usb_ep_regs[5].out.bd[0].ptr = 128;
		usb_ep_regs[5].out.bd[0].csr = USB_BD_STATE_RDY_DATA | USB_BD_LEN(64);
	}
}


static struct usb_fn_drv _cdc_acm_drv = {
	.sof		= _cdc_acm_sof,
	.ctrl_req	= _cdc_acm_ctrl_req,
	.state_chg	= _cdc_acm_state_chg,
//	.set_intf	= _cdc_acm_set_intf,
//	.get_intf	= _cdc_acm_get_intf,
};


void
usb_cdc_acm_init(void)
{
	usb_register_function_driver(&_cdc_acm_drv);
}
