/****************************************************************************
 * drivers/sercomm/console.c
 * Driver for NuttX Console
 *
 * (C) 2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 Stefan Richter <ichgeh@l--putt.de>
 *
 * This source code is derivated from Osmocom-BB project and was
 * relicensed as BSD with permission from original authors.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/serial/serial.h>

#include <errno.h>
#include <debug.h>
#include <string.h>

#include "uart.h"
#include <nuttx/sercomm/sercomm.h>
#include <osmocom/core/msgb.h>

/* stubs to make serial driver happy */
void sercomm_recvchars(void *a) { }
void sercomm_xmitchars(void *a) { }

/* Stubs to make memory allocator happy */
void cons_puts(void *foo){}
void delay_ms(int ms){}
void osmo_panic(const char *fmt, ...) {}

/************************************************************************************
 * Fileops Prototypes and Structures
 ************************************************************************************/

typedef FAR struct file		file_t;

static ssize_t sc_console_read(file_t *filep, FAR char *buffer, size_t buflen);
static ssize_t sc_console_write(file_t *filep, FAR const char *buffer, size_t buflen);
static int     sc_console_ioctl(file_t *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int     sc_console_poll(file_t *filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations g_sercom_console_ops =
{
	0,			/* open, always opened */
	0,			/* close, stays open */
	sc_console_read,	/* read */
	sc_console_write,	/* write */
	0,			/* seek, not supported */
	sc_console_ioctl,	/* ioctl */
#ifndef CONFIG_DISABLE_POLL
	sc_console_poll		/* poll */
#endif
};
#define SERCOMM_CONS_ALLOC	256

/****************************************************************************
 * Helper functions
 ****************************************************************************/
static FAR uart_dev_t *readdev = NULL;

void *tall_msgb_ctx;

/* ./comm/sercomm.c */
enum rx_state {
        RX_ST_WAIT_START,
        RX_ST_ADDR,
        RX_ST_CTRL,
        RX_ST_DATA,
        RX_ST_ESCAPE,
};

/* ./target/firmware/comm/sercomm.c */
static struct {
        int initialized;

        /* transmit side */
        struct {
                struct llist_head dlci_queues[_SC_DLCI_MAX];
                struct msgb *msg;
                enum rx_state state;
                uint8_t *next_char;
        } tx;

        /* receive side */
        struct {
                dlci_cb_t dlci_handler[_SC_DLCI_MAX];
                struct msgb *msg;
                enum rx_state state;
                uint8_t dlci;
                uint8_t ctrl;
        } rx;

} sercomm;
static struct msgb *recvmsg = NULL;
#if 0
/* ./target/firmware/comm/sercomm.c */
/* user interface for transmitting messages for a given DLCI */
void sercomm_sendmsg(uint8_t dlci, struct msgb *msg)
{
        irqstate_t flags;
        uint8_t *hdr;

        /* prepend address + control octet */
        hdr = msgb_push(msg, 2);
        hdr[0] = dlci;
        hdr[1] = HDLC_C_UI;

        /* This functiion can be called from any context: FIQ, IRQ
         * and supervisor context.  Proper locking is important! */
        flags = irqsave();
        msgb_enqueue(&sercomm.tx.dlci_queues[dlci], msg);
        irqrestore(flags);

        /* tell UART that we have something to send */
        uart_irq_enable(SERCOMM_UART_NR, UART_IRQ_TX_EMPTY, 1);
}
#endif

static void recv_cb(uint8_t dlci, struct msgb *msg)
{
	sem_post(&readdev->recvsem);
	recvmsg = msg;
}

/****************************************************************************
 * Fileops
 ****************************************************************************/

/* XXX: recvmsg is overwritten when multiple msg arrive! */
static ssize_t sc_console_read(file_t *filep, FAR char *buffer, size_t buflen)
{
	size_t len;
	struct msgb *tmp;

	/* Wait until data is received */
	while(recvmsg == NULL) {
		sem_wait(&readdev->recvsem);
	}

	len = recvmsg->len > buflen ? buflen : recvmsg->len;
	memcpy(buffer, msgb_pull(recvmsg, len) - len, len);

	if(recvmsg->len == 0) {
		/* prevent inconsistent msg by first invalidating it, then free it */
		tmp = recvmsg;
		recvmsg = NULL;
		msgb_free(tmp);
	}

	return len;
}

#define raw_putd(x)
#define raw_puts(x)

/* ./target/firmware/comm/sercomm_cons.c */
static struct {
        struct msgb *cur_msg;
} scons;

/* ./target/firmware/comm/sercomm_cons.c */
int sercomm_puts(const char *s)
{
	irqstate_t flags;
	const int len = strlen(s);
	unsigned int bytes_left = len;

	if (!sercomm_initialized()) {
		raw_putd("sercomm not initialized: ");
		raw_puts(s);
		return len - 1;
	}

	/* This function is called from any context: Supervisor, IRQ, FIQ, ...
	 * as such, we need to ensure re-entrant calls are either supported or
	 * avoided. */
	flags = irqsave();
	//local_fiq_disable(); /* TODO check it at runtime */

	while (bytes_left > 0) {
		unsigned int write_num, space_left, flush;
		uint8_t *data;

		if (!scons.cur_msg)
			scons.cur_msg = sercomm_alloc_msgb(SERCOMM_CONS_ALLOC);

		if (!scons.cur_msg) {
			raw_putd("cannot allocate sercomm msgb: ");
			raw_puts(s);
			return -ENOMEM;
		}

		/* space left in the current msgb */
		space_left = msgb_tailroom(scons.cur_msg);

		if (space_left <= bytes_left) {
			write_num = space_left;
			/* flush buffer when it is full */
			flush = 1;
		} else {
			write_num = bytes_left;
			flush = 0;
		}

		/* obtain pointer where to copy the data */
		data = msgb_put(scons.cur_msg, write_num);

		/* copy data while looking for \n line termination */
		{
			unsigned int i;
			for (i = 0; i < write_num; i++) {
				/* flush buffer at end of line, but skip
				 * flushing if we have a backlog in order to
				 * increase efficiency of msgb filling */
				if (*s == '\n' &&
				    sercomm_tx_queue_depth(SC_DLCI_CONSOLE) < 4)
					flush = 1;
				*data++ = *s++;
			}
		}
		bytes_left -= write_num;

		if (flush) {
			sercomm_sendmsg(SC_DLCI_CONSOLE, scons.cur_msg);
			/* reset scons.cur_msg pointer to ensure we allocate
			 * a new one next round */
			scons.cur_msg = NULL;
		}
	}

	irqrestore(flags);

	return len - 1;
}
/* XXX: redirect to old Osmocom-BB comm/sercomm_cons.c -> 2 buffers */
static ssize_t sc_console_write(file_t *filep, FAR const char *buffer, size_t buflen)
{
	int i, cnt;
	char dstbuf[32];

	if (buflen >= 31)
		cnt = 31;
	else
		cnt = buflen;

        memcpy(dstbuf, buffer, cnt);
        dstbuf[cnt] = '\0';

	/* print part of our buffer */
	sercomm_puts(dstbuf);

	/* wait a little bit to get data transfered */
	up_mdelay(1);

	return cnt;
}

/* Forward ioctl to uart driver */
static int sc_console_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	FAR struct inode *inode = filep->f_inode;
	FAR uart_dev_t   *dev   = inode->i_private;

	return dev->ops->ioctl(filep, cmd, arg);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Use sercomm on uart driver, register console driver */
int sercomm_register(FAR const char *path, FAR uart_dev_t *dev)
{
	/* XXX: initialize MODEMUART to be used for sercomm*/
	uart_init(SERCOMM_UART_NR, 1);
	uart_baudrate(SERCOMM_UART_NR, UART_115200);
	readdev = dev;
	sercomm_register_rx_cb(SC_DLCI_LOADER, &recv_cb);

	sem_init(&dev->xmit.sem, 0, 1);
	sem_init(&dev->recv.sem, 0, 1);
	sem_init(&dev->closesem, 0, 1);
	sem_init(&dev->xmitsem,  0, 0);
	sem_init(&dev->recvsem,  0, 0);
#ifndef CONFIG_DISABLE_POLL
	sem_init(&dev->pollsem,  0, 1);
#endif

	dbg("Registering %s\n", path);
	return register_driver(path, &g_sercom_console_ops, 0666, NULL);
}
