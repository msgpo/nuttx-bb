/* Driver for Analog Baseband Circuit (TWL3025) */

/* (C) 2010 by Harald Welte <laforge@gnumonks.org>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <stdint.h>
#include <stdio.h>

#include <debug.h>
#include <nuttx/spi.h>
#include <calypso/tsp.h>
#include <calypso/tpu.h>
#include <abb/twl3025.h>

/* TWL3025 */
#define REG_PAGE(n)	(n >> 7)
#define REG_ADDR(n)	(n & 0x3f)

#define TWL3025_DEV_IDX		0	/* On the SPI bus */
#define TWL3025_TSP_DEV_IDX	0	/* On the TSP bus */

/* values encountered on a GTA-02 for GSM900 (the same for GSM1800!?) */
const uint16_t twl3025_default_ramp[16] = {
	ABB_RAMP_VAL( 0,  0),
	ABB_RAMP_VAL( 0, 11),
	ABB_RAMP_VAL( 0, 31),
	ABB_RAMP_VAL( 0, 31),
	ABB_RAMP_VAL( 0, 31),
	ABB_RAMP_VAL( 0, 24),
	ABB_RAMP_VAL( 0,  0),
	ABB_RAMP_VAL( 0,  0),
	ABB_RAMP_VAL( 9,  0),
	ABB_RAMP_VAL(18,  0),
	ABB_RAMP_VAL(25,  0),
	ABB_RAMP_VAL(31,  0),
	ABB_RAMP_VAL(30,  0),
	ABB_RAMP_VAL(15,  0),
	ABB_RAMP_VAL( 0,  0),
	ABB_RAMP_VAL( 0,  0),
};

struct twl3025 {
	uint8_t page;
};
static struct twl3025 twl3025_state;
FAR struct spi_dev_s *dev;

/* Switch the register page of the TWL3025 */
static void twl3025_switch_page(uint8_t page)
{
	if (page == 0)
		twl3025_reg_write(PAGEREG, 1 << 0);
	else
		twl3025_reg_write(PAGEREG, 1 << 1);

	twl3025_state.page = page;
}


static void handle_charger(void)
{
	uint16_t status;
	printf("handle_charger();");

	status = twl3025_reg_read(VRPCSTS);
//	printf("\nvrpcsts: 0x%02x", status);

	if (status & 0x40) {
		printf(" inserted\n");
	} else {
		printf(" removed\n");
	}

//	twl3025_dump_madc();
}

static void handle_adc_done(void)
{
	printf("handle_adc_done();");
}

static void twl3025_irq(int irq, FAR void *contex)
{
	uint16_t src;
	printf("twl3025_irq: 0x%02x\n",irq);
	switch (irq){
	case IRQ_EXTERNAL: // charger in/out, pwrbtn, adc done
		src = twl3025_reg_read(ITSTATREG);
		printf("itstatreg 0x%02x\n", src);
		if (src & 0x08)
			handle_charger();
		if (src & 0x20)
			handle_adc_done();
		return OK;
		break;
	case IRQ_EXTERNAL_FIQ: // vcc <2.8V emergency power off
		puts("\nBROWNOUT!1!");
		board_power_off();
		return OK;
		break;
	default:
		return OK;
	}
}

void twl3025_init(void)
{
	printf("ENTER twl3025_init\n");
	dev = up_spiinitialize(TWL3025_DEV_IDX);
	if (!dev)
	{
		printf("twl3025_init: Failed to initialize SPI%d\n",TWL3025_DEV_IDX);
		return;
	}
	SPI_SETBITS(dev,16);
	twl3025_switch_page(0);
	twl3025_clk13m(1);
	twl3025_reg_write(AFCCTLADD, 0x01);	/* AFCCK(1:0) must not be zero! */
	twl3025_unit_enable(TWL3025_UNIT_AFC, 1);

	irq_attach(IRQ_EXTERNAL, (xcpt_t)twl3025_irq);
	up_prioritize_irq(IRQ_EXTERNAL, 0);
	up_enable_irq(IRQ_EXTERNAL);

	irq_attach(IRQ_EXTERNAL_FIQ, (xcpt_t)twl3025_irq);
	up_prioritize_irq(IRQ_EXTERNAL_FIQ, 0);
	up_enable_irq(IRQ_EXTERNAL_FIQ);
}

void twl3025_reg_write(uint8_t reg, uint16_t data)
{
	uint16_t tx;

	printf("tw3025_reg_write(%u,%u)=0x%04x\n", REG_PAGE(reg),
		REG_ADDR(reg), data);

	if (reg != PAGEREG && REG_PAGE(reg) != twl3025_state.page)
		twl3025_switch_page(REG_PAGE(reg));

	tx = ((data & 0x3ff) << 6) | (REG_ADDR(reg) << 1);

	SPI_SNDBLOCK(dev, &tx, 1);
}

void twl3025_tsp_write(uint8_t data)
{
	tsp_write(TWL3025_TSP_DEV_IDX, 7, data);
}

uint16_t twl3025_reg_read(uint8_t reg)
{
	uint16_t tx, rx;

	if (REG_PAGE(reg) != twl3025_state.page)
		twl3025_switch_page(REG_PAGE(reg));

	tx = (REG_ADDR(reg) << 1) | 1;

	/* A read cycle contains two SPI transfers */
	SPI_EXCHANGE(dev, &tx, &rx,1);
	usleep(1000);
	SPI_EXCHANGE(dev, &tx, &rx,1);
	rx >>= 6;

	printf("tw3025_reg_read(%u,%u)=0x%04x\n", REG_PAGE(reg),
		REG_ADDR(reg), rx);

	return rx;
}

static void twl3025_wait_ibic_access(void)
{
	/* Wait 6 * 32kHz clock cycles for first IBIC access (187us + 10% = 210us) */
	usleep(1000);
}

void twl3025_power_off(void)
{
	irqstate_t flags;

	/* turn off all IRQs, since received frames cannot be
	 * handled form here. (otherwise the message allocation
	 * runs out of memory) */
	flags = irqsave();

	/* poll PWON status and power off the phone when the
	 * powerbutton has been released (otherwise it will
	 * poweron immediately again) */
	while (!(twl3025_reg_read(VRPCSTS) & 0x10)) { };
	twl3025_reg_write(VRPCDEV, 0x01);
}

void twl3025_clk13m(int enable)
{
	if (enable) {
		twl3025_reg_write(TOGBR2, TOGBR2_ACTS);
		twl3025_wait_ibic_access();
		/* for whatever reason we need to do this twice */
		twl3025_reg_write(TOGBR2, TOGBR2_ACTS);
		twl3025_wait_ibic_access();
	} else {
		twl3025_reg_write(TOGBR2, TOGBR2_ACTR);
		twl3025_wait_ibic_access();
	}
}

#define	TSP_DELAY	6	/* 13* Tclk6M5 = ~ 3 GSM Qbits + 3 TPU instructions */
#define BDLON_TO_BDLCAL	6
#define BDLCAL_DURATION	66
#define BDLON_TO_BDLENA	7
#define BULON_TO_BULENA	16
#define BULON_TO_BULCAL	17
#define BULCAL_DURATION	143	/* really that long? */

/* bdl_ena - TSP_DELAY - BDLCAL_DURATION - TSP_DELAY - BDLON_TO_BDLCAL - TSP_DELAY */
#define DOWNLINK_DELAY	(3 * TSP_DELAY + BDLCAL_DURATION + BDLON_TO_BDLCAL)

/* Enqueue a series of TSP commands in the TPU to (de)activate the downlink path */
void twl3025_downlink(int on, int16_t at)
{
	int16_t bdl_ena = at - TSP_DELAY - 6;

	if (on) {
		if (bdl_ena < 0)
			printf("BDLENA time negative (%d)\n", bdl_ena);
		/* calibration should be done just before BDLENA */
		tpu_enq_at(bdl_ena - DOWNLINK_DELAY);
		/* bdl_ena - TSP_DELAY - BDLCAL_DURATION - TSP_DELAY - BDLON_TO_BDLCAL - TSP_DELAY */
		twl3025_tsp_write(BDLON);
		/* bdl_ena - TSP_DELAY - BDLCAL_DURATION - TSP_DELAY - BDLON_TO_BDLCAL */
		tpu_enq_wait(BDLON_TO_BDLCAL - TSP_DELAY);
		/* bdl_ena - TSP_DELAY - BDLCAL_DURATION - TSP_DELAY */
		twl3025_tsp_write(BDLON | BDLCAL);
		/* bdl_ena - TSP_DELAY - BDLCAL_DURATION */
		tpu_enq_wait(BDLCAL_DURATION - TSP_DELAY);
		/* bdl_ena - TSP_DELAY */
		twl3025_tsp_write(BDLON);
		//tpu_enq_wait(BDLCAL_TO_BDLENA)	this is only 3.7us == 4 qbits, i.e. less than the TSP_DELAY
		tpu_enq_at(bdl_ena);
		twl3025_tsp_write(BDLON | BDLENA);
	} else {
		tpu_enq_at(bdl_ena);
		twl3025_tsp_write(BDLON);
		//tpu_enq_wait(nBDLENA_TO_nBDLON)	this is only 3.7us == 4 qbits, i.e. less than the TSP_DELAY
		twl3025_tsp_write(0);
	}
}

/* bdl_ena - 35 - TSP_DELAY - BULCAL_DURATION - TSP_DELAY - BULON_TO_BULCAL - TSP_DELAY */
#define UPLINK_DELAY (3 * TSP_DELAY + BULCAL_DURATION + BULON_TO_BULCAL + 35)

void twl3025_uplink(int on, int16_t at)
{
	int16_t bul_ena = at - TSP_DELAY - 6;

	if (bul_ena < 0)
		printf("BULENA time negative (%d)\n", bul_ena);
	if (on) {
		/* calibration should  be done just before BULENA */
		tpu_enq_at(bul_ena - UPLINK_DELAY);
		/* bdl_ena - 35 - TSP_DELAY - BULCAL_DURATION - TSP_DELAY - BULON_TO_BULCAL - TSP_DELAY */
		twl3025_tsp_write(BULON);
		/* bdl_ena - 35 - TSP_DELAY - BULCAL_DURATION - TSP_DELAY - BULON_TO_BULCAL */
		tpu_enq_wait(BULON_TO_BULCAL - TSP_DELAY);
		/* bdl_ena - 35 - TSP_DELAY - BULCAL_DURATION - TSP_DELAY */
		twl3025_tsp_write(BULON | BULCAL);
		/* bdl_ena - 35 - TSP_DELAY - BULCAL_DURATION */
		tpu_enq_wait(BULCAL_DURATION - TSP_DELAY);
		/* bdl_ena - 35 - TSP_DELAY */
		twl3025_tsp_write(BULON);
		/* bdl_ena - 35 */
		tpu_enq_wait(35);	/* minimum time required to bring the ramp up (really needed?) */
		tpu_enq_at(bul_ena);
		twl3025_tsp_write(BULON | BULENA);
	} else {
		tpu_enq_at(bul_ena);
		twl3025_tsp_write(BULON);
		tpu_enq_wait(35);	/* minimum time required to bring the ramp down (needed!) */
		twl3025_tsp_write(0);
	}
}

void twl3025_afc_set(int16_t val)
{
	printf("twl3025_afc_set(%d)\n", val);

	if (val > 4095)
		val = 4095;
	else if (val <= -4096)
		val = -4096;

	/* FIXME: we currently write from the USP rather than BSP */
	twl3025_reg_write(AUXAFC2, val >> 10);
	twl3025_reg_write(AUXAFC1, val & 0x3ff);
}

int16_t twl3025_afc_get(void)
{
	int16_t val;

	val = (twl3025_reg_read(AUXAFC2) & 0x7);
	val = val << 10;
	val = val | (twl3025_reg_read(AUXAFC1) & 0x3ff);

	if (val > 4095)
		val = -(8192 - val);
	return val;
}

void twl3025_unit_enable(enum twl3025_unit unit, int on)
{
	uint16_t togbr1 = 0;

	switch (unit) {
	case TWL3025_UNIT_AFC:
		if (on)
			togbr1 = (1 << 7);
		else
			togbr1 = (1 << 6);
		break;
	case TWL3025_UNIT_MAD:
		if (on)
			togbr1 = (1 << 9);
		else
			togbr1 = (1 << 8);
		break;
	case TWL3025_UNIT_ADA:
		if (on)
			togbr1 = (1 << 5);
		else
			togbr1 = (1 << 4);
	case TWL3025_UNIT_VDL:
		if (on)
			togbr1 = (1 << 3);
		else
			togbr1 = (1 << 2);
		break;
	case TWL3025_UNIT_VUL:
		if (on)
			togbr1 = (1 << 1);
		else
			togbr1 = (1 << 0);
		break;
	}
	twl3025_reg_write(TOGBR1, togbr1);
}

uint8_t twl3025_afcout_get(void)
{
	return twl3025_reg_read(AFCOUT) & 0xff;
}

void twl3025_afcout_set(uint8_t val)
{
	twl3025_reg_write(AFCCTLADD, 0x05);
	twl3025_reg_write(AFCOUT, val);
}
