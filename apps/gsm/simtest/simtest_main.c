/* (C) 2012 by Denis 'GNUtoo' Carikli <GNUtoo@no-log.org>
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

#include <stdio.h>
#include <abb/twl3025.h>


int simtest_main(void)
{
        uint8_t atr[20];
        uint8_t atrLength = 0;

	printf("ENTER %s\n",__func__);

	twl3025_init();

	calypso_sim_init();
        memset(atr,0,sizeof(atr));
        atrLength = calypso_sim_powerup(atr);

	layer1_init();
	tpu_frame_irq_en(1, 1);

	while (1){
		l1a_compl_execute();
		osmo_timers_update();
		sim_handler();
		l1a_l23_handler();
	}
}
