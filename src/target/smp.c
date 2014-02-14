/***************************************************************************
 *                                                                         *
 * Copyright (C) ST-Ericsson SA 2011                                       *
 * Author: Michel Jaouen <michel.jaouen@stericsson.com> for ST-Ericsson.   *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "server/server.h"

#include "target/target.h"

#include "server/gdb_server.h"
#include "smp.h"
#include "helper/binarybuffer.h"

/*  implementation of new packet in gdb interface for smp feature          */
/*                                                                         */
/*   j : smp  status request                                               */
/*   J : smp  set request                                                  */
/*                                                                         */
/*   jc :read core id displayed by gdb connection                          */
/*   reply XXXXXXXX core id is int32_t , 8 hex digits                      */
/*                                                                         */
/*   Reply ENN error not supported (target not smp)                        */
/*                                                                         */
/*   JcXX  set core id displayed at next gdb continue                      */
/*   maximum 8 bytes described core id int32_t (8 hex digits)              */
/*  (core id -1 , reserved for returning to normal continue mode) */
/*  Reply ENN error not supported(target not smp,core id out of range)     */
/*  Reply OK : for success                                                 */
/*                                                                         */
/*  handling of this packet within gdb can be done by the creation         */
/*  internal variable by mean of function allocate_computed_value          */
/*  set $_core 1 => Jc01 packet is sent                                    */
/*  print $_core => jc packet is sent and result is affected in $          */
/*  Another way to test this packet is the usage of maintenance packet     */
/*  maint packet Jc01                                                      */
/*  maint packet jc                                                        */

/* packet j :smp status request */
int gdb_read_smp_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	int retval = ERROR_OK;
	if (target->smp) {
		if (strncmp(packet, "jc", 2) == 0) {
			const uint32_t len = sizeof(target->gdb_service->core[0]);
			char hex_buffer[len * 2 + 1];
			char buffer[len];
			buf_set_u32(buffer, 0, len * 8, target->gdb_service->core[0]);
			int pkt_len = hexify(hex_buffer, buffer, sizeof(buffer), sizeof(hex_buffer));

			retval = gdb_put_packet(connection, hex_buffer, pkt_len);
		}
	} else
		retval = gdb_put_packet(connection, "E01", 3);
	return retval;
}

/* J :  smp set request */
int gdb_write_smp_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	int coreid = 0;
	int retval = ERROR_OK;

	/* skip command character */
	if (target->smp) {
		if (strncmp(packet, "Jc", 2) == 0) {
			packet += 2;
			coreid = strtoul(packet, &separator, 16);
			target->gdb_service->core[1] = coreid;
			retval = gdb_put_packet(connection, "OK", 2);
		}
	} else
		retval = gdb_put_packet(connection, "E01", 3);

	return retval;
}
