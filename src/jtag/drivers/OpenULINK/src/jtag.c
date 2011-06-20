/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
 *                                                                         *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "jtag.h"

#include "io.h"
#include "msgtypes.h"
#include "common.h"

#include <stdbool.h>

/** Delay value for SCAN operations with less than maximum TCK frequency */
u8 delay_scan = 0;

/** Delay value for CLOCK_TCK operations */
u8 delay_tck = 0;

/** Delay value for CLOCK_TMS operations with less than maximum frequency */
u8 delay_tms = 0;

/**
 * Perform JTAG SCAN-IN operation at maximum TCK frequency.
 *
 * Dummy data is shifted into the JTAG chain via TDI, TDO data is sampled and
 * stored in the EP2 IN buffer.
 *
 * @param out_offset offset in OUT2BUF where payload data starts
 */
void jtag_scan_in(u8 out_offset, u8 in_offset)
{
  u8 scan_size_bytes, bits_last_byte;
  u8 tms_count_start, tms_count_end;
  u8 tms_sequence_start, tms_sequence_end;
  u8 tdo_data, i, j;

  u8 outb_buffer;

  /* Get parameters from OUT2BUF */
  scan_size_bytes = OUT2BUF[out_offset];
  bits_last_byte = OUT2BUF[out_offset + 1];
  tms_count_start = (OUT2BUF[out_offset + 2] >> 4) & 0x0F;
  tms_count_end = OUT2BUF[out_offset + 2] & 0x0F;
  tms_sequence_start = OUT2BUF[out_offset + 3];
  tms_sequence_end = OUT2BUF[out_offset + 4];

  if (tms_count_start > 0) {
    jtag_clock_tms(tms_count_start, tms_sequence_start);
  }

  outb_buffer = OUTB & ~(PIN_TDI | PIN_TCK | PIN_TMS);

  /* Shift all bytes except the last byte */
  for (i = 0; i < scan_size_bytes - 1; i++) {
    tdo_data = 0;

    for (j = 0; j < 8; j++) {
      OUTB = outb_buffer; /* TCK changes here */
      OUTB = (outb_buffer | PIN_TCK);
      tdo_data = tdo_data >> 1;

      if (GET_TDO()) {
        tdo_data |= 0x80;
      }
    }

    /* Copy TDO data to IN2BUF */
    IN2BUF[i + in_offset] = tdo_data;
  }

  tdo_data = 0;

  /* Shift the last byte */
  for (j = 0; j < bits_last_byte; j++) {
    /* Assert TMS signal if requested and this is the last bit */
    if ((j == bits_last_byte - 1) && (tms_count_end > 0)) {
      outb_buffer |= PIN_TMS;
      tms_count_end--;
      tms_sequence_end = tms_sequence_end >> 1;
    }

    OUTB = outb_buffer; /* TCK change here */
    OUTB = (outb_buffer | PIN_TCK);
    tdo_data = tdo_data >> 1;

    if (GET_TDO()) {
      tdo_data |= 0x80;
    }
  }
  tdo_data = tdo_data >> (8 - bits_last_byte);

  /* Copy TDO data to IN2BUF */
  IN2BUF[i + in_offset] = tdo_data;

  /* Move to correct end state */
  if (tms_count_end > 0) {
    jtag_clock_tms(tms_count_end, tms_sequence_end);
  }
}

/**
 * Perform JTAG SCAN-OUT operation at maximum TCK frequency.
 *
 * Data stored in EP2 OUT buffer is shifted into the JTAG chain via TDI, TDO
 * data is not sampled.
 * The TAP-FSM state is alyways left in the PAUSE-DR/PAUSE-IR state.
 *
 * @param out_offset offset in OUT2BUF where payload data starts
 */
void jtag_scan_out(u8 out_offset)
{
  u8 scan_size_bytes, bits_last_byte;
  u8 tms_count_start, tms_count_end;
  u8 tms_sequence_start, tms_sequence_end;
  u8 tdi_data, i, j;

  u8 outb_buffer;

  /* Get parameters from OUT2BUF */
  scan_size_bytes = OUT2BUF[out_offset];
  bits_last_byte = OUT2BUF[out_offset + 1];
  tms_count_start = (OUT2BUF[out_offset + 2] >> 4) & 0x0F;
  tms_count_end = OUT2BUF[out_offset + 2] & 0x0F;
  tms_sequence_start = OUT2BUF[out_offset + 3];
  tms_sequence_end = OUT2BUF[out_offset + 4];
  
  if (tms_count_start > 0) {
    jtag_clock_tms(tms_count_start, tms_sequence_start);
  }

  outb_buffer = OUTB & ~(PIN_TCK | PIN_TMS);

  /* Shift all bytes except the last byte */
  for (i = 0; i < scan_size_bytes - 1; i++) {
    tdi_data = OUT2BUF[i + out_offset + 5];

    for (j = 0; j < 8; j++) {
      if (tdi_data & 0x01) {
        outb_buffer |= PIN_TDI;
      }
      else {
        outb_buffer &= ~PIN_TDI;
      }

      OUTB = outb_buffer; /* TDI and TCK change here */
      tdi_data = tdi_data >> 1;
      OUTB = (outb_buffer | PIN_TCK);
    }
  }

  tdi_data = OUT2BUF[i + out_offset + 5];

  /* Shift the last byte */
  for (j = 0; j < bits_last_byte; j++) {
    if (tdi_data & 0x01) {
      outb_buffer |= PIN_TDI;
    }
    else {
      outb_buffer &= ~PIN_TDI;
    }

    /* Assert TMS signal if requested and this is the last bit */
    if ((j == bits_last_byte - 1) && (tms_count_end > 0)) {
      outb_buffer |= PIN_TMS;
      tms_count_end--;
      tms_sequence_end = tms_sequence_end >> 1;
    }

    OUTB = outb_buffer; /* TDI and TCK change here */
    tdi_data = tdi_data >> 1;
    OUTB = (outb_buffer | PIN_TCK);
  }

  /* Move to correct end state */
  if (tms_count_end > 0) {
    jtag_clock_tms(tms_count_end, tms_sequence_end);
  }
}

/**
 * Perform bidirectional JTAG SCAN operation at maximum TCK frequency.
 *
 * Data stored in EP2 OUT buffer is shifted into the JTAG chain via TDI, TDO
 * data is sampled and stored in the EP2 IN buffer.
 * The TAP-FSM state is alyways left in the PAUSE-DR/PAUSE-IR state.
 *
 * @param out_offset offset in OUT2BUF where payload data starts
 */
void jtag_scan_io(u8 out_offset, u8 in_offset)
{
  u8 scan_size_bytes, bits_last_byte;
  u8 tms_count_start, tms_count_end;
  u8 tms_sequence_start, tms_sequence_end;
  u8 tdi_data, tdo_data, i, j;

  u8 outb_buffer;

  /* Get parameters from OUT2BUF */
  scan_size_bytes = OUT2BUF[out_offset];
  bits_last_byte = OUT2BUF[out_offset + 1];
  tms_count_start = (OUT2BUF[out_offset + 2] >> 4) & 0x0F;
  tms_count_end = OUT2BUF[out_offset + 2] & 0x0F;
  tms_sequence_start = OUT2BUF[out_offset + 3];
  tms_sequence_end = OUT2BUF[out_offset + 4];
  
  if (tms_count_start > 0) {
    jtag_clock_tms(tms_count_start, tms_sequence_start);
  }

  outb_buffer = OUTB & ~(PIN_TCK | PIN_TMS);

  /* Shift all bytes except the last byte */
  for (i = 0; i < scan_size_bytes - 1; i++) {
    tdi_data = OUT2BUF[i + out_offset + 5];
    tdo_data = 0;

    for (j = 0; j < 8; j++) {
      if (tdi_data & 0x01) {
        outb_buffer |= PIN_TDI;
      }
      else {
        outb_buffer &= ~PIN_TDI;
      }

      OUTB = outb_buffer; /* TDI and TCK change here */
      tdi_data = tdi_data >> 1;
      OUTB = (outb_buffer | PIN_TCK);
      tdo_data = tdo_data >> 1;

      if (GET_TDO()) {
        tdo_data |= 0x80;
      }
    }

    /* Copy TDO data to IN2BUF */
    IN2BUF[i + in_offset] = tdo_data;
  }

  tdi_data = OUT2BUF[i + out_offset + 5];
  tdo_data = 0;

  /* Shift the last byte */
  for (j = 0; j < bits_last_byte; j++) {
    if (tdi_data & 0x01) {
      outb_buffer |= PIN_TDI;
    }
    else {
      outb_buffer &= ~PIN_TDI;
    }

    /* Assert TMS signal if requested and this is the last bit */
    if ((j == bits_last_byte - 1) && (tms_count_end > 0)) {
      outb_buffer |= PIN_TMS;
      tms_count_end--;
      tms_sequence_end = tms_sequence_end >> 1;
    }

    OUTB = outb_buffer; /* TDI and TCK change here */
    tdi_data = tdi_data >> 1;
    OUTB = (outb_buffer | PIN_TCK);
    tdo_data = tdo_data >> 1;

    if (GET_TDO()) {
      tdo_data |= 0x80;
    }
  }
  tdo_data = tdo_data >> (8 - bits_last_byte);

  /* Copy TDO data to IN2BUF */
  IN2BUF[i + in_offset] = tdo_data;
  
  /* Move to correct end state */
  if (tms_count_end > 0) {
    jtag_clock_tms(tms_count_end, tms_sequence_end);
  }
}

/**
 * Generate TCK clock cycles.
 *
 * @param count number of TCK clock cyclces to generate.
 */
void jtag_clock_tck(u16 count)
{
  u16 i;
  u8 j;

  for ( i = 0; i < count; i++ ) {
    SET_TCK_LOW();
    for(j = 0; j < delay_tck; j++);

    SET_TCK_HIGH();
    for(j = 0; j < delay_tck; j++);
  }
}

/**
 * Perform TAP-FSM state transitions at maximum TCK frequency.
 *
 * @param count the number of state transitions to perform.
 * @param sequence the TMS pin levels for each state transition, starting with
 *  the least-significant bit.
 */
void jtag_clock_tms(u8 count, u8 sequence)
{
  volatile u8 outb_buffer;
  u8 i;

  outb_buffer = OUTB & ~(PIN_TCK);

  for ( i = 0; i < count; i++ ) {
    /* Set TMS pin according to sequence parameter */
    if ( sequence & 0x1 ) {
      outb_buffer |= PIN_TMS;
    }
    else {
      outb_buffer &= ~PIN_TMS;
    }

    OUTB = outb_buffer;
    sequence = sequence >> 1;
    OUTB = outb_buffer | PIN_TCK;
  }
}

/**
 * Perform TAP-FSM state transitions at less than maximum TCK frequency.
 *
 * @param count the number of state transitions to perform.
 * @param sequence the TMS pin levels for each state transition, starting with
 *  the least-significant bit.
 */
void jtag_slow_clock_tms(u8 count, u8 sequence)
{

}

/**
 * Get current JTAG signal states.
 *
 * @return a 16-bit integer where the most-significant byte contains the state
 *  of the JTAG input signals and the least-significant byte cotains the state
 *  of the JTAG output signals.
 */
u16 jtag_get_signals(void)
{
  u8 input_signal_state, output_signal_state;

  input_signal_state = 0;
  output_signal_state = 0;

  /* Get states of input pins */
  if (GET_TDO()) {
    input_signal_state |= SIGNAL_TDO;
  }
  if (GET_BRKOUT()) {
    input_signal_state |= SIGNAL_BRKOUT;
  }
  if (GET_TRAP()) {
    input_signal_state |= SIGNAL_TRAP;
  }
  if (GET_RTCK()) {
    /* Using RTCK this way would be extremely slow,
     * implemented only for the sake of completeness */
    input_signal_state |= SIGNAL_RTCK;
  }

  /* Get states of output pins */
  output_signal_state = PINSB & MASK_PORTB_DIRECTION_OUT;

  return ((u16)input_signal_state << 8) | ((u16)output_signal_state);
}

/**
 * Set state of JTAG output signals.
 *
 * @param low signals which should be de-asserted.
 * @param high signals which should be asserted.
 */
void jtag_set_signals(u8 low, u8 high)
{
  OUTB &= ~(low & MASK_PORTB_DIRECTION_OUT);
  OUTB |= (high & MASK_PORTB_DIRECTION_OUT);
}

/**
 * Configure TCK delay parameters.
 *
 * @param scan number of delay cycles in shift operations.
 * @param tck number of delay cycles in clock_tck operations.
 * @param tms number of delay cycles in clock_tms operations.
 */
void jtag_configure_tck_delay(u8 scan, u8 tck, u8 tms)
{
  delay_scan = scan;
  delay_tck = tck;
  delay_tms = tms;
}
