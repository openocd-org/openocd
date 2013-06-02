#!/bin/perl
#***************************************************************************
#*   Copyright (C) 2008 Lou Deluxe                                         *
#*   lou.openocd012@fixit.nospammail.net                                   *
#*                                                                         *
#*   This program is free software; you can redistribute it and/or modify  *
#*   it under the terms of the GNU General Public License as published by  *
#*   the Free Software Foundation; either version 2 of the License, or     *
#*   (at your option) any later version.                                   *
#*                                                                         *
#*   This program is distributed in the hope that it will be useful,       *
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
#*   GNU General Public License for more details.                          *
#*                                                                         *
#*   You should have received a copy of the GNU General Public License     *
#*   along with this program; if not, write to the                         *
#*   Free Software Foundation, Inc.,                                       *
#*   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
#***************************************************************************

# A simple utility to read a list of files (names composed by numeric prescaler arguments) and compose a C source file defining data structures which hold the binary data read from those files.

my @speed_table = ();

print <<HEADER;
/* This file was created automatically by the following script:
 *   $0
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rlink.h"
#include "rlink_st7.h"

HEADER

for $prescaler (sort {$b <=> $a} @ARGV) {
	my(@ary) = (
		byte_array_from_file(${prescaler} . "_init.dtc"),
		byte_array_from_file(${prescaler} . "_call.dtc")
	);

	for $i (@ary) {
		$i = sprintf("%d", $i);
	}
	$bytes = join(', ', @ary);
	$bytes =~ s/(^|\s)(.{70}?\S*)/\2\n/go;	# break up long lines
	$bytes =~ s/\n +/\n/go;
	$bytes =~ s/(^|\n)/\1\t/go;		# format nicely
	printf("static const uint8_t dtc_%d[] = {\n%s\n};\n\n", $prescaler, $bytes);
	push(@speed_table, sprintf("\tdtc_%d, sizeof(dtc_%d), (ST7_FOSC * 2) / (1000 * %d), %d\n", $prescaler, $prescaler, $prescaler, $prescaler));
}

printf("const struct rlink_speed_table rlink_speed_table[] = { {\n%s} };\n\n", join("}, {\n", @speed_table));
printf("const size_t rlink_speed_table_size = ARRAY_SIZE(rlink_speed_table);\n\n");


sub byte_array_from_file {
	my($filename) = @_;

	my(@array, $text, $i) = ();

	open(IN, '<', $filename) || die "$filename: $!";
	undef($/);
	$text = <IN>;
	close(IN);

	for($i = 0; $i < length($text); $i++) {
		push(@array, ord(substr($text, $i, 1)));
	}

	@array;
}
