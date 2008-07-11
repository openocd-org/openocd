#!/bin/bash
# restart using a Tcl shell \
	exec sh -c 'for tclshell in tclsh tclsh83 cygtclsh80 ; do \
			( echo | $tclshell ) 2> /dev/null && exec $tclshell "`( cygpath -w \"$0\" ) 2> /dev/null || echo $0`" "$@" ; \
		done ; \
		echo "file2c.tcl: cannot find Tcl shell" ; exit 1' "$0" "$@"

#===============================================================================
#
#    file2c.tcl
#
#    Convert a file into a header that can be #included from C.
#
#===============================================================================
#####ECOSGPLCOPYRIGHTBEGIN####
## -------------------------------------------
## This file is part of eCos, the Embedded Configurable Operating System.
## Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
##
## eCos is free software; you can redistribute it and/or modify it under
## the terms of the GNU General Public License as published by the Free
## Software Foundation; either version 2 or (at your option) any later version.
##
## eCos is distributed in the hope that it will be useful, but WITHOUT ANY
## WARRANTY; without even the implied warranty of MERCHANTABILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License along
## with eCos; if not, write to the Free Software Foundation, Inc.,
## 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
##
## As a special exception, if other files instantiate templates or use macros
## or inline functions from this file, or you compile this file and link it
## with other works to produce a work based on this file, this file does not
## by itself cause the resulting work to be covered by the GNU General Public
## License. However the source code for this file must still be made available
## in accordance with section (3) of the GNU General Public License.
##
## This exception does not invalidate any other reasons why a work based on
## this file might be covered by the GNU General Public License.
##
## Alternative licenses for eCos may be arranged by contacting Red Hat, Inc.
## at http://sources.redhat.com/ecos/ecos-license/
## -------------------------------------------
#####ECOSGPLCOPYRIGHTEND####
#===============================================================================
######DESCRIPTIONBEGIN####
#
# Author(s):	jlarmour,bartv
# Contact(s):	
# Date:		2001-07-20
# Purpose:      
# Description:
# Usage:        file2c.tcl <file to encode> <output C header file>
#
#####DESCRIPTIONEND####
#===============================================================================



if { $argc != 2 } {
	puts "Usage: file2c.tcl <file to encode> <output C file>"
	exit 1
}
set infile [lindex $argv 0]
set outfile [lindex $argv 1]
set label [string range $outfile [expr 1+[string last / $outfile]] [expr [string last . $outfile]-1]]

set status [ catch {
	set infilefd [open $infile "r"]
	fconfigure $infilefd -translation binary
	set data [read $infilefd]
	close $infilefd
} message]

if { $status != 0 } {
	error "Unable to read file $infile: $message"
}

set result ""

set status [ catch {
	set outfilefd [ open $outfile "w" ]
} message ]

if { $status != 0 } {
	error "Unable to create file $outfile: $message"
}

append result "/* This is a generated file. Do not edit. */\n\n"
append result "const unsigned char filedata_$label\[\] = {\n"

set datalength [ string length $data ]

set aligned_datalength [expr $datalength - ($datalength % 8)]

for { set i 0 } {$i < $aligned_datalength} {incr i 8} {
	binary scan $data "@[set i]H16" var0
	append result [format "    0x%2s, 0x%2s, 0x%2s, 0x%2s, 0x%2s, 0x%2s, 0x%2s, 0x%2s,\n" \
			[string range $var0  0  1] \
			[string range $var0  2  3] \
			[string range $var0  4  5] \
			[string range $var0  6  7] \
			[string range $var0  8  9] \
			[string range $var0 10 11] \
			[string range $var0 12 13] \
			[string range $var0 14 15]]
}

if { $aligned_datalength != $datalength } {
	append result "    "
	for { set i $aligned_datalength } {$i < $datalength} {incr i} {
		binary scan $data "@[set i]H2" var0
		append result [format "0x%2s, " $var0]
	}
}

# Remove either comma+newline or comma+space from the end
set result [string range $result 0 [expr [string length $result] - 3]]

append result "\n};"

puts $outfilefd $result
close $outfilefd
