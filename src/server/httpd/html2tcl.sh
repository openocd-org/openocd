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
	puts "Usage: html2tcl.tcl <infile> <outfile>"
	exit 1
}
set infile [lindex $argv 0]
set outfile [lindex $argv 1]
				
set infilefd [open $infile "r"]
set data [read $infilefd]
close $infilefd




if [string match *\.tcl $infile]==0 {
	puts "Not .tcl file, skipping $infile"
	exit 0
}

set outfilefd [ open $outfile "w" ]
if [regexp -start 0 {^\s*<html.*} $data]==0 {
	puts "copy $infile"
	puts -nonewline $outfilefd $data
	close $outfilefd
	exit 0
}

puts "converting $infile"

set result ""
append result "# converted to .tcl by html2tcl.tcl\n"
append result "set buffer \"\"\n"

set pos 0
set done 0
while {$done==0} {
	set start [string first <tcl> $data $pos]
	if $start==-1 {
		# We're done...
		set done 1
		set start [string length $data]
		set end $start
	} else {
		set end [string first </tcl> $data $start]
		if $end==-1 {
			# uh-oh, not closed
			puts "<tcl> not closed!"
			exit 1
		}
	}
	#puts "done $done start $start end $end"
	# Dump HTML into resulting file.
	append result "append buffer {"
	append result [string range $data $pos [expr $start-1]]
	#puts [string range $data $pos $start]
	append result "}\n"
	
	# Dump TCL into resulting file.
	append result "[string range $data [expr $start+5] [expr $end-1]]\n"
	
	set pos [expr $end+6]
}

append result "start_chunked \"html\"\n"
append result {write_chunked $buffer} "\n"
append result "end_chunked\n"

puts $outfilefd $result
close $outfilefd
