# Simple tcl client to connect to openocd
puts "Use empty line to exit"
set fo [socket 127.0.0.1 6666]
puts -nonewline stdout "> "
flush stdout
while {[gets stdin line] >= 0} {
    if {$line eq {}} break
    puts $fo $line
    flush $fo
    gets $fo line
    puts $line
    puts -nonewline stdout "> "
    flush stdout
}
close $fo
