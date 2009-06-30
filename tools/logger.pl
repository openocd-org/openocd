#!/usr/bin/perl
# logger.pl: masks long meaningless output with pretty lines of dots
#  Details: 1) reads lines from STDIN and echos them on STDOUT,
#           2) print a '.' to STDERR every $N lines.
#           3) print a newline after a sequence of $C dots

use strict;
use warnings;

# make sure all output gets displayed immediately
$| = 1;

# TODO: add -n and -c options w/ zero checks)
# line and column limits
my $N = 10;
my $C = 72;

# current line and column counters
my $n = 0;
my $c = 0;

# read all lines from STDIN
while (<STDIN>)
{
	# echo line to output
	print STDOUT $_;
	# echo line to console if it is important
	if (/(Warning|Error)/) {
		print STDERR "\n" if $c;
		print STDERR $_;
		$c = 0;
	}
	# only display progress every Nth step
	next if ++$n % $N;
	print STDERR ".";
	# wrap at column C to provide fixed-width rows of dots
	print STDERR "\n" unless ++$c % $C;
}

print STDERR "\n" if $c;
