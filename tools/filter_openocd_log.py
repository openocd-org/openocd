#!/usr/bin/env python

import sys
import re

# This function is the only OpenOCD-specific part of this script.
def make_canonical(line):
    # Remove the line number and time stamp.
    m = re.match(r"(Debug|Error|Info |User |Warn ): \d+ \d+ (.*)", line)
    if m:
        return "%s: - - %s\n" % (m.group(1), m.group(2))
    else:
        return line

def buf_startswith(buf, sequence):
    if len(buf) < len(sequence):
        return False
    for i, entry in enumerate(sequence):
        if entry[1] != buf[i][1]:
            return False
    return True

def shorten_buffer(outfd, buf, current_repetition):
    """Do something to the buffer to make it shorter. If we can't compress
    anything, then print out the first line and remove it."""
    length_before = len(buf)

    if current_repetition:
        while buf_startswith(buf, current_repetition[0]):
            del buf[:len(current_repetition[0])]
            current_repetition[1] += 1
        if len(buf) < length_before:
            return current_repetition
        outfd.write("## The following %d lines repeat %d times:\n" % (
                len(current_repetition[0]), current_repetition[1]))
        for entry in current_repetition[0]:
            outfd.write("# %s" % entry[1])

    # Look for repeated sequences...
    repetitions = []
    for length in range(1, len(buf)/2):
        # Is there a repeating sequence of `length` lines?
        matched_lines = 0
        for i, entry in enumerate(buf[length:]):
            if entry[1] == buf[i % length][1]:
                matched_lines += 1
            else:
                break
        if matched_lines >= length:
            repetitions.append((matched_lines + length, length))

    if repetitions:
        repetitions.sort(key=lambda entry: (entry[0] * (entry[1] / entry[0]), -entry[1]))
        matched_lines, length = repetitions[-1]
        repeated = matched_lines / length
        if repeated * length >= 3:
            sequence = buf[:length]
            del buf[:repeated * length]

            if matched_lines == length_before:
                # Could be continued...
                return [sequence, repeated]

            else:
                outfd.write("## The following %d lines repeat %d times:\n" %
                        (length, repeated))
                for entry in sequence:
                    outfd.write("# %s" % entry[1])
                return None

    if len(buf) >= length_before:
        line, _ = buf[0]
        outfd.write(line)
        buf.pop(0)

    if length_before <= len(buf):
        print "Buffer:"
        for entry in buf:
            print "%r" % entry[0]
        assert False

    return None

def compress_log(infd, outfd, window):
    """Compress log by finding repeated runs of lines. Runs in O(lines *
    window**2), which can probably be improved."""
    # Contains line, canonical tuples
    buf = []
    current_repetition = None

    for line in infd:
        buf.append((line, make_canonical(line)))
        if len(buf) > window:
            current_repetition = shorten_buffer(outfd, buf, current_repetition)

    while len(buf) > 0:
        current_repetition = shorten_buffer(outfd, buf, current_repetition)

def main(args):
    import argparse
    parser = argparse.ArgumentParser(
            description='Combine repeated OpenOCD debug output lines. This is '
            'very helpful when looking at verbose log files where e.g. target '
            'polling is repeated over and over.',
            epilog='If no files are specified, read standard input.',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('file', nargs='*', help='input file')
    parser.add_argument('-o', '--output', help='output file', default=sys.stdout)
    parser.add_argument('-w', '--window', type=int, default=100,
            help='number of lines to consider when looking for repetitions')
    args = parser.parse_args(args)

    if args.file:
        for f in args.file:
            compress_log(open(f, "r"), args.output, args.window)
    else:
        compress_log(sys.stdin, args.output, args.window)

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
