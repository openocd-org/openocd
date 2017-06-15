#!/usr/bin/python3
#
#  Copyright (C) 2015 Robert Jordens <jordens@gmail.com>
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#

from migen import *
from migen.build.generic_platform import *
from migen.build import xilinx


"""
This migen script produces proxy bitstreams to allow programming SPI flashes
behind FPGAs.

Bitstream binaries built with this script are available at:
https://github.com/jordens/bscan_spi_bitstreams

JTAG signalling is connected directly to SPI signalling. CS_N is
asserted when the JTAG IR contains the USER1 instruction and the state is
SHIFT-DR. Xilinx bscan cells sample TDO on falling TCK and forward it.
MISO requires sampling on rising CLK and leads to one cycle of latency.

https://github.com/m-labs/migen
"""


class Spartan3(Module):
    macro = "BSCAN_SPARTAN3"
    toolchain = "ise"

    def __init__(self, platform):
        platform.toolchain.bitgen_opt += " -g compress -g UnusedPin:Pullup"
        self.clock_domains.cd_jtag = ClockDomain(reset_less=True)
        spi = platform.request("spiflash")
        shift = Signal()
        tdo = Signal()
        sel1 = Signal()
        self.comb += [
            self.cd_jtag.clk.eq(spi.clk),
            spi.cs_n.eq(~shift | ~sel1),
        ]
        self.sync.jtag += tdo.eq(spi.miso)
        self.specials += Instance(self.macro,
                                  o_DRCK1=spi.clk, o_SHIFT=shift,
                                  o_TDI=spi.mosi, i_TDO1=tdo, i_TDO2=0,
                                  o_SEL1=sel1)


class Spartan3A(Spartan3):
    macro = "BSCAN_SPARTAN3A"


class Spartan6(Module):
    toolchain = "ise"

    def __init__(self, platform):
        platform.toolchain.bitgen_opt += " -g compress -g UnusedPin:Pullup"
        self.clock_domains.cd_jtag = ClockDomain(reset_less=True)
        spi = platform.request("spiflash")
        shift = Signal()
        tdo = Signal()
        sel = Signal()
        self.comb += self.cd_jtag.clk.eq(spi.clk), spi.cs_n.eq(~shift | ~sel)
        self.sync.jtag += tdo.eq(spi.miso)
        self.specials += Instance("BSCAN_SPARTAN6", p_JTAG_CHAIN=1,
                                  o_TCK=spi.clk, o_SHIFT=shift, o_SEL=sel,
                                  o_TDI=spi.mosi, i_TDO=tdo)


class Series7(Module):
    toolchain = "vivado"

    def __init__(self, platform):
        platform.toolchain.bitstream_commands.extend([
            "set_property BITSTREAM.GENERAL.COMPRESS True [current_design]",
            "set_property BITSTREAM.CONFIG.UNUSEDPIN Pullnone [current_design]",
        ])
        self.clock_domains.cd_jtag = ClockDomain(reset_less=True)
        spi = platform.request("spiflash")
        clk = Signal()
        shift = Signal()
        tdo = Signal()
        sel = Signal()
        self.comb += self.cd_jtag.clk.eq(clk), spi.cs_n.eq(~shift | ~sel)
        self.sync.jtag += tdo.eq(spi.miso)
        self.specials += Instance("BSCANE2", p_JTAG_CHAIN=1,
                                  o_SHIFT=shift, o_TCK=clk, o_SEL=sel,
                                  o_TDI=spi.mosi, i_TDO=tdo)
        self.specials += Instance("STARTUPE2", i_CLK=0, i_GSR=0, i_GTS=0,
                                  i_KEYCLEARB=0, i_PACK=1, i_USRCCLKO=clk,
                                  i_USRCCLKTS=0, i_USRDONEO=1, i_USRDONETS=1)


class XilinxBscanSpi(xilinx.XilinxPlatform):
    packages = {
        # (package-speedgrade, id): [cs_n, clk, mosi, miso, *pullups]
        ("cp132", 1): ["M2", "N12", "N2", "N8"],
        ("fg320", 1): ["U3", "U16", "T4", "N10"],
        ("fg320", 2): ["V3", "U16", "T11", "V16"],
        ("fg484", 1): ["Y4", "AA20", "AB14", "AB20"],
        ("fgg484", 1): ["Y4", "AA20", "AB14", "AB20"],
        ("fgg400", 1): ["Y2", "Y19", "W12", "W18"],
        ("ftg256", 1): ["T2", "R14", "P10", "T14"],
        ("ft256", 1): ["T2", "R14", "P10", "T14"],
        ("fg400", 1): ["Y2", "Y19", "W12", "W18"],
        ("cs484", 1): ["U7", "V17", "V13", "W17"],
        ("qg144-2", 1): ["P38", "P70", "P64", "P65", "P62", "P61"],
        ("cpg196-2", 1): ["P2", "N13", "P11", "N11", "N10", "P10"],
        ("cpg236-1", 1): ["K19", None, "D18", "D19", "G18", "F18"],
        ("csg484-2", 1): ["AB5", "W17", "AB17", "Y17", "V13", "W13"],
        ("csg324-2", 1): ["V3", "R15", "T13", "R13", "T14", "V14"],
        ("csg324-1", 1): ["L13", None, "K17", "K18", "L14", "M14"],
        ("fbg484-1", 1): ["T19", None, "P22", "R22", "P21", "R21"],
        ("fbg484-1", 2): ["L16", None, "H18", "H19", "G18", "F19"],
        ("fbg676-1", 1): ["C23", None, "B24", "A25", "B22", "A22"],
        ("ffg901-1", 1): ["V26", None, "R30", "T30", "R28", "T28"],
        ("ffg1156-1", 1): ["V30", None, "AA33", "AA34", "Y33", "Y34"],
        ("ffg1157-1", 1): ["AL33", None, "AN33", "AN34", "AK34", "AL34"],
        ("ffg1158-1", 1): ["C24", None, "A23", "A24", "B26", "A26"],
        ("ffg1926-1", 1): ["AK33", None, "AN34", "AN35", "AJ34", "AK34"],
        ("fhg1761-1", 1): ["AL36", None, "AM36", "AN36", "AJ36", "AJ37"],
        ("flg1155-1", 1): ["AL28", None, "AE28", "AF28", "AJ29", "AJ30"],
        ("flg1932-1", 1): ["V32", None, "T33", "R33", "U31", "T31"],
        ("flg1926-1", 1): ["AK33", None, "AN34", "AN35", "AJ34", "AK34"],
    }

    pinouts = {
        # bitstreams are named by die, package does not matter, speed grade
        # should not matter.
        #
        # chip: (package, id, standard, class)
        "xc3s100e": ("cp132", 1, "LVCMOS33", Spartan3),
        "xc3s1200e": ("fg320", 1, "LVCMOS33", Spartan3),
        "xc3s1400a": ("fg484", 1, "LVCMOS33", Spartan3A),
        "xc3s1400an": ("fgg484", 1, "LVCMOS33", Spartan3A),
        "xc3s1600e": ("fg320", 1, "LVCMOS33", Spartan3),
        "xc3s200a": ("fg320", 2, "LVCMOS33", Spartan3A),
        "xc3s200an": ("ftg256", 1, "LVCMOS33", Spartan3A),
        "xc3s250e": ("cp132", 1, "LVCMOS33", Spartan3),
        "xc3s400a": ("fg320", 2, "LVCMOS33", Spartan3A),
        "xc3s400an": ("fgg400", 1, "LVCMOS33", Spartan3A),
        "xc3s500e": ("cp132", 1, "LVCMOS33", Spartan3),
        "xc3s50a": ("ft256", 1, "LVCMOS33", Spartan3A),
        "xc3s50an": ("ftg256", 1, "LVCMOS33", Spartan3A),
        "xc3s700a": ("fg400", 1, "LVCMOS33", Spartan3A),
        "xc3s700an": ("fgg484", 1, "LVCMOS33", Spartan3A),
        "xc3sd1800a": ("cs484", 1, "LVCMOS33", Spartan3A),
        "xc3sd3400a": ("cs484", 1, "LVCMOS33", Spartan3A),

        "xc6slx100": ("csg484-2", 1, "LVCMOS33", Spartan6),
        "xc6slx100t": ("csg484-2", 1, "LVCMOS33", Spartan6),
        "xc6slx150": ("csg484-2", 1, "LVCMOS33", Spartan6),
        "xc6slx150t": ("csg484-2", 1, "LVCMOS33", Spartan6),
        "xc6slx16": ("cpg196-2", 1, "LVCMOS33", Spartan6),
        "xc6slx25": ("csg324-2", 1, "LVCMOS33", Spartan6),
        "xc6slx25t": ("csg324-2", 1, "LVCMOS33", Spartan6),
        "xc6slx45": ("csg324-2", 1, "LVCMOS33", Spartan6),
        "xc6slx45t": ("csg324-2", 1, "LVCMOS33", Spartan6),
        "xc6slx4": ("cpg196-2", 1, "LVCMOS33", Spartan6),
        "xc6slx4t": ("qg144-2", 1, "LVCMOS33", Spartan6),
        "xc6slx75": ("csg484-2", 1, "LVCMOS33", Spartan6),
        "xc6slx75t": ("csg484-2", 1, "LVCMOS33", Spartan6),
        "xc6slx9": ("cpg196-2", 1, "LVCMOS33", Spartan6),
        "xc6slx9t": ("qg144-2", 1, "LVCMOS33", Spartan6),

        "xc7a100t": ("csg324-1", 1, "LVCMOS25", Series7),
        "xc7a15t": ("cpg236-1", 1, "LVCMOS25", Series7),
        "xc7a200t": ("fbg484-1", 1, "LVCMOS25", Series7),
        "xc7a35t": ("cpg236-1", 1, "LVCMOS25", Series7),
        "xc7a50t": ("cpg236-1", 1, "LVCMOS25", Series7),
        "xc7a75t": ("csg324-1", 1, "LVCMOS25", Series7),
        "xc7k160t": ("fbg484-1", 2, "LVCMOS25", Series7),
        "xc7k325t": ("fbg676-1", 1, "LVCMOS25", Series7),
        "xc7k355t": ("ffg901-1", 1, "LVCMOS25", Series7),
        "xc7k410t": ("fbg676-1", 1, "LVCMOS25", Series7),
        "xc7k420t": ("ffg1156-1", 1, "LVCMOS25", Series7),
        "xc7k480t": ("ffg1156-1", 1, "LVCMOS25", Series7),
        "xc7k70t": ("fbg484-1", 2, "LVCMOS25", Series7),
        "xc7v2000t": ("fhg1761-1", 1, "LVCMOS18", Series7),
        "xc7v585t": ("ffg1157-1", 1, "LVCMOS18", Series7),
        "xc7vh580t": ("flg1155-1", 1, "LVCMOS18", Series7),
        "xc7vh870t": ("flg1932-1", 1, "LVCMOS18", Series7),
        "xc7vx1140t": ("flg1926-1", 1, "LVCMOS18", Series7),
        "xc7vx330t": ("ffg1157-1", 1, "LVCMOS18", Series7),
        "xc7vx415t": ("ffg1157-1", 1, "LVCMOS18", Series7),
        "xc7vx485t": ("ffg1157-1", 1, "LVCMOS18", Series7),
        "xc7vx550t": ("ffg1158-1", 1, "LVCMOS18", Series7),
        "xc7vx690t": ("ffg1157-1", 1, "LVCMOS18", Series7),
        "xc7vx980t": ("ffg1926-1", 1, "LVCMOS18", Series7),
    }

    def __init__(self, device, pins, std, toolchain="ise"):
        cs_n, clk, mosi, miso = pins[:4]
        io = ["spiflash", 0,
              Subsignal("cs_n", Pins(cs_n)),
              Subsignal("mosi", Pins(mosi)),
              Subsignal("miso", Pins(miso), Misc("PULLUP")),
              IOStandard(std),
              ]
        if clk:
            io.append(Subsignal("clk", Pins(clk)))
        for i, p in enumerate(pins[4:]):
            io.append(Subsignal("pullup{}".format(i), Pins(p), Misc("PULLUP")))
        xilinx.XilinxPlatform.__init__(self, device, [io], toolchain=toolchain)

    @classmethod
    def make(cls, device, errors=False):
        pkg, id, std, Top = cls.pinouts[device]
        pins = cls.packages[(pkg, id)]
        platform = cls("{}-{}".format(device, pkg), pins, std, Top.toolchain)
        top = Top(platform)
        name = "bscan_spi_{}".format(device)
        dir = "build_{}".format(device)
        try:
            platform.build(top, build_name=name, build_dir=dir)
        except Exception as e:
            print(("ERROR: xilinx_bscan_spi build failed "
                  "for {}: {}").format(device, e))
            if errors:
                raise


if __name__ == "__main__":
    import argparse
    import multiprocessing
    p = argparse.ArgumentParser(description="build bscan_spi bitstreams "
                                "for openocd jtagspi flash driver")
    p.add_argument("device", nargs="*",
                   default=sorted(list(XilinxBscanSpi.pinouts)),
                   help="build for these devices (default: %(default)s)")
    p.add_argument("-p", "--parallel", default=1, type=int,
                   help="number of parallel builds (default: %(default)s)")
    args = p.parse_args()
    pool = multiprocessing.Pool(args.parallel)
    pool.map(XilinxBscanSpi.make, args.device, chunksize=1)
