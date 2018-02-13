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

import unittest

import migen as mg
import migen.build.generic_platform as mb
from migen.genlib import io
from migen.build import xilinx


"""
This migen script produces proxy bitstreams to allow programming SPI flashes
behind FPGAs.

Bitstream binaries built with this script are available at:
https://github.com/jordens/bscan_spi_bitstreams

A JTAG2SPI transfer consists of:

1. an arbitrary number of 0 bits (from BYPASS registers in front of the
   JTAG2SPI DR)
2. a marker bit (1) indicating the start of the JTAG2SPI transaction
3. 32 bits (big endian) describing the length of the SPI transaction
4. a number of SPI clock cycles (corresponding to 3.) with CS_N asserted
5. an arbitrary number of cycles (to shift MISO/TDO data through subsequent
   BYPASS registers)

Notes:

* The JTAG2SPI DR is 1 bit long (due to different sampling edges of
  {MISO,MOSI}/{TDO,TDI}).
* MOSI is TDI with half a cycle delay.
* TDO is MISO with half a cycle delay.
* CAPTURE-DR needs to be performed before SHIFT-DR on the BYPASSed TAPs in
  JTAG chain to clear the BYPASS registers to 0.

https://github.com/m-labs/migen
"""


class JTAG2SPI(mg.Module):
    def __init__(self, spi=None, bits=32):
        self.jtag = mg.Record([
            ("sel", 1),
            ("shift", 1),
            ("capture", 1),
            ("tck", 1),
            ("tdi", 1),
            ("tdo", 1),
        ])
        self.cs_n = mg.TSTriple()
        self.clk = mg.TSTriple()
        self.mosi = mg.TSTriple()
        self.miso = mg.TSTriple()

        # # #

        self.cs_n.o.reset = mg.Constant(1)
        self.mosi.o.reset_less = True
        bits = mg.Signal(bits, reset_less=True)
        head = mg.Signal(max=len(bits), reset=len(bits) - 1)
        self.clock_domains.cd_sys = mg.ClockDomain()
        self.submodules.fsm = mg.FSM("IDLE")
        if spi is not None:
            self.specials += [
                    self.cs_n.get_tristate(spi.cs_n),
                    self.mosi.get_tristate(spi.mosi),
                    self.miso.get_tristate(spi.miso),
            ]
            if hasattr(spi, "clk"):  # 7 Series drive it fixed
                self.specials += self.clk.get_tristate(spi.clk)
                # self.specials += io.DDROutput(1, 0, spi.clk, self.clk.o)
        self.comb += [
                self.cd_sys.rst.eq(self.jtag.sel & self.jtag.capture),
                self.cd_sys.clk.eq(self.jtag.tck),
                self.cs_n.oe.eq(self.jtag.sel),
                self.clk.oe.eq(self.jtag.sel),
                self.mosi.oe.eq(self.jtag.sel),
                self.miso.oe.eq(0),
                # Do not suppress CLK toggles outside CS_N asserted.
                # Xilinx USRCCLK0 requires three dummy cycles to do anything
                # https://www.xilinx.com/support/answers/52626.html
                # This is fine since CS_N changes only on falling CLK.
                self.clk.o.eq(~self.jtag.tck),
                self.jtag.tdo.eq(self.miso.i),
        ]
        # Latency calculation (in half cycles):
        # 0 (falling TCK, rising CLK):
        #   JTAG adapter: set TDI
        # 1 (rising TCK, falling CLK):
        #   JTAG2SPI: sample TDI -> set MOSI
        #   SPI: set MISO
        # 2 (falling TCK, rising CLK):
        #   SPI: sample MOSI
        #   JTAG2SPI (BSCAN primitive): sample MISO -> set TDO
        # 3 (rising TCK, falling CLK):
        #   JTAG adapter: sample TDO
        self.fsm.act("IDLE",
                mg.If(self.jtag.tdi & self.jtag.sel & self.jtag.shift,
                    mg.NextState("HEAD")
                )
        )
        self.fsm.act("HEAD",
                mg.If(head == 0,
                    mg.NextState("XFER")
                )
        )
        self.fsm.act("XFER",
                mg.If(bits == 0,
                    mg.NextState("IDLE")
                ),
        )
        self.sync += [
                self.mosi.o.eq(self.jtag.tdi),
                self.cs_n.o.eq(~self.fsm.ongoing("XFER")),
                mg.If(self.fsm.ongoing("HEAD"),
                    bits.eq(mg.Cat(self.jtag.tdi, bits)),
                    head.eq(head - 1)
                ),
                mg.If(self.fsm.ongoing("XFER"),
                    bits.eq(bits - 1)
                )
        ]


class JTAG2SPITest(unittest.TestCase):
    def setUp(self):
        self.bits = 8
        self.dut = JTAG2SPI(bits=self.bits)

    def test_instantiate(self):
        pass

    def test_initial_conditions(self):
        def check():
            yield
            self.assertEqual((yield self.dut.cs_n.oe), 0)
            self.assertEqual((yield self.dut.mosi.oe), 0)
            self.assertEqual((yield self.dut.miso.oe), 0)
            self.assertEqual((yield self.dut.clk.oe), 0)
        mg.run_simulation(self.dut, check())

    def test_enable(self):
        def check():
            yield self.dut.jtag.sel.eq(1)
            yield self.dut.jtag.shift.eq(1)
            yield
            self.assertEqual((yield self.dut.cs_n.oe), 1)
            self.assertEqual((yield self.dut.mosi.oe), 1)
            self.assertEqual((yield self.dut.miso.oe), 0)
            self.assertEqual((yield self.dut.clk.oe), 1)
        mg.run_simulation(self.dut, check())

    def run_seq(self, tdi, tdo, spi=None):
        yield self.dut.jtag.sel.eq(1)
        yield
        yield self.dut.jtag.shift.eq(1)
        for di in tdi:
            yield self.dut.jtag.tdi.eq(di)
            yield
            tdo.append((yield self.dut.jtag.tdo))
            if spi is not None:
                v = []
                for k in "cs_n clk mosi miso".split():
                    t = getattr(self.dut, k)
                    v.append("{}>".format((yield t.o)) if (yield t.oe)
                            else "<{}".format((yield t.i)))
                spi.append(" ".join(v))
        yield self.dut.jtag.sel.eq(0)
        yield
        yield self.dut.jtag.shift.eq(0)
        yield

    def test_shift(self):
        bits = 8
        data = 0x81
        tdi = [0, 0, 1]  # dummy from BYPASS TAPs and marker
        tdi += [((bits - 1) >> j) & 1 for j in range(self.bits - 1, -1, -1)]
        tdi += [(data >> j) & 1 for j in range(bits)]
        tdi += [0, 0, 0, 0]  # dummy from BYPASS TAPs
        tdo = []
        spi = []
        mg.run_simulation(self.dut, self.run_seq(tdi, tdo, spi))
        # print(tdo)
        for l in spi:
            print(l)


class Spartan3(mg.Module):
    macro = "BSCAN_SPARTAN3"
    toolchain = "ise"

    def __init__(self, platform):
        platform.toolchain.bitgen_opt += " -g compress -g UnusedPin:Pullup"
        self.submodules.j2s = j2s = JTAG2SPI(platform.request("spiflash"))
        self.specials += [
                mg.Instance(
                    self.macro,
                    o_SHIFT=j2s.jtag.shift, o_SEL1=j2s.jtag.sel,
                    o_CAPTURE=j2s.jtag.capture,
                    o_DRCK1=j2s.jtag.tck,
                    o_TDI=j2s.jtag.tdi, i_TDO1=j2s.jtag.tdo,
                    i_TDO2=0),
        ]
        platform.add_period_constraint(j2s.jtag.tck, 6)


class Spartan3A(Spartan3):
    macro = "BSCAN_SPARTAN3A"


class Spartan6(mg.Module):
    toolchain = "ise"

    def __init__(self, platform):
        platform.toolchain.bitgen_opt += " -g compress -g UnusedPin:Pullup"
        self.submodules.j2s = j2s = JTAG2SPI(platform.request("spiflash"))
        # clk = mg.Signal()
        self.specials += [
                mg.Instance(
                    "BSCAN_SPARTAN6", p_JTAG_CHAIN=1,
                    o_SHIFT=j2s.jtag.shift, o_SEL=j2s.jtag.sel,
                    o_CAPTURE=j2s.jtag.capture,
                    o_DRCK=j2s.jtag.tck,
                    o_TDI=j2s.jtag.tdi, i_TDO=j2s.jtag.tdo),
                # mg.Instance("BUFG", i_I=clk, o_O=j2s.jtag.tck)
        ]
        platform.add_period_constraint(j2s.jtag.tck, 6)


class Series7(mg.Module):
    toolchain = "vivado"

    def __init__(self, platform):
        platform.toolchain.bitstream_commands.extend([
            "set_property BITSTREAM.GENERAL.COMPRESS True [current_design]",
            "set_property BITSTREAM.CONFIG.UNUSEDPIN Pullnone [current_design]"
        ])
        self.submodules.j2s = j2s = JTAG2SPI(platform.request("spiflash"))
        # clk = mg.Signal()
        self.specials += [
                mg.Instance(
                    "BSCANE2", p_JTAG_CHAIN=1,
                    o_SHIFT=j2s.jtag.shift, o_SEL=j2s.jtag.sel,
                    o_CAPTURE=j2s.jtag.capture,
                    o_DRCK=j2s.jtag.tck,
                    o_TDI=j2s.jtag.tdi, i_TDO=j2s.jtag.tdo),
                mg.Instance(
                    "STARTUPE2", i_CLK=0, i_GSR=0, i_GTS=0,
                    i_KEYCLEARB=0, i_PACK=1,
                    i_USRCCLKO=j2s.clk.o, i_USRCCLKTS=~j2s.clk.oe,
                    i_USRDONEO=1, i_USRDONETS=1),
                # mg.Instance("BUFG", i_I=clk, o_O=j2s.jtag.tck)
        ]
        platform.add_period_constraint(j2s.jtag.tck, 6)
        try:
            self.comb += [
                    platform.request("user_sma_gpio_p").eq(j2s.cs_n.i),
                    platform.request("user_sma_gpio_n").eq(j2s.clk.o),
                    platform.request("user_sma_clock_p").eq(j2s.mosi.o),
                    platform.request("user_sma_clock_n").eq(j2s.miso.i),
            ]
        except mb.ConstraintError:
            pass


class Ultrascale(mg.Module):
    toolchain = "vivado"

    def __init__(self, platform):
        platform.toolchain.bitstream_commands.extend([
            "set_property BITSTREAM.GENERAL.COMPRESS True [current_design]",
            "set_property BITSTREAM.CONFIG.UNUSEDPIN Pullnone [current_design]",
        ])
        self.submodules.j2s0 = j2s0 = JTAG2SPI()
        self.submodules.j2s1 = j2s1 = JTAG2SPI(platform.request("spiflash"))
        di = mg.Signal(4)
        self.comb += mg.Cat(j2s0.mosi.i, j2s0.miso.i).eq(di)
        self.specials += [
                mg.Instance("BSCANE2", p_JTAG_CHAIN=1,
                    o_SHIFT=j2s0.jtag.shift, o_SEL=j2s0.jtag.sel,
                    o_CAPTURE=j2s0.jtag.capture,
                    o_DRCK=j2s0.jtag.tck,
                    o_TDI=j2s0.jtag.tdi, i_TDO=j2s0.jtag.tdo),
                mg.Instance("BSCANE2", p_JTAG_CHAIN=2,
                    o_SHIFT=j2s1.jtag.shift, o_SEL=j2s1.jtag.sel,
                    o_CAPTURE=j2s1.jtag.capture,
                    o_DRCK=j2s1.jtag.tck,
                    o_TDI=j2s1.jtag.tdi, i_TDO=j2s1.jtag.tdo),
                mg.Instance("STARTUPE3", i_GSR=0, i_GTS=0,
                    i_KEYCLEARB=0, i_PACK=1,
                    i_USRDONEO=1, i_USRDONETS=1,
                    i_USRCCLKO=mg.Mux(j2s0.clk.oe, j2s0.clk.o, j2s1.clk.o),
                    i_USRCCLKTS=~(j2s0.clk.oe | j2s1.clk.oe),
                    i_FCSBO=j2s0.cs_n.o, i_FCSBTS=~j2s0.cs_n.oe,
                    o_DI=di,
                    i_DO=mg.Cat(j2s0.mosi.o, j2s0.miso.o, 0, 0),
                    i_DTS=mg.Cat(~j2s0.mosi.oe, ~j2s0.miso.oe, 1, 1))
        ]
        platform.add_period_constraint(j2s0.jtag.tck, 6)
        platform.add_period_constraint(j2s1.jtag.tck, 6)


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
        ("ffg900-1", 1): ["U19", None, "P24", "R25", "R20", "R21"],
        ("ffg1156-1", 1): ["V30", None, "AA33", "AA34", "Y33", "Y34"],
        ("ffg1157-1", 1): ["AL33", None, "AN33", "AN34", "AK34", "AL34"],
        ("ffg1158-1", 1): ["C24", None, "A23", "A24", "B26", "A26"],
        ("ffg1926-1", 1): ["AK33", None, "AN34", "AN35", "AJ34", "AK34"],
        ("fhg1761-1", 1): ["AL36", None, "AM36", "AN36", "AJ36", "AJ37"],
        ("flg1155-1", 1): ["AL28", None, "AE28", "AF28", "AJ29", "AJ30"],
        ("flg1932-1", 1): ["V32", None, "T33", "R33", "U31", "T31"],
        ("flg1926-1", 1): ["AK33", None, "AN34", "AN35", "AJ34", "AK34"],

        ("ffva1156-2-e", 1): ["G26", None, "M20", "L20", "R21", "R22"],
        ("ffva1156-2-e", "sayma"): ["K21", None, "M20", "L20", "R21", "R22"],
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
        "xc7k325t-debug": ("ffg900-1", 1, "LVCMOS25", Series7),
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

        "xcku040": ("ffva1156-2-e", 1, "LVCMOS18", Ultrascale),
        "xcku040-sayma": ("ffva1156-2-e", "sayma", "LVCMOS18", Ultrascale),
    }

    def __init__(self, device, pins, std, toolchain="ise"):
        ios = [self.make_spi(0, pins, std, toolchain)]
        if device == "xc7k325t-ffg900-1":  # debug
            ios += [
                ("user_sma_clock_p", 0, mb.Pins("L25"), mb.IOStandard("LVCMOS25")),
                ("user_sma_clock_n", 0, mb.Pins("K25"), mb.IOStandard("LVCMOS25")),
                ("user_sma_gpio_p", 0, mb.Pins("Y23"), mb.IOStandard("LVCMOS25")),
                ("user_sma_gpio_n", 0, mb.Pins("Y24"), mb.IOStandard("LVCMOS25")),
            ]
        xilinx.XilinxPlatform.__init__(self, device, ios, toolchain=toolchain)

    @staticmethod
    def make_spi(i, pins, std, toolchain):
        pu = "PULLUP" if toolchain == "ise" else "PULLUP TRUE"
        pd = "PULLDOWN" if toolchain == "ise" else "PULLDOWN TRUE"
        cs_n, clk, mosi, miso = pins[:4]
        io = ["spiflash", i,
            mb.Subsignal("cs_n", mb.Pins(cs_n), mb.Misc(pu)),
            mb.Subsignal("mosi", mb.Pins(mosi), mb.Misc(pu)),
            mb.Subsignal("miso", mb.Pins(miso), mb.Misc(pu)),
            mb.IOStandard(std),
            ]
        if clk:
            io.append(mb.Subsignal("clk", mb.Pins(clk), mb.Misc(pd)))
        for i, p in enumerate(pins[4:]):
            io.append(mb.Subsignal("pullup{}".format(i), mb.Pins(p),
                                mb.Misc(pu)))
        return io

    @classmethod
    def make(cls, target, errors=False):
        pkg, id, std, Top = cls.pinouts[target]
        pins = cls.packages[(pkg, id)]
        device = target.split("-", 1)[0]
        platform = cls("{}-{}".format(device, pkg), pins, std, Top.toolchain)
        top = Top(platform)
        name = "bscan_spi_{}".format(target)
        try:
            platform.build(top, build_name=name)
        except Exception as e:
            print(("ERROR: xilinx_bscan_spi build failed "
                  "for {}: {}").format(target, e))
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
