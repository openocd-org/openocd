-- SPDX-License-Identifier: BSD-3-Clause
----------------------------------------------------------------------------
-- Project Context: nanoXplore USB-JTAG Adapter Board, Spartan6
-- Design Name:     NJTAG USB-JTAG Adapter FPGA source code
-- Module Name:     _angie_openocd.vhd
-- Target Device:   XC6SLX9-2 TQ144
-- Tool versions:   ISE Webpack 13.2 -> 14.2
-- Author:          Ahmed BOUDJELIDA    nanoXplore SAS
----------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
library UNISIM;
use UNISIM.VComponents.all;

entity S609 is port(
  TRST : in std_logic;
  TMS : in std_logic;
  TCK : in std_logic;
  TDI : in std_logic;
  TDO : out std_logic;
  SRST : in std_logic;

  SDA : inout std_logic;
  SDA_DIR : in std_logic;
  SCL : in std_logic;

  FTP : out std_logic_vector(7 downto 0):=(others => '1'); -- Test points
  SI_TDO : in  std_logic;
  ST_0 : out std_logic;
  ST_1 : out std_logic;
  ST_2 : out std_logic;

  ST_4 : out std_logic;
  ST_5 : out std_logic;

  SO_SDA_OUT : out std_logic;
  SO_SDA_IN : in std_logic;
  SO_SCL : out std_logic;

  SO_TRST : out std_logic;
  SO_TMS : out std_logic;
  SO_TCK : out std_logic;
  SO_TDI : out std_logic;
  SO_SRST : out std_logic
);
end S609;

architecture A_S609 of S609 is
begin

--Directions:
ST_0 <= '0';
ST_1 <= '1';

ST_4 <= '0';

--TDO:
TDO <= not SI_TDO;

--TRST - TCK - TMS - TDI:
SO_TRST <= TRST;
SO_TMS <= TMS;
SO_TCK <= TCK;
SO_TDI <= TDI;
ST_2 <= SRST;
SO_SRST <= '0';

SO_SCL <= SCL;

SDA <= not(SO_SDA_IN) when (SDA_DIR = '1') else 'Z';
SO_SDA_OUT <= SDA;

process(SDA_DIR)
begin
	if(SDA_DIR = '1') then
		ST_5 <= '1';
	else
		ST_5 <= '0';
	end if;
end process;


--Points de test:
FTP(0) <= SDA;
FTP(1) <= SCL;
FTP(2) <= not(SO_SDA_IN);
FTP(3) <= SDA_DIR;
FTP(5) <= SRST;
FTP(4) <= SI_TDO;
FTP(6) <= '1';
FTP(7) <= '1';

end A_S609;
