-- SPDX-License-Identifier: BSD-3-Clause
----------------------------------------------------------------------------
-- Project Context: nanoXplore USB to JTAG/I2C Adapter Board, Spartan6
-- Design Name:     ANGIE USB to JTAG/I2C Adapter FPGA source code
-- Module Name:     angie_bitstream.vhd
-- Target Device:   XC6SLX9-2 TQ144
-- Tool versions:   ISE Webpack 13.2 -> 14.2
-- Author:          Ahmed BOUDJELIDA    nanoXplore SAS
----------------------------------------------------------------------------
library work;
use work.all;
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity angie_bitstream is port(
  SDA_IO : inout std_logic;
  SDA_DIR_I : in std_logic;
  SCL_I : in std_logic;

  JPW_I : in std_logic;     --Devkit power

  SO_SDA_OUT_O : out std_logic;
  SO_SDA_IN_I : in std_logic;
  SO_SCL_O : out std_logic;

  ST_0_O : out std_logic;
  ST_1_O : out std_logic;
  ST_2_O : out std_logic;
  ST_3_O : out std_logic;
  ST_4_O : out std_logic;
  ST_5_O : out std_logic;

  SO_TRST_O : out std_logic;
  SO_TMS_O : out std_logic;
  SO_TCK_O : out std_logic;
  SO_TDI_O : out std_logic;
  SO_SRST_O : out std_logic;
  SI_TDO_I : in std_logic;

  PA2_I : in std_logic; -- GPIF IN

  -- Clock 48MHz
  IFCLK_I : in std_logic;

  GCTL0_I : in std_logic;
  GRDY1_I : out std_logic;
  GD_IO : inout std_logic_vector(7 downto 0);
  FTP_O : out std_logic_vector(15 downto 0)
);
end angie_bitstream;

architecture A_angie_bitstream of angie_bitstream is
----------------------------------------Fifo out (PC to devkit)
signal rst_o, clk_wr_o, clk_rd_o : std_logic;
signal write_en_o, read_en_o : std_logic;
signal data_in_o, data_out_o : std_logic_vector(7 downto 0);
signal empty_o, full_o : std_logic;

----------------------------------------Fifo in (devkit to PC)
signal rst_i, clk_wr_i, clk_rd_i : std_logic;
signal write_en_i, read_en_i : std_logic;
signal data_in_i, data_out_i : std_logic_vector(7 downto 0);
signal empty_i, full_i : std_logic;

signal wr_o, rd_i : std_logic;

----------------------------------------MAE
signal transit1, transit2 : std_logic;

----------------------------------------DFF
signal pa2_dff_clk, pa2_dff_rst, pa2_dff_d, pa2_dff_q : std_logic;
signal trst_clk, trst_rst, trst_d, trst_q : std_logic;
signal tms_clk, tms_rst, tms_d, tms_q : std_logic;
signal tdi_clk, tdi_rst, tdi_d, tdi_q : std_logic;
signal tdo_clk, tdo_rst, tdo_d, tdo_q : std_logic;

----------------------------------------clk_div
signal clk_div_in, clk_div_out, reset_clk_div : std_logic;
signal clk_div2_in, clk_div2_out, reset_clk_div2 : std_logic;

----------------------------------------MAE
type State_Type is (IDLE, WRITE_OUT, WRITE_IN, DELAY, READ_IN);
signal state, state2 : State_Type;
signal reset_mae, reset_mae2 : std_logic;

-- Add Component DFF
component DFF
    Port (
        clk : in std_logic;
        reset : in std_logic;
        d : in std_logic;
        q : out std_logic
    );
end component;

-- Add Component Clk_div
component clk_div
Port (
    clk_in : in std_logic;
    reset : in std_logic;
    clk_out : out std_logic
);
end component;

-- Add component FIFO 64B
component fifo_generator_v9_3
PORT (
    rst : IN STD_LOGIC;
    wr_clk : IN STD_LOGIC;
    rd_clk : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC
  );
end component;

signal state1_debug, state2_debug : std_logic;

begin
-------------------------------------------------------------I2C :
SDA_IO <= not(SO_SDA_IN_I) when (SDA_DIR_I = '1') else 'Z';
SO_SDA_OUT_O <= SDA_IO;
ST_5_O <= SDA_DIR_I;

SO_SCL_O <= SCL_I when (JPW_I = '1') else '0';
ST_4_O <= '0';

------------------------------------------------------------JTAG :
-- Instantiate the Clk div by 10
clk_div_inst : clk_div
port map (
    clk_in => clk_div_in,
    reset => reset_clk_div,
    clk_out => clk_div_out
);
-- Instantiate the Clk div by 10
clk_div2_inst : clk_div
port map (
    clk_in => clk_div2_in,
    reset => reset_clk_div2,
    clk_out => clk_div2_out
);

-- Instantiate DFFs
DFF_inst_PA2 : DFF
port map (
   clk => pa2_dff_clk,
   reset => pa2_dff_rst,
   d => pa2_dff_d,
   q => pa2_dff_q
);

DFF_inst_TRST : DFF
port map (
   clk => trst_clk,
   reset => trst_rst,
   d => trst_d,
   q => trst_q
);

DFF_inst_TMS : DFF
port map (
   clk => tms_clk,
   reset => tms_rst,
   d => tms_d,
   q => tms_q
);

DFF_inst_TDI : DFF
port map (
   clk => tdi_clk,
   reset => tdi_rst,
   d => tdi_d,
   q => tdi_q
);

DFF_inst_TDO : DFF
port map (
   clk => tdo_clk,
   reset => tdo_rst,
   d => tdo_d,
   q => tdo_q
);

-- Instantiate the FIFO OUT
U0 : fifo_generator_v9_3
port map (
    rst => rst_o,
    wr_clk => clk_wr_o,
    rd_clk => clk_rd_o,
    din => data_in_o,
    wr_en => write_en_o,
    rd_en => read_en_o,
    dout => data_out_o,
    full => full_o,
    empty => empty_o
);
-- Instantiate the FIFO IN
U1 : fifo_generator_v9_3
port map (
    rst => rst_i,
    wr_clk => clk_wr_i,
    rd_clk => clk_rd_i,
    din => data_in_i,
    wr_en => write_en_i,
    rd_en => read_en_i,
    dout => data_out_i,
    full => full_i,
    empty => empty_i
);

--------------- clock dividers
clk_div_in <= IFCLK_I;          -- 48Mhz
clk_div2_in <= clk_div_out;     -- 24Mhz

--------------- DFFs
pa2_dff_clk <= IFCLK_I;
trst_clk <= IFCLK_I;
tms_clk <= IFCLK_I;
tdi_clk <= IFCLK_I;
tdo_clk <= IFCLK_I;

--------------- FIFOs
clk_wr_o <= IFCLK_I;
clk_rd_o <= clk_div2_out;
clk_wr_i <= clk_div2_out;
clk_rd_i <= IFCLK_I;

--------------------------- GPIF ready :
GRDY1_I <= '1';

-------------------------------PA2 DFF :
pa2_dff_rst <= '0';
pa2_dff_d <= PA2_I;

-------------------- FX2<->Fifo Enable pins :
write_en_o <= not(wr_o) and not(GCTL0_I);
read_en_i <= not(rd_i) and not(GCTL0_I);

---------------- FX2->Fifo Data :
data_in_o <= GD_IO;

------------ FIFO_OUT->Devkit :
SO_TRST_O <= trst_q;
trst_d <= data_out_o(4);
SO_TMS_O <= tms_q;
tms_d <= data_out_o(3);
SO_TDI_O <= tdi_q;
tdi_d <= data_out_o(1);
------------
SO_TCK_O <= data_out_o(0);

-------------------- FIFO_OUT->FIFO_IN :
--data_in_i <= data_out_o;

-------------------- FIFO_IN<-Devkit :
data_in_i(0) <= '0';
data_in_i(1) <= '0';
data_in_i(2) <= tdo_q;
tdo_d <= not SI_TDO_I;
data_in_i(3) <= '0';
data_in_i(4) <= '0';
data_in_i(5) <= '0';
data_in_i(6) <= '0';
data_in_i(7) <= '0';

-------------------- FX2<-FIFO_IN :
GD_IO <= data_out_i when (state = READ_IN) else "ZZZZZZZZ";

state1_debug <= '1' when state = READ_IN else '0';
state2_debug <= '1' when state2 = WRITE_IN else '0';

--Points de test:
FTP_O(0) <= IFCLK_I;
FTP_O(1) <= GCTL0_I;
FTP_O(2) <= GD_IO(0);
FTP_O(3) <= GD_IO(1);
FTP_O(4) <= JPW_I;
FTP_O(5) <= PA2_I;
FTP_O(6) <= empty_o;
FTP_O(7) <= not SI_TDO_I;

process(pa2_dff_d, pa2_dff_q)
begin
    if pa2_dff_d = '0' and pa2_dff_q = '1' then
        reset_mae <= '1';   -- Reset State Machine
        reset_mae2 <= '1';   -- Reset State Machine
        rst_o <= '1';   -- Reset OUT
        rst_i <= '1';   -- Reset IN
        reset_clk_div <= '1';
        reset_clk_div2 <= '1';
        trst_rst <= '1';
        tms_rst <= '1';
        tdi_rst <= '1';
        tdo_rst <= '1';
    else
        reset_mae <= '0';   -- No Reset State Machine
        reset_mae2 <= '0';   -- Reset State Machine
        rst_o <= '0';   -- No Reset OUT
        rst_i <= '0';   -- No Reset IN
        reset_clk_div <= '0';
        reset_clk_div2 <= '0';
        trst_rst <= '0';
        tms_rst <= '0';
        tdi_rst <= '0';
        tdo_rst <= '0';
    end if;
end process;

process(clk_div2_out, reset_mae2)
begin
    if reset_mae2 = '1' then
        state2 <= IDLE;
    elsif rising_edge(clk_div2_out) then
        case state2 is
            when IDLE =>
                read_en_o <= '0';   -- Disable read OUT
                write_en_i <= '0';   -- Disable write IN
                transit2 <= '1';
                if transit1 = '0' and PA2_I = '0' then
                    state2 <= WRITE_IN;
				else
					state2 <= IDLE;
                end if;

            when WRITE_IN =>
                read_en_o <= '1';    -- Enable read OUT
                write_en_i <= '1';    -- Enable write IN
                if PA2_I = '1' then
                    state2 <= DELAY;  -- Change state to DELAY
                else
                    state2 <= WRITE_IN;  -- Stay in WRITE_IN state
                end if;

            when DELAY =>
                transit2 <= '0'; -- Enable READ IN
                if empty_o = '1' then
						  read_en_o <= '0';    -- Disable read OUT
						  write_en_i <= '0';    -- Disable write IN
                    state2 <= IDLE; -- Change state to IDLE
                else
                    state2 <= DELAY;  -- Stay in READ_IN state
                end if;

            when others =>
                state2 <= IDLE;
        end case;
    end if;
end process;

process(IFCLK_I, reset_mae)
begin
    if reset_mae = '1' then
        state <= IDLE;
    elsif rising_edge(IFCLK_I) then
        case state is
            when IDLE =>
                wr_o <= '1';   -- Disable write OUT
                rd_i <= '1';    -- Disable read IN
                transit1 <= '1';
                if PA2_I = '0' then
                    state <= WRITE_OUT;    -- Change state to RESET
                else
                    state <= IDLE; -- Stay in IDLE state
                end if;

            when WRITE_OUT =>
                wr_o <= '0';   -- Enable write OUT
                if empty_o = '0' then
                    transit1 <= '0'; -- Enable Rd OUT & Wr IN
                    state <= DELAY; -- Change state to DELAY
                else
                    state <= WRITE_OUT;    -- Stay in WRITE_OUT state
                end if;

            when DELAY =>
                if transit2 = '0' then
                    wr_o <= '1';   -- Disable write OUT
                    state <= READ_IN;
                else
                    state <= DELAY;
                end if;

            when READ_IN =>
                rd_i <= '0';    -- Enable read IN
                if empty_i = '1' then
                    rd_i <= '1';    -- Enable read IN
                    state <= IDLE; -- Change state to IDLE
                else
                    state <= READ_IN;  -- Stay in READ_IN state
                end if;

            when others =>
                state <= IDLE;
        end case;
    end if;
end process;

-- OUT signals direction
-- TRST, TMS, TCK and TDI : out
ST_0_O <= '0';
-- TDO : in
ST_1_O <= '1';
-- SRST : out
ST_2_O <= '1';
SO_SRST_O <= '0';
-- MOD : in
ST_3_O <= '1';

end A_angie_bitstream;
