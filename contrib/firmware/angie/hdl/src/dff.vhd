library ieee;
use ieee.std_logic_1164.ALL;
use ieee.std_logic_arith.ALL;
use ieee.std_logic_unsigned.ALL;

entity DFF is
port (  clk : in std_logic;
        reset : in std_logic;
        d : in std_logic;
        q : out std_logic);
end DFF;

architecture Behavioral of DFF is
begin
    process(clk, reset)
    begin
        if reset = '1' then
            q <= '1'; -- Reset output to 0
        elsif rising_edge(clk) then
            q <= d; -- Capture D at the rising edge of the clock
        end if;
    end process;
end Behavioral;