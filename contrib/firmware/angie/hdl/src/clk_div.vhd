library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;

entity clk_div is
    Port (
        clk_in : in  std_logic;
        reset : in std_logic;
        clk_out : out std_logic
    );
end clk_div;

architecture behavioral of clk_div is
    -- Division factor N = 4, so we need a 2-bit counter (2^2 = 4)
--    signal counter : unsigned(1 downto 0) := (others => '0');
    signal tmp : std_logic;
begin
    process(clk_in, reset)
    begin
        if reset = '1' then
--            counter <= (others => '0');
            tmp <= '0';
        elsif rising_edge(clk_in) then
--            if counter = (2**2 - 1) then
--                counter <= (others => '0');
            tmp <= NOT tmp; -- Toggle the output clock
--            else
--                counter <= counter + 1;
--            end if;
        end if;
    end process;
    clk_out <= tmp;
end behavioral;