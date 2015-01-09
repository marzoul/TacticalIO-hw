----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    20:20:05 11/03/2014 
-- Design Name: 
-- Module Name:    cs_dmux - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity cs_dmux is
    Port ( 
			  CS : in  STD_LOGIC;									-- Main chip select
           CSMUX : in  STD_LOGIC_VECTOR (2 downto 0);		-- Chip select muxed selection
           CS_0 : out  STD_LOGIC;								-- Demux out chip select 0
           CS_1 : out  STD_LOGIC;								-- Demux out chip select 1
           CS_2 : out  STD_LOGIC;								-- Demux out chip select 2
           CS_3 : out  STD_LOGIC;								-- Demux out chip select 3
           CS_4 : out  STD_LOGIC;								-- Demux out chip select 4
           CS_5 : out  STD_LOGIC;								-- Demux out chip select 5
           CS_6 : out  STD_LOGIC;								-- Demux out chip select 6
           CS_7 : out  STD_LOGIC);								-- Demux out chip select 7
end cs_dmux;

architecture Behavioral of cs_dmux is

begin

process(CS)

begin

-- Update chip select on rising edge of the system clock only
--if(rising_edge(CLK) ) then
	if(CS = '0') then
		case CSMUX is
			when "000" =>
				CS_0 <= '0';
				CS_1 <= '1';
				CS_2 <= '1';
				CS_3 <= '1';
				CS_4 <= '1';
				CS_5 <= '1';
				CS_6 <= '1';
				CS_7 <= '1';

			when "001" =>
				CS_0 <= '1';
				CS_1 <= '0';
				CS_2 <= '1';
				CS_3 <= '1';
				CS_4 <= '1';
				CS_5 <= '1';
				CS_6 <= '1';
				CS_7 <= '1';
			
			when "010" =>
				CS_0 <= '1';
				CS_1 <= '1';
				CS_2 <= '0';
				CS_3 <= '1';
				CS_4 <= '1';
				CS_5 <= '1';
				CS_6 <= '1';
				CS_7 <= '1';

			when "011" =>
				CS_0 <= '1';
				CS_1 <= '1';
				CS_2 <= '1';
				CS_3 <= '0';
				CS_4 <= '1';
				CS_5 <= '1';
				CS_6 <= '1';
				CS_7 <= '1';

			when "100" =>
				CS_0 <= '1';
				CS_1 <= '1';
				CS_2 <= '1';
				CS_3 <= '1';
				CS_4 <= '0';
				CS_5 <= '1';
				CS_6 <= '1';
				CS_7 <= '1';

			when "101" =>
				CS_0 <= '1';
				CS_1 <= '1';
				CS_2 <= '1';
				CS_3 <= '1';
				CS_4 <= '1';
				CS_5 <= '0';
				CS_6 <= '1';
				CS_7 <= '1';

			when "110" =>
				CS_0 <= '1';
				CS_1 <= '1';
				CS_2 <= '1';
				CS_3 <= '1';
				CS_4 <= '1';
				CS_5 <= '1';
				CS_6 <= '0';
				CS_7 <= '1';

			when "111" =>
				CS_0 <= '1';
				CS_1 <= '1';
				CS_2 <= '1';
				CS_3 <= '1';
				CS_4 <= '1';
				CS_5 <= '1';
				CS_6 <= '1';
				CS_7 <= '0';

			when others => null;
	end case;

	else
				CS_0 <= '1';
				CS_1 <= '1';
				CS_2 <= '1';
				CS_3 <= '1';
				CS_4 <= '1';
				CS_5 <= '1';
				CS_6 <= '1';
				CS_7 <= '1';
	end if;
--end if;
end process;

end Behavioral;

