----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    20:20:05 11/03/2014 
-- Design Name: 
-- Module Name:    counter - Behavioral 
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
use IEEE.NUMERIC_STD.ALL;
use ieee.std_logic_unsigned.all;
-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity counter is
    Port ( 
			  CLK 			: in  STD_LOGIC;								-- Main clock of the system
           RESET	 		: in  STD_LOGIC;								-- Counter increment clock tick
           EN			 	: in  STD_LOGIC;								-- Counter reset signal
           COUNT			: out  STD_LOGIC_VECTOR(32 downto 0)	-- Counter output
           );								
end time_sync;

architecture Behavioral of time_sync is

signal sync_clk_s_d : STD_LOGIC_VECTOR(2 downto 0);
signal sync_time_s : STD_LOGIC_VECTOR(15 downto 0);

begin

SYNC_TIME <= sync_time_s;

	process(CLK)
		begin

		if(rising_edge(CLK) ) then
			
			-- Buff the sync clk signal to sync it with the system clock
			sync_clk_s_d(0) <= SYNC_CLK;
			sync_clk_s_d(1) <= sync_clk_s_d(0);
			sync_clk_s_d(2) <= sync_clk_s_d(1);

			-- detect rising edge on sync clk signal
			if(sync_clk_s_d(2) = '0' and sync_clk_s_d(1) = '1') then
				sync_time_s <= sync_time_s + '1';
			end if;
			
			if(SYNC_RESET = '1') then
				sync_time_s <= (others => '0');
			end if;
			
		end if;

	end process;

end Behavioral;

