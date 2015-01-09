----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    20:20:05 11/03/2014 
-- Design Name: 
-- Module Name:    time_sync - Behavioral 
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

entity time_sync is
	 Generic (
			sync_time_size : POSITIVE := 16
			);
    Port ( 
			  CLK 			: in  STD_LOGIC;								-- Main clock of the system
           SYNC_CLK 		: in  STD_LOGIC;								-- Counter increment clock tick
           SYNC_RESET 	: in  STD_LOGIC;								-- Counter reset signal
           SYNC_TIME 	: out  STD_LOGIC_VECTOR(sync_time_size-1 downto 0)	-- Counter output
           );								
end time_sync;

architecture Behavioral of time_sync is



signal sync_clk_s_d : STD_LOGIC_VECTOR(7 downto 0);
signal sync_reset_s_d : STD_LOGIC_VECTOR(2 downto 0);
signal sync_time_s : STD_LOGIC_VECTOR(sync_time_size-1 downto 0);

signal count_en : STD_LOGIC;

-- DAC fifo out state machine
TYPE STATE_TYPE IS (wait_high_level,pulse,wait_low_level);
SIGNAL state_s   : STATE_TYPE := wait_high_level;


begin

SYNC_TIME <= sync_time_s;

	process(CLK)
		begin

		if(rising_edge(CLK) ) then

			-- Buff the sync clk signal to sync it with the system clock
			sync_clk_s_d(0) <= SYNC_CLK;
			sync_clk_s_d(1) <= sync_clk_s_d(0);
			sync_clk_s_d(2) <= sync_clk_s_d(1);
			sync_clk_s_d(3) <= sync_clk_s_d(2);
			sync_clk_s_d(4) <= sync_clk_s_d(3);
			sync_clk_s_d(5) <= sync_clk_s_d(4);
			sync_clk_s_d(6) <= sync_clk_s_d(4);
			sync_clk_s_d(7) <= sync_clk_s_d(4);

			-- Buf the async reset signal (to avoid metastability)
			sync_reset_s_d(0) <= SYNC_RESET;
			sync_reset_s_d(1) <= sync_reset_s_d(0);
			sync_reset_s_d(2) <= sync_reset_s_d(1);
			
			-- Edge detect with glitch removal on 4 samples
			CASE state_s IS
				WHEN wait_high_level=> -- Wait here until data to extract form the fifo
					if(sync_clk_s_d(7 downto 2) = "111111") then
						state_s <= pulse;
						count_en <= '1';
					end if;
				WHEN pulse=>	
					state_s <= wait_low_level;
					count_en <= '0';
				WHEN wait_low_level=>
					if(sync_clk_s_d(7 downto 2) = "000000") then
						state_s <= wait_high_level;
					end if;
			END CASE;
			
			-- Reset is more important than the 
			if(sync_reset_s_d(2) = '1') then
				sync_time_s <= (others => '0');
			elsif(count_en = '1') then
				sync_time_s <= sync_time_s + '1';
			end if; -- if(sync_reset_s_d(2) = '1') then
	
		end if; --if(rising_edge(CLK) ) then

	end process;

end Behavioral;

