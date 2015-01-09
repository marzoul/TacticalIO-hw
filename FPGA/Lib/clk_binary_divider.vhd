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

entity clk_binary_divider is
    Generic
			(
			NB_OF_BIT : positive := 16										-- determine the number of bit in the counter
			);
	 Port ( 
			CLK 			: in  STD_LOGIC;								-- Main clock of the system
			OUTPUT			: out  STD_LOGIC								-- Counter output
			);								
end clk_binary_divider;

architecture Behavioral of clk_binary_divider is

signal counter_s : STD_LOGIC_VECTOR(NB_OF_BIT-1 downto 0);

begin

OUTPUT <= counter_s(NB_OF_BIT-1);

	process(CLK)
	begin

		if(rising_edge(CLK) ) then
			counter_s <= counter_s + '1';
		end if;

	end process;

end Behavioral;

