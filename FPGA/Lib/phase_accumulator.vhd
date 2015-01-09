----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    20:20:05 11/03/2014 
-- Design Name: 
-- Module Name:    phase_accumulator - Behavioral 
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
use IEEE.STD_LOGIC_ARITH.ALL;

use IEEE.NUMERIC_STD.ALL;
use ieee.std_logic_unsigned.all;



-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity phase_accumulator is
	 Generic (
				phase_accumulator_length  : INTEGER := 32 -- number of bit of the frequency register
	 );
    Port ( 
				CLK : in  STD_LOGIC;

				RESET : in STD_LOGIC;

				PHASE_ACCUMULATOR_LOAD_EN : in STD_LOGIC;
				PHASE_ACCUMULATOR : in STD_LOGIC_VECTOR(phase_accumulator_length-1 downto 0);
				
				CLK_OUT : out  STD_LOGIC;
				PULSE_OUT : out  STD_LOGIC
			  );
end phase_accumulator;

architecture Behavioral of phase_accumulator is

-- config registers
signal phase_accumulator_word_s : STD_LOGIC_VECTOR(phase_accumulator_length-1 downto 0);

-- internal registers
signal phase_accumulator_s : STD_LOGIC_VECTOR(phase_accumulator_length-1 downto 0) := (others => '0');

signal clk_out_s : STD_LOGIC;
signal clk_out_d_s : STD_LOGIC;


begin

CLK_OUT <= clk_out_s;


process(CLK)

begin

if(rising_edge(CLK) ) then
	
	if(PHASE_ACCUMULATOR_LOAD_EN = '1') then
		phase_accumulator_word_s <= PHASE_ACCUMULATOR;
	end if;

	if(RESET = '1') then
		phase_accumulator_s <= (others => '0');
	else
		-- Phase accumulator
		phase_accumulator_s <= phase_accumulator_s + phase_accumulator_word_s;
	end if;

	-- Pattern clock
	clk_out_s <= phase_accumulator_s(phase_accumulator_length-1);
	clk_out_d_s <= clk_out_s;

	if(clk_out_s = '0' and clk_out_d_s ='1') then
		PULSE_OUT <= '1';
	else
		PULSE_OUT <= '0';
	end if;

end if;
end process;

end Behavioral;

