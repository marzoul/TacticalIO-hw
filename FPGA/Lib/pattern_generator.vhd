----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    20:20:05 11/03/2014 
-- Design Name: 
-- Module Name:    pattern_generator - Behavioral 
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

entity pattern_generator is
	 Generic (
				pattern_length  : INTEGER := 8; -- number of bit in pattern
				phase_accumulator_length  : INTEGER := 32; -- number of bit of the frequency register
				phase_accumulator_word_limit : INTEGER := 10995117; -- Maximum value for the phase accumulator word
				offset_register_length  : INTEGER := 16 -- number of bit of the offset register
				
	 );
    Port ( 
				CLK : in  STD_LOGIC;

				PATTERN_LOAD_EN : in STD_LOGIC;
				PATTERN : in STD_LOGIC_VECTOR(pattern_length-1 downto 0);

				PHASE_ACCUMULATOR_LOAD_EN : in STD_LOGIC;
				PHASE_ACCUMULATOR : in STD_LOGIC_VECTOR(phase_accumulator_length-1 downto 0);

				OFFSET_LOAD_EN : in STD_LOGIC;
				OFFSET : in STD_LOGIC_VECTOR(offset_register_length-1 downto 0);
				
				OUT1 : out  STD_LOGIC;
				OUT2 : out  STD_LOGIC
			  );
end pattern_generator;

architecture Behavioral of pattern_generator is

-- config registers
signal pattern_s : STD_LOGIC_VECTOR(pattern_length-1 downto 0);
signal offset_pattern_s : STD_LOGIC_VECTOR(pattern_length-1 downto 0);
signal phase_accumulator_word_s : STD_LOGIC_VECTOR(phase_accumulator_length-1 downto 0);
signal offset_s : STD_LOGIC_VECTOR(offset_register_length-1 downto 0);

-- internal registers
signal phase_accumulator_s : STD_LOGIC_VECTOR(phase_accumulator_length-1 downto 0) := (others => '0');
signal pattern_clock_s : STD_LOGIC;
signal pattern_clock_d_s : STD_LOGIC;
signal pattern_shift_pulse_s : STD_LOGIC;

signal offset_accumulator_s : STD_LOGIC_VECTOR(phase_accumulator_length-1 downto 0) := (others => '0');
signal offset_pattern_clock_s : STD_LOGIC;
signal offset_pattern_clock_d_s : STD_LOGIC;
signal offset_pattern_shift_pulse_s : STD_LOGIC;


begin

process(CLK)

begin

if(rising_edge(CLK) ) then
	
	-- Load of registers
	
	if(PATTERN_LOAD_EN = '1') then
		pattern_s <= PATTERN;
		offset_pattern_s <= PATTERN;
	end if;
	
	if(PHASE_ACCUMULATOR_LOAD_EN = '1') then
		if(PHASE_ACCUMULATOR > phase_accumulator_word_limit) then
			phase_accumulator_word_s <= conv_std_logic_vector(phase_accumulator_word_limit,32);
		else
			phase_accumulator_word_s <= PHASE_ACCUMULATOR;
		end if;
	end if;

	if(OFFSET_LOAD_EN = '1') then
		offset_s <= OFFSET;
	end if;
	

	-- Phase accumulator
	phase_accumulator_s <= phase_accumulator_s + phase_accumulator_word_s;

	-- Pattern clock
	pattern_clock_s <= phase_accumulator_s(phase_accumulator_length-1);
	pattern_clock_d_s <= pattern_clock_s;

	-- Pattern shift pulse
	if(pattern_clock_s = '1' and pattern_clock_d_s = '0') then
		pattern_shift_pulse_s <= '1';
	else
		pattern_shift_pulse_s <= '0';
	end if;


	-- Offset accumulator
	offset_accumulator_s <= phase_accumulator_s + offset_s;

	-- Pattern clock
	offset_pattern_clock_s <= offset_accumulator_s(phase_accumulator_length-1);
	offset_pattern_clock_d_s <= offset_pattern_clock_s;

	-- Pattern shift pulse
	if(offset_pattern_clock_s = '1' and offset_pattern_clock_d_s = '0') then
		offset_pattern_shift_pulse_s <= '1';
	else
		offset_pattern_shift_pulse_s <= '0';
	end if;




	-- On pattern clock, rotate right the pattern register
	if(PATTERN_LOAD_EN = '0') then -- avoid shifting when load en signal
		if(pattern_shift_pulse_s = '1') then
				pattern_s <= pattern_s(0) & pattern_s(7 downto 1);
		end if;
		
		if(offset_pattern_shift_pulse_s = '1') then
				offset_pattern_s <= offset_pattern_s(0) & offset_pattern_s(7 downto 1);
		end if;
		
		
	end if;
		
	-- update output
	OUT1 <= pattern_s(0);
	
	-- Offset for output 2 genrate
	OUT2 <= offset_pattern_s(0);

end if;
end process;

end Behavioral;

