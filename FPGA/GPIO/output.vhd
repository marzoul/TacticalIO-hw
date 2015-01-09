----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    20:20:05 11/03/2014 
-- Design Name: 
-- Module Name:    output - Behavioral 
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

entity output_block is
    Port ( 
				-- System clock
				CLK : in  STD_LOGIC;

				-- Output mapping
				OUT1_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT2_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT3_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT4_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT5_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT6_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT7_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT8_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT9_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT10_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT11_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT12_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT13_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT14_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT15_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				OUT16_SELECT : in STD_LOGIC_VECTOR(1 downto 0);
				
				-- Digital output
				DIGITAL_OUTPUT_DATA : in STD_LOGIC_VECTOR(15 downto 0);

				-- Patern generator 1
				PG1_PATTERN_LOAD_EN : in STD_LOGIC;
				PG1_PATTERN : in STD_LOGIC_VECTOR(7 downto 0);

				PG1_PHASE_ACCUMULATOR_LOAD_EN : in STD_LOGIC;
				PG1_PHASE_ACCUMULATOR : in STD_LOGIC_VECTOR(31 downto 0);

				PG1_OFFSET_LOAD_EN : in STD_LOGIC;
				PG1_OFFSET : in STD_LOGIC_VECTOR(31 downto 0);

				-- Patern generator 2
				PG2_PATTERN_LOAD_EN : in STD_LOGIC;
				PG2_PATTERN : in STD_LOGIC_VECTOR(7 downto 0);

				PG2_PHASE_ACCUMULATOR_LOAD_EN : in STD_LOGIC;
				PG2_PHASE_ACCUMULATOR : in STD_LOGIC_VECTOR(31 downto 0);

				PG2_OFFSET_LOAD_EN : in STD_LOGIC;
				PG2_OFFSET : in STD_LOGIC_VECTOR(31 downto 0);

				-- Patern generator 3
				PG3_PATTERN_LOAD_EN : in STD_LOGIC;
				PG3_PATTERN : in STD_LOGIC_VECTOR(7 downto 0);

				PG3_PHASE_ACCUMULATOR_LOAD_EN : in STD_LOGIC;
				PG3_PHASE_ACCUMULATOR : in STD_LOGIC_VECTOR(31 downto 0);

				PG3_OFFSET_LOAD_EN : in STD_LOGIC;
				PG3_OFFSET : in STD_LOGIC_VECTOR(31 downto 0);

				-- Output
				OUTPUT : out STD_LOGIC_VECTOR(15 downto 0)

			  );
end output_block;

architecture Behavioral of output_block is

component pattern_generator is
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
end component;

-- Pattern generator output signals
signal pg1_out1_s : STD_LOGIC;
signal pg1_out2_s : STD_LOGIC;
signal pg2_out1_s : STD_LOGIC;
signal pg2_out2_s : STD_LOGIC;


begin

-- Pattern generator 1
PG1 : pattern_generator
	 Generic map(
				pattern_length  				=> 8,
				phase_accumulator_length  	=> 32, -- number of bit of the frequency register
				phase_accumulator_word_limit => 10995117, --Maximum value for the phase accumulator word (freq limit at 256 kHz)
				offset_register_length  	=> 32	 -- number of bit of the offset register
	 )
    Port map( 
				CLK 								=> CLK,

				PATTERN_LOAD_EN 				=> PG1_PATTERN_LOAD_EN,
				PATTERN							=> PG1_PATTERN,

				PHASE_ACCUMULATOR_LOAD_EN 	=> PG1_PHASE_ACCUMULATOR_LOAD_EN,
				PHASE_ACCUMULATOR				=> PG1_PHASE_ACCUMULATOR,

				OFFSET_LOAD_EN					=> PG1_OFFSET_LOAD_EN,
				OFFSET							=> PG1_OFFSET,
				
				OUT1 								=> pg1_out1_s,
				OUT2								=> pg1_out2_s
			  );

-- Pattern generator 2
PG2 : pattern_generator
	 Generic map(
				pattern_length  				=> 8,
				phase_accumulator_length  	=> 32, -- number of bit of the frequency register
				phase_accumulator_word_limit => 10995117, --Maximum value for the phase accumulator word (freq limit at 256 kHz)
				offset_register_length  	=> 32	 -- number of bit of the offset register
	 )
    Port map( 
				CLK 								=> CLK,

				PATTERN_LOAD_EN 				=> PG2_PATTERN_LOAD_EN,
				PATTERN							=> PG2_PATTERN,

				PHASE_ACCUMULATOR_LOAD_EN 	=> PG2_PHASE_ACCUMULATOR_LOAD_EN,
				PHASE_ACCUMULATOR				=> PG2_PHASE_ACCUMULATOR,

				OFFSET_LOAD_EN					=> PG2_OFFSET_LOAD_EN,
				OFFSET							=> PG2_OFFSET,
				
				OUT1 								=> pg2_out1_s,
				OUT2								=> pg2_out2_s
			  );


process(CLK)

begin
	if(rising_edge(CLK) ) then
	
	-- Output 1 mux
	CASE OUT1_SELECT IS
		WHEN "00" =>
			OUTPUT(0) <= DIGITAL_OUTPUT_DATA(0);
		WHEN "01" =>
			OUTPUT(0) <= pg1_out1_s;
		WHEN "10" =>
			OUTPUT(0) <= '0';
		WHEN "11" =>
			OUTPUT(0) <= '1';
		WHEN OTHERS =>
			OUTPUT(0) <= '0';
	END CASE;

	-- Output 2 mux
	CASE OUT2_SELECT IS
		WHEN "00" =>
			OUTPUT(1) <= DIGITAL_OUTPUT_DATA(1);
		WHEN "01" =>
			OUTPUT(1) <= pg1_out2_s;
		WHEN "10" =>
			OUTPUT(1) <= '0';
		WHEN "11" =>
			OUTPUT(1) <= '1';
		WHEN OTHERS =>
			OUTPUT(1) <= '0';
	END CASE;

	-- Output 3 mux
	CASE OUT3_SELECT IS
		WHEN "00" =>
			OUTPUT(2) <= DIGITAL_OUTPUT_DATA(2);
		WHEN "01" =>
			OUTPUT(2) <= pg2_out1_s;
		WHEN "10" =>
			OUTPUT(2) <= '0';
		WHEN "11" =>
			OUTPUT(2) <= '1';
		WHEN OTHERS =>
			OUTPUT(2) <= '0';
	END CASE;

	-- Output 4 mux
	CASE OUT4_SELECT IS
		WHEN "00" =>
			OUTPUT(3) <= DIGITAL_OUTPUT_DATA(3);
		WHEN "01" =>
			OUTPUT(3) <= pg2_out2_s;
		WHEN "10" =>
			OUTPUT(3) <= '0';
		WHEN "11" =>
			OUTPUT(3) <= '1';
		WHEN OTHERS =>
			OUTPUT(3) <= '0';
	END CASE;

	-- Others output doesn't have feature, so always digial value
	OUTPUT(15 downto 4) <= DIGITAL_OUTPUT_DATA(15 downto 4);
	


	end if;
end process;

end Behavioral;

