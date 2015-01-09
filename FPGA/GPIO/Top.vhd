----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    14:13:28 09/24/2014 
-- Design Name: 
-- Module Name:    Top - Behavioral 
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
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity Top is
    Port ( 
				CLK_8MHZ 		: in 		STD_LOGIC;
				BTN_0 			: in  	STD_LOGIC;
				BTN_1 			: in  	STD_LOGIC;
				LED_0 			: out 	STD_LOGIC;
				LED_1 			: out 	STD_LOGIC;
				LED_2 			: out 	STD_LOGIC;
				LED_3 			: out 	STD_LOGIC;

				BUS_CLK			: in 		STD_LOGIC;

				RESET				: in  	STD_LOGIC;
				SYNC_RESET		: in  	STD_LOGIC;

				CS					: in  	STD_LOGIC;
				CSMUX				: in  	STD_LOGIC_VECTOR(2 downto 0);
				MISO				: inout  STD_LOGIC;
				SCK				: in  	STD_LOGIC;
				MOSI				: in  	STD_LOGIC;

				SYNC_CLK			: in  	STD_LOGIC;
				FAULT				: inout 	STD_LOGIC;

				SPARE1			: in  	STD_LOGIC;
				SPARE2			: in  	STD_LOGIC;

				-- Specific IO for card
				OUTPUT			: out STD_LOGIC_VECTOR(15 downto 0); 
				INPUT				: in STD_LOGIC_VECTOR(15 downto 0)
			  );
end Top;

architecture Behavioral of Top is

component clk_mult_100mhz
port
 (-- Clock in ports
  CLK_IN1           			: in     std_logic;
  -- Clock out ports
  CLK_OUT1          			: out    std_logic
 );
 end component;

component clk_binary_divider
    Generic
			(
			NB_OF_BIT 			: positive := 16										-- determine the number of bit in the counter
			);
	 Port ( 
			CLK 					: in  	STD_LOGIC;								-- Main clock of the system
			OUTPUT				: out  	STD_LOGIC								-- Counter output
			);								
end component;

component time_sync
	 Generic (
			sync_time_size 	: POSITIVE := 16
			);
    Port ( 
			  CLK 				: in  	STD_LOGIC;								-- Main clock of the system
           SYNC_CLK 			: in  	STD_LOGIC;								-- Counter increment clock tick
           SYNC_RESET 		: in  	STD_LOGIC;								-- Counter reset signal
           SYNC_TIME 		: out  	STD_LOGIC_VECTOR(sync_time_size-1 downto 0)	-- Counter output
           );								
end component;

component gpio_spi_protocol
	 Generic (
				sync_time_size : POSITIVE := 16
	 );
    Port ( 
				-- System clock
				CLK 				: in  	STD_LOGIC;
				
				-- System reset signal
				RESET 			: in 		STD_LOGIC;
				
				-- SPI Interface
				MOSI 				: in 		STD_LOGIC;
				MISO 				: inout 	STD_LOGIC;
				SCK 				: in		STD_LOGIC;
				CS 				: in 		STD_LOGIC;
				
				-- Time reference
				SYNC_TIME 		: in 		STD_LOGIC_VECTOR(sync_time_size-1 downto 0);
				
				-- Time feed-back pulse (named as the fault signal in the system)
				FAULT				: inout  STD_LOGIC;

				-- User led control
				LED1				: out 	STD_LOGIC;
				LED2				: out 	STD_LOGIC;
				LED3				: out 	STD_LOGIC;

				-- BOARD IO

				OUTPUT			: out 	STD_LOGIC_VECTOR(15 downto 0); 
				INPUT				: in 		STD_LOGIC_VECTOR(15 downto 0);

				-- SPARE IO
				SPARE1			: out  	STD_LOGIC;
				SPARE2			: out  	STD_LOGIC
				);
end component;

signal clk_100mhz_s 	: STD_LOGIC; -- system clock

signal sync_time_s 	: STD_LOGIC_VECTOR(31 downto 0);

begin

CLK0 :  clk_mult_100mhz
port map
 (-- Clock in ports
  CLK_IN1						=> CLK_8MHZ,
  -- Clock out ports
  CLK_OUT1						=> clk_100mhz_s
 );


U0 : clk_binary_divider
    Generic map
			(
			NB_OF_BIT 			=> 25
			)
	 Port map ( 
			CLK 					=> clk_100mhz_s,
			OUTPUT 				=> LED_0
			);				
	
U1 : time_sync
	 Generic map(
			sync_time_size 	=> 32
			)
    Port map( 
			  CLK 				=> clk_100mhz_s,						-- Main clock of the system
           SYNC_CLK 			=> SYNC_CLK,							-- Counter increment clock tick
           SYNC_RESET 		=> SYNC_RESET,							-- Counter reset signal
           SYNC_TIME 		=> sync_time_s							-- Counter output
           );								
	
U2 : gpio_spi_protocol
	 Generic map(
				sync_time_size => 32
	 )
    Port map( 
				-- System clock
				CLK 				=> clk_100mhz_s,
				
				-- System reset signal
				RESET 			=> RESET,
				
				-- SPI Interface
				MOSI 				=> MOSI,
				MISO 				=> MISO,
				SCK 				=> SCK,
				CS 				=> CS,
				
				-- Time reference
				SYNC_TIME 		=> sync_time_s,
				
				-- Time feed-back pulse (named as the fault signal in the system)
				FAULT				=> FAULT,

				-- User led control
				LED1				=> LED_1,
				LED2				=> LED_2,
				LED3				=> LED_3,

				-----------------------------
				-- BOARD IO

				OUTPUT			=> OUTPUT,
				INPUT				=> INPUT,

				-- SPARE IO
				SPARE1			=> open,
				SPARE2			=> open
				);


end Behavioral;

