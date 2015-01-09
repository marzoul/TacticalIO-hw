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
			  
				RESET				: in  	STD_LOGIC;
				SYNC_RESET		: in  	STD_LOGIC;
				BUS_CLK			: in 		STD_LOGIC;
				MISO				: inout  STD_LOGIC;
				CS					: in  	STD_LOGIC;
				CSMUX				: in  	STD_LOGIC_VECTOR(2 downto 0);
				SCK				: in  	STD_LOGIC;
				MOSI				: in  	STD_LOGIC;
				SYNC_CLK			: in  	STD_LOGIC;
				FAULT				: inout 	STD_LOGIC;

				-- Board specific IO

				DDS_SCLK 			: out STD_LOGIC;
				DDS_N_CS 			: out STD_LOGIC;
				DDS_FSK_BPSK_HOLD : out STD_LOGIC;
				DDS_OSK 				: out STD_LOGIC;
				DDS_IO_UD_CLK 		: out STD_LOGIC;
				DDS_SDIO 			: out STD_LOGIC;
				DDS_SDO 				: in STD_LOGIC;
				DDS_IO_RESET 		: out STD_LOGIC;
				DDS_MASTER_RESET 	: out STD_LOGIC;
				DDS_FPGA_CLK 		: out STD_LOGIC;

				UART1_N_IRQ 		: in STD_LOGIC;
				UART1_N_CS 			: out STD_LOGIC;
				UART1_MISO 			: in STD_LOGIC;
				UART1_LDOEN 		: out STD_LOGIC;
				UART1_SCK 			: out STD_LOGIC;
				UART1_MOSI 			: out STD_LOGIC;
				UART1_N_RST 		: out STD_LOGIC;
				UART1_CLK			: out STD_LOGIC;

				UART2_N_IRQ 		: in STD_LOGIC;
				UART2_N_CS 			: out STD_LOGIC;
				UART2_MISO 			: in STD_LOGIC;
				UART2_LDOEN 		: out STD_LOGIC;
				UART2_SCK 			: out STD_LOGIC;
				UART2_MOSI 			: out STD_LOGIC;
				UART2_N_RST 		: out STD_LOGIC;
				UART2_CLK			: out STD_LOGIC;

				SPARE1 				: in STD_LOGIC;
				SPARE2 				: in STD_LOGIC;
				SPARE3 				: in STD_LOGIC;
				SPARE4 				: in STD_LOGIC;
				SPARE5 				: in STD_LOGIC;
				SPARE6 				: in STD_LOGIC;
				SPARE7 				: in STD_LOGIC;
				SPARE8 				: in STD_LOGIC
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

component uart_clk
port
 (-- Clock in ports
  CLK_IN1           : in     std_logic;
  -- Clock out ports
  CLK_OUT1          : out    std_logic;
  -- Status and control signals
  RESET             : in     std_logic
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

component fsk_com_spi_protocol
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

				-- Board specific IO

				DDS_SCLK 			: out STD_LOGIC;
				DDS_N_CS 			: out STD_LOGIC;
				DDS_FSK_BPSK_HOLD : out STD_LOGIC;
				DDS_OSK 				: out STD_LOGIC;
				DDS_IO_UD_CLK 		: out STD_LOGIC;
				DDS_SDIO 			: out STD_LOGIC;
				DDS_SDO 				: in STD_LOGIC;
				DDS_IO_RESET 		: out STD_LOGIC;
				DDS_MASTER_RESET 	: out STD_LOGIC;
				DDS_FPGA_CLK 		: out STD_LOGIC;

				UART1_N_IRQ 		: in STD_LOGIC;
				UART1_N_CS 			: out STD_LOGIC;
				UART1_MISO 			: in STD_LOGIC;
				UART1_LDOEN 		: out STD_LOGIC;
				UART1_SCK 			: out STD_LOGIC;
				UART1_MOSI 			: out STD_LOGIC;
				UART1_N_RST 		: out STD_LOGIC;

				UART2_N_IRQ 		: in STD_LOGIC;
				UART2_N_CS 			: out STD_LOGIC;
				UART2_MISO 			: in STD_LOGIC;
				UART2_LDOEN 		: out STD_LOGIC;
				UART2_SCK 			: out STD_LOGIC;
				UART2_MOSI 			: out STD_LOGIC;
				UART2_N_RST 		: out STD_LOGIC;

				SPARE1 				: out STD_LOGIC;
				SPARE2 				: out STD_LOGIC;
				SPARE3 				: out STD_LOGIC;
				SPARE4 				: out STD_LOGIC;
				SPARE5 				: out STD_LOGIC;
				SPARE6 				: out STD_LOGIC;
				SPARE7 				: out STD_LOGIC;
				SPARE8 				: out STD_LOGIC
				);
end component;

signal clk_100mhz_s 	: STD_LOGIC; -- system clock
signal uart_clk_s : STD_LOGIC;

signal sync_time_s 	: STD_LOGIC_VECTOR(31 downto 0);

begin

-- UART clock
UART1_CLK <= uart_clk_s;
UART2_CLK <= uart_clk_s;


CLK0 :  clk_mult_100mhz
port map
 (-- Clock in ports
  CLK_IN1						=> CLK_8MHZ,
  -- Clock out ports
  CLK_OUT1						=> clk_100mhz_s
 );

CLK1 : uart_clk
port map
 (-- Clock in ports
  CLK_IN1           => clk_100mhz_s,
  -- Clock out ports
  CLK_OUT1          => uart_clk_s,
  -- Status and control signals
  RESET             => RESET
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
	
U2 : fsk_com_spi_protocol
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

				DDS_SCLK 			=>DDS_SCLK,
				DDS_N_CS 			=>DDS_N_CS,
				DDS_FSK_BPSK_HOLD =>DDS_FSK_BPSK_HOLD,
				DDS_OSK 				=>DDS_OSK,
				DDS_IO_UD_CLK 		=>DDS_IO_UD_CLK,
				DDS_SDIO 			=>DDS_SDIO,
				DDS_SDO 				=>DDS_SDO,
				DDS_IO_RESET 		=>DDS_IO_RESET,
				DDS_MASTER_RESET 	=>DDS_MASTER_RESET,
				DDS_FPGA_CLK 		=>DDS_FPGA_CLK,

				UART1_N_IRQ 		=>UART1_N_IRQ,
				UART1_N_CS 			=>UART1_N_CS,
				UART1_MISO 			=>UART1_MISO,
				UART1_LDOEN 		=>UART1_LDOEN,
				UART1_SCK 			=>UART1_SCK,
				UART1_MOSI 			=>UART1_MOSI,
				UART1_N_RST 		=>UART1_N_RST,
				
				UART2_N_IRQ 		=>UART2_N_IRQ,
				UART2_N_CS 			=>UART2_N_CS,
				UART2_MISO 			=>UART2_MISO,
				UART2_LDOEN 		=>UART2_LDOEN,
				UART2_SCK 			=>UART2_SCK,
				UART2_MOSI 			=>UART2_MOSI,
				UART2_N_RST 		=>UART2_N_RST,

				SPARE1 				=>open,
				SPARE2 				=>open,
				SPARE3 				=>open,
				SPARE4 				=>open,
				SPARE5 				=>open,
				SPARE6 				=>open,
				SPARE7 				=>open,
				SPARE8 				=>open
				);


end Behavioral;

