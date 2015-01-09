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
use IEEE.STD_LOGIC_ARITH.ALL;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;


entity spi_protocol is
	 Generic (
				sync_time_size : POSITIVE := 16
	 );
    Port ( 
				-- System clock
				CLK : in  STD_LOGIC;
				
				-- System reset signal
				RESET : in STD_LOGIC;
				
				-- SPI Interface
				MOSI : in STD_LOGIC;
				MISO : inout STD_LOGIC;
				SCK : in STD_LOGIC;
				CS : in STD_LOGIC;
				
				-- Time reference
				SYNC_TIME : in STD_LOGIC_VECTOR(sync_time_size-1 downto 0);
				
				-- BOARD IO
				OUTPUT : out STD_LOGIC_VECTOR(15 DOWNTO 0);
				INPUT  : in STD_LOGIC_VECTOR(15 DOWNTO 0)

				);
end spi_protocol;

architecture Behavioral of spi_protocol is

-- Define the maximum size (in bits) of the data in a command
constant MAX_DATA_SIZE : integer := 64;


-- Instanciate the SPI Slave device
component spi_slave_gpio
	 Generic (
		byte_size : positive := 8
		);
    Port ( 
				-- System clock
				CLK : in  STD_LOGIC;
				
				-- SPI Interface
				MOSI : in STD_LOGIC;
				MISO : inout STD_LOGIC;
				SCK : in STD_LOGIC;
				CS : in STD_LOGIC;
				
				-- Parallel interface
				BYTE_RX : out STD_LOGIC_VECTOR(byte_size-1 downto 0);
				BYTE_TX : in STD_LOGIC_VECTOR(byte_size-1 downto 0);
				BYTE_RX_COUNT : out STD_LOGIC_VECTOR(7 downto 0);
				BYTE_RX_READY_PULSE : out STD_LOGIC
				);
end component;

component specific_protocol
	 Generic (
				sync_time_size : POSITIVE := 16
	 );
    Port ( 
				-- System clock
				CLK : in  STD_LOGIC;
				
				-- System reset signal
				RESET : in STD_LOGIC;
				
				-- Time reference
				SYNC_TIME : in STD_LOGIC_VECTOR(sync_time_size-1 downto 0);
				
				-- Protocol interface
				COMMAND : in STD_LOGIC_VECTOR(7 downto 0);
				COMMAND_READY : in STD_LOGIC;
				
				BYTE_RX : in STD_LOGIC_VECTOR(7 downto 0);
				BYTE_TX : out STD_LOGIC_VECTOR(7 downto 0);
				BYTE_RX_COUNT : in STD_LOGIC_VECTOR(7 downto 0);
				BYTE_RX_READY_PULSE : in STD_LOGIC;
				
				-- BOARD IO
				OUTPUT : out STD_LOGIC_VECTOR(15 DOWNTO 0);
				INPUT  : in STD_LOGIC_VECTOR(15 DOWNTO 0)

				);
end component;


-- SPI parallel interface signals
signal byte_rx_s : STD_LOGIC_VECTOR(7 downto 0);
signal byte_tx_s : STD_LOGIC_VECTOR(7 downto 0) := (others => '0');
signal byte_rx_count_s : STD_LOGIC_VECTOR(7 downto 0);
signal byte_rx_ready_pulse_s : STD_LOGIC;
signal byte_tx_specific_protocol_s : STD_LOGIC_VECTOR(7 downto 0) := (others => '0');

TYPE STATE_TYPE IS (idle,command, data);
SIGNAL state_s   : STATE_TYPE := idle;

signal byte_count_s : UNSIGNED(7 downto 0); 	-- byte counter

-- Protocol parsing variable
signal card_state_s : STD_LOGIC_VECTOR( 7 downto 0) := (others => '0'); -- card state indicator (to be defined)
signal command_s : STD_LOGIC_VECTOR(7 downto 0);
signal command_ready_s : STD_LOGIC;

-- Chip select clocking signal to detect edge
signal cs_d_s : STD_LOGIC_VECTOR(2 downto 0);
signal cs_s : STD_LOGIC; -- buffered chip select signal
signal cs_falling_edge_s : STD_LOGIC; 
signal cs_raising_edge_s : STD_LOGIC; 

begin

U20 : spi_slave_gpio
	 Generic map(
		byte_size => 8
		)
    Port map( 
		-- System clock
		CLK => CLK,
		
		-- SPI Interface
		MOSI => MOSI,
		MISO => MISO,
		SCK => SCK,
		CS => CS,
		
		-- Parallel interface
		BYTE_RX => byte_rx_s,
		BYTE_TX => byte_tx_s,
		BYTE_RX_COUNT => byte_rx_count_s,
		BYTE_RX_READY_PULSE => byte_rx_ready_pulse_s
		);

U21 : specific_protocol
	 Generic map(
				sync_time_size => sync_time_size
	 )
    Port map( 
				-- System clock
				CLK => CLK,
				
				-- System reset signal
				RESET => RESET,
				
				-- Time reference
				SYNC_TIME => SYNC_TIME,
				
				-- Protocol interface
				COMMAND => command_s,
				COMMAND_READY => command_ready_s,
				
				BYTE_RX => byte_rx_s,
				BYTE_TX => byte_tx_specific_protocol_s,
				BYTE_RX_COUNT => byte_rx_count_s,
				BYTE_RX_READY_PULSE => byte_rx_ready_pulse_s,
				
				-- BOARD IO
				OUTPUT => OUTPUT,
				INPUT  => INPUT
				);

-- Permanent assignation
cs_s <= cs_d_s(2); -- chip select signal buffered
card_state_s <= "01001000"; -- state is 48 by default (number of the universe : see ref = hitchhiker guide to the galaxy)

process(CLK)
begin

	-- Update chip select on rising edge of the system clock only
	if(rising_edge(CLK) ) then
		
			-- Reset signal
			if(RESET = '1') then
				state_s <= idle;
			else
					
				-- Chip select buffering
				cs_d_s(0) <= CS;
				cs_d_s(1) <= cs_d_s(0);
				cs_d_s(2) <= cs_d_s(1);
				
				-- Chip select falling edge detect
				if(cs_d_s(2) = '1' and cs_d_s(1) = '0') then
					cs_falling_edge_s <= '1';
				else
					cs_falling_edge_s <= '0';
				end if;

				-- Chip select raising edge detect
				if(cs_d_s(2) = '0' and cs_d_s(1) = '1') then
					cs_raising_edge_s <= '1';
				else
					cs_raising_edge_s <= '0';
				end if;

				-- start the state machine when chip select goes down
				if(cs_falling_edge_s = '1') then
					state_s <= command;
				end if;
						
				-- run the state machine only if the CS is low
				if(cs_s = '0') then
					CASE state_s IS
						WHEN idle=>
							byte_tx_s <= (others => '0');
							command_ready_s <= '0';
						WHEN command=>
							-- Send the state of the card while the CPU send the command byte
							byte_tx_s <= card_state_s;
							
							-- on the receive of the first byte
							if(byte_rx_ready_pulse_s = '1') then
								command_s <= byte_rx_s; -- latch the command
								command_ready_s <= '1';
								state_s <= data;
							end if;
							
						WHEN data=>
							-- Assign the data from the specific protocol block now
							byte_tx_s <= byte_tx_specific_protocol_s;
							
					END CASE;
				else -- if(cs_s = '0') then
					-- When the CS goes up, place the state machine to idle position
					state_s <= idle;
					-- Clear the command ready flag
					command_ready_s <= '0';
					
				end if; -- if(cs_s = '0') then
			end if; -- if(RESET = '1') then
	end if; -- if(rising_edge(CLK) ) then

end process;
end Behavioral;

