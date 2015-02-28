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


entity fsk_com_spi_protocol is
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

				-- Time feed-back pulse (named as the fault signal in the system)
				FAULT				: inout  STD_LOGIC;

				-- User led control
				LED1				: out STD_LOGIC;
				LED2				: out STD_LOGIC;
				LED3				: out STD_LOGIC;

				--------------------
				-- BOARD IO

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
end fsk_com_spi_protocol;

architecture Behavioral of fsk_com_spi_protocol is

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

				-- Force MISO to 'Z'
				FORCEZ : in STD_LOGIC;

				-- Parallel interface
				BYTE_RX : out STD_LOGIC_VECTOR(byte_size-1 downto 0);
				BYTE_TX : in STD_LOGIC_VECTOR(byte_size-1 downto 0);
				BYTE_RX_COUNT : out STD_LOGIC_VECTOR(7 downto 0);
				BYTE_RX_READY_PULSE : out STD_LOGIC
				);
end component;

component fsk_com_protocol_data_decode
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

				-- SPI Interface from cpu
				MOSI : in STD_LOGIC;
				SCK : in STD_LOGIC;
				CS : in STD_LOGIC;

				-- Protocol interface
				COMMAND : in STD_LOGIC_VECTOR(7 downto 0);
				COMMAND_READY : in STD_LOGIC;

				BYTE_RX : in STD_LOGIC_VECTOR(7 downto 0);
				BYTE_TX : out STD_LOGIC_VECTOR(7 downto 0);
				BYTE_RX_COUNT : in STD_LOGIC_VECTOR(7 downto 0);
				BYTE_RX_READY_PULSE : in STD_LOGIC;

				-- MISO source selector
				MISO_SRC_SEL : out STD_LOGIC_VECTOR(1 downto 0);

				-- Time feed-back pulse (named as the fault signal in the system)
				FAULT				: inout  STD_LOGIC;

				-- Command status indicator
				VALID_COMMAND_RX 		: out 	STD_LOGIC; -- High pulse at each time a valid command has been parsed
				INCOMPLETE_COMMAND_RX: out 	STD_LOGIC; -- LOGIC high when a incompleted command has been received
				INVALID_COMMAND_RX	: out 	STD_LOGIC; -- LOGIC high when a invalid command has been received

				-- User led control
				LED1				: out STD_LOGIC;
				LED2				: out STD_LOGIC;
				LED3				: out STD_LOGIC;

				-- BOARD IO

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

-- MISO mux signals
signal miso_src_sel_s : STD_LOGIC_VECTOR(1 downto 0);
signal miso_spi_slave_gpio_s : STD_LOGIC;
signal miso_uart1_s : STD_LOGIC;
signal miso_uart2_s : STD_LOGIC;
signal miso_dds_s : STD_LOGIC;

-- comm check signal
signal valid_command_rx_s : STD_LOGIC;					-- valid command pulse
signal invalid_command_rx_s : STD_LOGIC;				-- invalid command indicator
signal incomplete_command_rx_s : STD_LOGIC;			-- incomplete command indicator


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
signal card_state_s : STD_LOGIC_VECTOR( 7 downto 0); -- card state indicator
-- card_state_s Bit 0 : Constant bit ID : 0
-- card_state_s Bit 1 : Constant bit ID : 1
-- card_state_s Bit 2 : Constant bit ID : 0
-- card_state_s Bit 3 : Constant bit ID : 1
-- card_state_s Bit 4 : Card has been reset
-- card_state_s Bit 5 :	Alive indicator (Toogle at each valid command completed)
-- card_state_s Bit 6 : Last command received was invalid
-- card_state_s Bit 7 : Last command received was truncated

signal command_s : STD_LOGIC_VECTOR(7 downto 0);
signal command_ready_s : STD_LOGIC;

-- Chip select clocking signal to detect edge
signal cs_d_s : STD_LOGIC_VECTOR(2 downto 0);
signal cs_s : STD_LOGIC; -- buffered chip select signal
signal cs_falling_edge_s : STD_LOGIC;
signal cs_raising_edge_s : STD_LOGIC;

signal reset_status_s : STD_LOGIC := '0';
signal alive_indicator_s : STD_LOGIC;

begin

-- Constant bit ID
card_state_s(0) <= '0';
card_state_s(1) <= '1';
card_state_s(2) <= '0';
card_state_s(3) <= '1';
-- Reset status
card_state_s(4) <= reset_status_s;
-- Alive indicator
card_state_s(5) <= alive_indicator_s;
-- Invalid command received
card_state_s(6) <= invalid_command_rx_s;
-- Truncated command received
card_state_s(7) <= incomplete_command_rx_s;

--SPARE1 <= invalid_command_rx_s or incomplete_command_rx_s;

U1 : spi_slave_gpio
	 Generic map(
		byte_size => 8
		)
    Port map(
		-- System clock
		CLK => CLK,

		-- SPI Interface
		MOSI => MOSI,
		MISO => miso_spi_slave_gpio_s,
		SCK => SCK,
		CS => CS,

		-- Force MISO to 'Z'
		FORCEZ => '0',

		-- Parallel interface
		BYTE_RX => byte_rx_s,
		BYTE_TX => byte_tx_s,
		BYTE_RX_COUNT => byte_rx_count_s,
		BYTE_RX_READY_PULSE => byte_rx_ready_pulse_s
		);

U2 : fsk_com_protocol_data_decode
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

				-- SPI Interface from cpu
				MOSI => MOSI,
				SCK => SCK,
				CS => CS,

				-- Protocol interface
				COMMAND => command_s,
				COMMAND_READY => command_ready_s,

				BYTE_RX => byte_rx_s,
				BYTE_TX => byte_tx_specific_protocol_s,
				BYTE_RX_COUNT => byte_rx_count_s,
				BYTE_RX_READY_PULSE => byte_rx_ready_pulse_s,

				-- MISO source selector
				MISO_SRC_SEL => miso_src_sel_s,

				-- Time feed-back pulse (named as the fault signal in the system)
				FAULT			=> FAULT,

				-- Command status indicator
				VALID_COMMAND_RX 		=> valid_command_rx_s,
				INCOMPLETE_COMMAND_RX => incomplete_command_rx_s,
				INVALID_COMMAND_RX	=> invalid_command_rx_s,

				-- User led control
				LED1				=> LED1,
				LED2				=> LED2,
				LED3				=> LED3,

				-- BOARD IO

				DDS_SCLK 			=>DDS_SCLK,
				DDS_N_CS 			=>DDS_N_CS,
				DDS_FSK_BPSK_HOLD =>DDS_FSK_BPSK_HOLD,
				DDS_OSK 				=>DDS_OSK,
				DDS_IO_UD_CLK 		=>DDS_IO_UD_CLK,
				DDS_SDIO 			=>DDS_SDIO,
				DDS_SDO 				=>miso_dds_s,
				DDS_IO_RESET 		=>DDS_IO_RESET,
				DDS_MASTER_RESET 	=>DDS_MASTER_RESET,
				DDS_FPGA_CLK 		=>DDS_FPGA_CLK,

				UART1_N_IRQ 		=>UART1_N_IRQ,
				UART1_N_CS 			=>UART1_N_CS,
				UART1_MISO 			=>miso_uart1_s,
				UART1_LDOEN 		=>UART1_LDOEN,
				UART1_SCK 			=>UART1_SCK,
				UART1_MOSI 			=>UART1_MOSI,
				UART1_N_RST 		=>UART1_N_RST,

				UART2_N_IRQ 		=>UART2_N_IRQ,
				UART2_N_CS 			=>UART2_N_CS,
				UART2_MISO 			=>miso_uart2_s,
				UART2_LDOEN 		=>UART2_LDOEN,
				UART2_SCK 			=>UART2_SCK,
				UART2_MOSI 			=>UART2_MOSI,
				UART2_N_RST 		=>UART2_N_RST,

				SPARE1 				=>SPARE1,
				SPARE2 				=>SPARE2,
				SPARE3 				=>SPARE3,
				SPARE4 				=>SPARE4,
				SPARE5 				=>SPARE5,
				SPARE6 				=>SPARE6,
				SPARE7 				=>SPARE7,
				SPARE8 				=>SPARE8
				);


-- Permanent assignation
cs_s <= cs_d_s(2); -- chip select signal buffered

-- MISO mux
with (CS & miso_src_sel_s) select MISO <=
	miso_spi_slave_gpio_s when "000",
	miso_uart1_s when "001",
	miso_uart2_s when "010",
	miso_dds_s when "011",
	'Z' when others;

-- Plug the miso signals to the physical devices
miso_dds_s <= DDS_SDO;
miso_uart1_s <= UART1_MISO;
miso_uart2_s <= UART2_MISO;


process(CLK)
begin

	-- Update chip select on rising edge of the system clock only
	if(rising_edge(CLK) ) then

			-- Reset signal
			if(RESET = '1') then
				state_s <= idle;
				reset_status_s <= '1';
			else

				-- Alive bit in status register
				if(valid_command_rx_s = '1') then
					-- Tooggle at each valid command received
					alive_indicator_s <= not alive_indicator_s;
				end if;


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

