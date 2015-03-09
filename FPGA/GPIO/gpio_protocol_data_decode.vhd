----------------------------------------------------------------------------------
-- Company:
-- Engineer:
--
-- Create Date:    20:20:05 11/03/2014
-- Design Name:
-- Module Name:    gpio_protocol_data_decode - Behavioral
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

entity gpio_protocol_data_decode is
    Port (
				-- System clock
				CLK 						: in		STD_LOGIC;

				-- System reset signal
				RESET 					: in 		STD_LOGIC;

				-- Time reference
				SYNC_TIME 				: in 		STD_LOGIC_VECTOR(31 downto 0);

				-- Protocol interface
				COMMAND 					: in 		STD_LOGIC_VECTOR(7 downto 0);
				COMMAND_READY 			: in 		STD_LOGIC;

				FORCEZ						: out STD_LOGIC;

				BYTE_RX 					: in 		STD_LOGIC_VECTOR(7 downto 0);
				BYTE_TX 					: out 	STD_LOGIC_VECTOR(7 downto 0);
				BYTE_TX_IS_STATUS : out STD_LOGIC;
				BYTE_RX_COUNT 			: in 		STD_LOGIC_VECTOR(7 downto 0);
				BYTE_RX_READY_PULSE 	: in 		STD_LOGIC;

				-- Time feed-back pulse (named as the fault signal in the system)
				FAULT						: inout  STD_LOGIC;

				-- Command status indicator
				VALID_COMMAND_RX 		: out 	STD_LOGIC; -- High pulse at each time a valid command has been parsed
				INCOMPLETE_COMMAND_RX: out 	STD_LOGIC; -- High pulse at each time a incompleted command has been received
				INVALID_COMMAND_RX	: out 	STD_LOGIC; -- High pulse at each time a invalid command has been parsed

				-- User led control
				LED1						: out 	STD_LOGIC;
				LED2						: out 	STD_LOGIC;
				LED3						: out 	STD_LOGIC;

				-- BOARD IO

				OUTPUT					: out 	STD_LOGIC_VECTOR(15 downto 0);
				INPUT						: in 		STD_LOGIC_VECTOR(15 downto 0);

				-- SPARE IO
				SPARE1					: out  	STD_LOGIC;
				SPARE2					: out  	STD_LOGIC
				);
end gpio_protocol_data_decode;

architecture Behavioral of gpio_protocol_data_decode is

component fifo
	Generic(
			ADDR_W	: integer	:= 4;					-- address width in bits
			DATA_W 	: integer	:= 24; 				-- data width in bits
			BUFF_L	: integer 	:=16;					-- buffer length must be less than address space as in  BUFF_L <or= 2^(ADDR_W)-1
			ALMST_F	: integer 	:= 3;					-- fifo flag for almost full regs away from empty fifo
			ALMST_E	: integer	:= 3						-- fifo regs away from empty fifo
			);
	Port (
			clk 					: in std_logic;
			n_reset 				: in std_logic;
			rd_en 				: in std_logic; 		-- read enable
			wr_en					: in std_logic; 		-- write enable
			data_in 				: in std_logic_vector(DATA_W- 1 downto 0);
			data_out				: out std_logic_vector(DATA_W- 1 downto 0);
			data_count			: out std_logic_vector(ADDR_W downto 0);
			empty 				: out std_logic;
			full					: out std_logic;
			almst_empty 		: out std_logic;
			almst_full 			: out std_logic;
			err					: out std_logic
);
end component;

component output_block
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
end component;

-- Board informations (for the ID register)
constant CARD_ID : integer := 2; -- (0=Invalid, 1=AIO, 2=GPIO, 3=FSK-COM)
constant CARD_MAJOR_VERSION : integer := 1;
constant CARD_MINOR_VERSION : integer := 0;

-- Internal signals

signal valid_command_completed_s : STD_LOGIC;		-- Goes high at end of a valid command
signal invalid_command_detected_s : STD_LOGIC;		-- Goes high at end of a invalid command

signal valid_command_rx_s : STD_LOGIC;					-- valid command pulse
signal invalid_command_rx_s : STD_LOGIC;				-- invalid command pulse
signal incomplete_command_rx_s : STD_LOGIC;			-- incomplete command pulse

signal command_ready_d_s : STD_LOGIC; -- Delay of command ready to detect rising edge for latch of inputs

signal sync_time_read_latch_s : STD_LOGIC_VECTOR(31 downto 0); -- to read it in the spi protocol

signal gpio_config_register_s : STD_LOGIC_VECTOR(7 downto 0); -- Configuration register for the AIO card
-- Bit 0 : Unused
-- Bit 1 : Unused
-- Bit 2 : Unused
-- Bit 3 : Unused
-- Bit 4 : Unused
-- Bit 5 : Unused
-- Bit 6 : Unused
-- Bit 7 : Unused

signal gpio_led_control_register_s : STD_LOGIC_VECTOR(7 downto 0); -- Led control register for the AIO card
-- Bit 0 : LED1 on FPGA
-- Bit 1 : LED2 on FPGA
-- Bit 2 : LED3 on FPGA
-- Bit 3 to 7 : Unused

-- PG1 fifo signals
signal pg_fifo_rd_en_s : STD_LOGIC;
signal pg_fifo_wr_en_s : STD_LOGIC;
signal pg_fifo_data_in_s : std_logic_vector(97 downto 0); -- pg select+  time stamp + frequency word + offset word
signal pg_fifo_data_out_s : std_logic_vector(97 downto 0);
signal pg_fifo_empty_flag_s : STD_LOGIC;
signal pg_fifo_full_flag_s : STD_LOGIC;

signal pg_fifo_out_ready_s : STD_LOGIC;

signal pg_fifo_full_flag_latch_s : STD_LOGIC;
--
signal pg_write_fifo_register_s : STD_LOGIC_VECTOR(97 DOWNTO 0); --data received from spi
signal pg_write_fifo_register_ready_s : STD_LOGIC;
--
-- PG1 fifo out state machine
TYPE PG_STATE_TYPE IS (idle,extract,time_and_busy_check,send_complete);
SIGNAL pg_state_s   : PG_STATE_TYPE := idle;

-- GPIO registers
signal gpio_input_data_register_s : STD_LOGIC_VECTOR(15 downto 0); 	-- Data register for GPIO input
signal gpio_output_data_register_s : STD_LOGIC_VECTOR(15 downto 0);  -- Data register for GPIO output

-- GPIO output mapping registers
signal gpio_output_config_register1_s : STD_LOGIC_VECTOR(15 downto 0); -- Output config register
--signal gpio_output_config_register2_s : STD_LOGIC_VECTOR(15 downto 0); -- Output config register
--signal gpio_output_config_register3_s : STD_LOGIC_VECTOR(15 downto 0); -- Output config register
--signal gpio_output_config_register4_s : STD_LOGIC_VECTOR(15 downto 0); -- Output config register


signal pg_pattern_register_select_s : STD_LOGIC_VECTOR(1 downto 0);

-- Pattern generator 1 signals
signal pg1_pattern_load_en_s : STD_LOGIC;
signal pg1_pattern_s: STD_LOGIC_VECTOR(7 downto 0);
signal pg1_phase_accumulator_load_en_s : STD_LOGIC;
signal pg1_phase_accumulator_s: STD_LOGIC_VECTOR(31 downto 0);
signal pg1_offset_load_en_s : STD_LOGIC;
signal pg1_offset_s: STD_LOGIC_VECTOR(31 downto 0);

-- Pattern generator 2 signals
signal pg2_pattern_load_en_s : STD_LOGIC;
signal pg2_pattern_s: STD_LOGIC_VECTOR(7 downto 0);
signal pg2_phase_accumulator_load_en_s : STD_LOGIC;
signal pg2_phase_accumulator_s: STD_LOGIC_VECTOR(31 downto 0);
signal pg2_offset_load_en_s : STD_LOGIC;
signal pg2_offset_s: STD_LOGIC_VECTOR(31 downto 0);

-- Pattern generator 3 signals
signal pg3_pattern_load_en_s : STD_LOGIC;
signal pg3_pattern_s: STD_LOGIC_VECTOR(7 downto 0);
signal pg3_phase_accumulator_load_en_s : STD_LOGIC;
signal pg3_phase_accumulator_s: STD_LOGIC_VECTOR(31 downto 0);
signal pg3_offset_load_en_s : STD_LOGIC;
signal pg3_offset_s: STD_LOGIC_VECTOR(31 downto 0);

-- GPIO output mux select
signal out1_select_s : STD_LOGIC_VECTOR(1 downto 0);
signal out2_select_s : STD_LOGIC_VECTOR(1 downto 0);
signal out3_select_s : STD_LOGIC_VECTOR(1 downto 0);
signal out4_select_s : STD_LOGIC_VECTOR(1 downto 0);
signal out5_select_s : STD_LOGIC_VECTOR(1 downto 0);
signal out6_select_s : STD_LOGIC_VECTOR(1 downto 0);
--signal out7_select_s : STD_LOGIC_VECTOR(3 downto 0);
--signal out8_select_s : STD_LOGIC_VECTOR(3 downto 0);
--signal out9_select_s : STD_LOGIC_VECTOR(3 downto 0);
--signal out10_select_s : STD_LOGIC_VECTOR(3 downto 0);
--signal out11_select_s : STD_LOGIC_VECTOR(3 downto 0);
--signal out12_select_s : STD_LOGIC_VECTOR(3 downto 0);
--signal out13_select_s : STD_LOGIC_VECTOR(3 downto 0);
--signal out14_select_s : STD_LOGIC_VECTOR(3 downto 0);
--signal out15_select_s : STD_LOGIC_VECTOR(3 downto 0);
--signal out16_select_s : STD_LOGIC_VECTOR(3 downto 0);

-- Digital output value
signal digital_output_data_s : STD_LOGIC_VECTOR(15 downto 0);

--signal output_s : STD_LOGIC_VECTOR(15 downto 0);

begin

-- Output mux register mapping
out1_select_s <= gpio_output_config_register1_s(1 downto 0);
out2_select_s <= gpio_output_config_register1_s(3 downto 2);
out3_select_s <= gpio_output_config_register1_s(5 downto 4);
out4_select_s <= gpio_output_config_register1_s(7 downto 6);
out5_select_s <= gpio_output_config_register1_s(9 downto 8);
out6_select_s <= gpio_output_config_register1_s(11 downto 10);
--out7_select_s <= gpio_output_config_register2_s(11 downto 8);
--out8_select_s <= gpio_output_config_register2_s(15 downto 12);
--out9_select_s <= gpio_output_config_register3_s(3 downto 0);
--out10_select_s <= gpio_output_config_register3_s(7 downto 4);
--out11_select_s <= gpio_output_config_register3_s(11 downto 8);
--out12_select_s <= gpio_output_config_register3_s(15 downto 12);
--out13_select_s <= gpio_output_config_register4_s(3 downto 0);
--out14_select_s <= gpio_output_config_register4_s(7 downto 4);
--out15_select_s <= gpio_output_config_register4_s(11 downto 8);
--out16_select_s <= gpio_output_config_register4_s(15 downto 12);

PG_FIFO : fifo
	Generic map(
			ADDR_W	=> 6,					-- address width in bits
			DATA_W 	=> 98, 				-- data width in bits
			BUFF_L	=> 16,				-- buffer length must be less than address space as in  BUFF_L <or= 2^(ADDR_W)-1
			ALMST_F	=> 3,					-- fifo flag for almost full regs away from empty fifo
			ALMST_E	=> 3						-- fifo regs away from empty fifo
			)
	Port map(
			clk 					=> CLK,
			n_reset 				=> not RESET,
			rd_en 				=> pg_fifo_rd_en_s,
			wr_en					=> pg_fifo_wr_en_s,
			data_in 				=> pg_fifo_data_in_s,
			data_out				=> pg_fifo_data_out_s,
			data_count			=> open,
			empty 				=> pg_fifo_empty_flag_s,
			full					=> pg_fifo_full_flag_s,
			almst_empty 		=> open,
			almst_full 			=> open,
			err					=> open
);

U1 : output_block
    Port map(
				-- System clock
				CLK									=> CLK,

				-- Output mapping
				OUT1_SELECT 						=> out1_select_s,
				OUT2_SELECT 						=> out2_select_s,
				OUT3_SELECT 						=> out3_select_s,
				OUT4_SELECT 						=> out4_select_s,
				OUT5_SELECT 						=> out5_select_s,
				OUT6_SELECT 						=> out6_select_s,
				OUT7_SELECT 						=> "00",
				OUT8_SELECT 						=> "00",
				OUT9_SELECT 						=> "00",
				OUT10_SELECT 						=> "00",
				OUT11_SELECT 						=> "00",
				OUT12_SELECT 						=> "00",
				OUT13_SELECT 						=> "00",
				OUT14_SELECT 						=> "00",
				OUT15_SELECT 						=> "00",
				OUT16_SELECT 						=> "00",

				-- Digital output
				DIGITAL_OUTPUT_DATA 				=> digital_output_data_s,

				-- Patern generator 1
				PG1_PATTERN_LOAD_EN 				=> pg1_pattern_load_en_s,
				PG1_PATTERN							=> pg1_pattern_s,

				PG1_PHASE_ACCUMULATOR_LOAD_EN => pg1_phase_accumulator_load_en_s,
				PG1_PHASE_ACCUMULATOR			=> pg1_phase_accumulator_s,

				PG1_OFFSET_LOAD_EN				=> pg1_offset_load_en_s,
				PG1_OFFSET							=> pg1_offset_s,

				-- Patern generator 2
				PG2_PATTERN_LOAD_EN 				=> pg2_pattern_load_en_s,
				PG2_PATTERN							=> pg2_pattern_s,

				PG2_PHASE_ACCUMULATOR_LOAD_EN => pg2_phase_accumulator_load_en_s,
				PG2_PHASE_ACCUMULATOR			=> pg2_phase_accumulator_s,

				PG2_OFFSET_LOAD_EN				=> pg2_offset_load_en_s,
				PG2_OFFSET							=> pg2_offset_s,

				-- Patern generator 3
				PG3_PATTERN_LOAD_EN 				=> pg3_pattern_load_en_s,
				PG3_PATTERN							=> pg3_pattern_s,

				PG3_PHASE_ACCUMULATOR_LOAD_EN => pg3_phase_accumulator_load_en_s,
				PG3_PHASE_ACCUMULATOR			=> pg3_phase_accumulator_s,

				PG3_OFFSET_LOAD_EN				=> pg3_offset_load_en_s,
				PG3_OFFSET							=> pg3_offset_s,

				-- Output
				OUTPUT 								=> OUTPUT
			  );


VALID_COMMAND_RX <= valid_command_rx_s;
INVALID_COMMAND_RX <= invalid_command_rx_s;
INCOMPLETE_COMMAND_RX <= incomplete_command_rx_s;

-- user led display
LED1 <= gpio_led_control_register_s(0);
LED2 <= gpio_led_control_register_s(1);
LED3 <= gpio_led_control_register_s(2);


-- time sync pulse synchro signal (named as fault signal)
FAULT <= 'Z'; -- Leave floating for now...


-- Debug signals
SPARE1 <= '0';
SPARE2 <= '0';


--OUTPUT(15) <= pg_write_fifo_register_ready_s;
--OUTPUT(14) <= pg1_pattern_load_en_s;
--OUTPUT(13) <= pg1_phase_accumulator_load_en_s;
--OUTPUT(12) <= command_ready_d_s;
--OUTPUT(11) <= pg_fifo_wr_en_s;
--OUTPUT(10) <= pg_fifo_rd_en_s;
--OUTPUT(9) <= pg1_offset_load_en_s;

--OUTPUT(8 downto 2) <= (others => '0');

--OUTPUT(1 downto 0) <= output_s(1 downto 0);

process(CLK)
begin

	-- Update chip select on rising edge of the system clock only
		if(rising_edge(CLK) ) then

			if(RESET = '1') then
					pg_write_fifo_register_ready_s <= '0';
					pg1_pattern_load_en_s <= '0';
					pg1_phase_accumulator_load_en_s <= '0';
					pg1_offset_load_en_s <= '0';
					pg2_pattern_load_en_s <= '0';
					pg2_phase_accumulator_load_en_s <= '0';
					pg2_offset_load_en_s <= '0';
					pg3_pattern_load_en_s <= '0';
					pg3_phase_accumulator_load_en_s <= '0';
					pg3_offset_load_en_s <= '0';
					pg_state_s <= idle;
					pg_fifo_wr_en_s <= '0';
					pg_fifo_rd_en_s <= '0';
			else


			-- Delay the command ready signal to have a edge detect
			command_ready_d_s <= COMMAND_READY;


			-- Command validity detect
			-- Detect falling edge of command ready
			if(command_ready_d_s = '1' and COMMAND_READY = '0') then
				-- Detection of valid command and incompleted valid command
				if(valid_command_completed_s = '1') then
					valid_command_completed_s <= '0';
					valid_command_rx_s <= '1';
				elsif(invalid_command_detected_s = '1') then
					invalid_command_detected_s <= '0';
					invalid_command_rx_s <= '1';
				else
					incomplete_command_rx_s <= '1';
				end if;

			else -- Clear the flags rest of the time
				valid_command_rx_s <= '0';
			end if;

			-- Clear the invalid flags on a sucessful command received
			if(valid_command_rx_s = '1') then
				invalid_command_rx_s <= '0';
				incomplete_command_rx_s <= '0';
			end if;


			-- Action to perform on command start (latch inputs, reset stuff...)
			if(command_ready_d_s = '0' and COMMAND_READY = '1') then
				sync_time_read_latch_s <= SYNC_TIME; -- Latch the time for a read in the spi command
				gpio_input_data_register_s <= INPUT;
			end if;

			-- Action to perform on command end (output update ...)
			if(command_ready_d_s = '1' and COMMAND_READY = '0') then
				digital_output_data_s <= gpio_output_data_register_s;
			end if;

			-- Manage the pg write into the fifo
			if(pg_write_fifo_register_ready_s = '1') then
				pg_write_fifo_register_ready_s <= '0';
				pg_fifo_wr_en_s <= '1';
				pg_fifo_data_in_s <= pg_write_fifo_register_s;
			else
				pg_fifo_wr_en_s <= '0';
			end if;

			if(pg_fifo_out_ready_s = '1') then
				-- Check if time to sent the last element from the fifo (element has to be sent now, and spi is not busy)
				if(SYNC_TIME >= pg_fifo_data_out_s(95 downto 64)) then
					-- Jump to next state
					pg_state_s <= send_complete;

					CASE(pg_fifo_data_out_s(97 downto 96)) IS
							WHEN "00" =>
								--Append the data to the right buffer
								pg1_phase_accumulator_s <= pg_fifo_data_out_s(63 downto 32);
								pg1_offset_s <= pg_fifo_data_out_s(31 downto 0);
								-- Rise the load signal
								pg1_offset_load_en_s <= '1';
								pg1_phase_accumulator_load_en_s <= '1';

							WHEN "01" =>
								-- Append the data to the right buffer
								pg2_phase_accumulator_s <= pg_fifo_data_out_s(63 downto 32);
								pg2_offset_s <= pg_fifo_data_out_s(31 downto 0);
								-- Rise the load signal
								pg2_offset_load_en_s <= '1';
								pg2_phase_accumulator_load_en_s <= '1';

							WHEN "10" =>
								-- Append the data to the right buffer
								pg3_phase_accumulator_s <= pg_fifo_data_out_s(63 downto 32);
								pg3_offset_s <= pg_fifo_data_out_s(31 downto 0);
								-- Rise the load signal
								pg3_offset_load_en_s <= '1';
								pg3_phase_accumulator_load_en_s <= '1';
							WHEN OTHERS =>
								-- Nothing to do in this case
						END CASE;
				end if; -- if(SYNC_TIME >= pg_fifo_data_out_s(95 downto 64)) then

			else
				pg1_offset_load_en_s <= '0';
				pg1_phase_accumulator_load_en_s <= '0';
				pg2_offset_load_en_s <= '0';
				pg2_phase_accumulator_load_en_s <= '0';
				pg3_offset_load_en_s <= '0';
				pg3_phase_accumulator_load_en_s <= '0';
			end if;

			-- State machine for Processing of the PG fifo output
			CASE pg_state_s IS
				WHEN idle=> -- Wait here until data to extract form the fifo
					if(pg_fifo_empty_flag_s = '0') then
						pg_state_s <= extract;
					end if;

				WHEN extract=>
					pg_fifo_rd_en_s <= '1';
					pg_state_s <= time_and_busy_check;
				WHEN time_and_busy_check=>
					-- Clear the fifo output eable signal (the data is still valid at it's output, but no more data is pump out)
					pg_fifo_rd_en_s <= '0';
					pg_fifo_out_ready_s <= '1';

				WHEN send_complete=>
					pg_fifo_out_ready_s <= '0';
					pg_state_s <= idle;
			END CASE;

			-- auto clear of the load en signals
			if(pg1_pattern_load_en_s = '1') then
				pg1_pattern_load_en_s <= '0';
			end if;


			if(pg2_pattern_load_en_s = '1') then
				pg2_pattern_load_en_s <= '0';
			end if;


			if(pg3_pattern_load_en_s = '1') then
				pg3_pattern_load_en_s <= '0';
			end if;


				-- Default value: inhibit SPI MISO
				FORCEZ <= '1';
				-- Default value for the signal to send the status register
				BYTE_TX_IS_STATUS <= '0';

				-- if the command is now ready
				if(COMMAND_READY = '1') then
					CASE COMMAND IS

						-- Commands that are received by multiple slaves simultaneously

						-- Read the status registers of the cards (one byte per card)
						-- This card position is 3
						WHEN "10000000" =>
							-- No receive data

							-- Set the reply data at the right time
							if (unsigned(BYTE_RX_COUNT) = 3) then
								-- Authorize output on SPI MISO
								FORCEZ <= '0';
								-- Indicate that the status register has to be sent
								BYTE_TX_IS_STATUS <= '1';
								-- Command check
								if(BYTE_RX_READY_PULSE ='1') then
									valid_command_completed_s <= '1';
								end if;
							end if;

						-- Send data to all cards. Here it corresponds to existing command 0x09 (26 data bytes)
						-- This card position is 2: bytes 8 to 33
						WHEN "10000001" =>
							-- Check if data received flag is set
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 8) then
									pg_write_fifo_register_s(97 downto 96) <= BYTE_RX(1 downto 0);
								elsif(unsigned(BYTE_RX_COUNT) = 9) then
									pg_write_fifo_register_s(95 downto 88) <= BYTE_RX;
									pg_fifo_full_flag_latch_s <= pg_fifo_full_flag_s;
								elsif(unsigned(BYTE_RX_COUNT) = 10) then
									pg_write_fifo_register_s(87 downto 80) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 11) then
									pg_write_fifo_register_s(79 downto 72) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 12) then
									pg_write_fifo_register_s(71 downto 64) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 13) then
									pg_write_fifo_register_s(63 downto 56) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 14) then
									pg_write_fifo_register_s(55 downto 48) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 15) then
									pg_write_fifo_register_s(47 downto 40) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 16) then
									pg_write_fifo_register_s(39 downto 32) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 17) then
									pg_write_fifo_register_s(31 downto 24) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 18) then
									pg_write_fifo_register_s(23 downto 16) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 19) then
									pg_write_fifo_register_s(15 downto 8) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 20) then
									pg_write_fifo_register_s(7 downto 0) <= BYTE_RX;
									if(pg_fifo_full_flag_latch_s = '0') then
										pg_write_fifo_register_ready_s <= '1';
									end if;
								elsif(unsigned(BYTE_RX_COUNT) = 21) then
									pg_write_fifo_register_s(97 downto 96) <= BYTE_RX(1 downto 0);
								elsif(unsigned(BYTE_RX_COUNT) = 22) then
									pg_write_fifo_register_s(95 downto 88) <= BYTE_RX;
									pg_fifo_full_flag_latch_s <= pg_fifo_full_flag_s;
								elsif(unsigned(BYTE_RX_COUNT) = 23) then
									pg_write_fifo_register_s(87 downto 80) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 24) then
									pg_write_fifo_register_s(79 downto 72) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 25) then
									pg_write_fifo_register_s(71 downto 64) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 26) then
									pg_write_fifo_register_s(63 downto 56) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 27) then
									pg_write_fifo_register_s(55 downto 48) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 28) then
									pg_write_fifo_register_s(47 downto 40) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 29) then
									pg_write_fifo_register_s(39 downto 32) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 30) then
									pg_write_fifo_register_s(31 downto 24) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 31) then
									pg_write_fifo_register_s(23 downto 16) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 32) then
									pg_write_fifo_register_s(15 downto 8) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 33) then
									pg_write_fifo_register_s(7 downto 0) <= BYTE_RX;
									if(pg_fifo_full_flag_latch_s = '0') then
										pg_write_fifo_register_ready_s <= '1';
									end if;
								end if;
							end if;

							-- Authorize output on SPI MISO, only when it's this card's turn
							if unsigned(BYTE_RX_COUNT) >= 8 and unsigned(BYTE_RX_COUNT) <= 33 then
								FORCEZ <= '0';
							end if;

							if (unsigned(BYTE_RX_COUNT) = 33) then
								-- Send the full flag state of the fifo in the LSB. If the full flag is set, the write will not occur
								BYTE_TX <= "0000000" & pg_fifo_full_flag_latch_s;
							else
								BYTE_TX <= (others => '0');
							end if;

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 33) then
									valid_command_completed_s <= '1';
								end if;
							end if;

						-- Commands that target only this slave

						WHEN "00000000" => -- Command 0 : Read card ID
							-- No receive data

							-- Authorize output on SPI MISO
							FORCEZ <= '0';

							-- setup the reply data as the byte counter change (not that you have few clock cycle to setup data (before next byte start to output...)
							if(unsigned(BYTE_RX_COUNT) = 1) then
								BYTE_TX <= conv_std_logic_vector(CARD_ID,8);
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								BYTE_TX <= conv_std_logic_vector(CARD_MAJOR_VERSION,8);
							elsif (unsigned(BYTE_RX_COUNT) = 3) then
								BYTE_TX <= conv_std_logic_vector(CARD_MINOR_VERSION,8);
							end if;

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 3) then
									valid_command_completed_s <= '1';
								end if;
							end if;

						WHEN "00000001" => -- Command 1 : Card config register
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 1) then
									gpio_config_register_s <= BYTE_RX;
								end if;
							end if;

							-- Nothing to reply
							BYTE_TX <= (others => '0');

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 1) then
									valid_command_completed_s <= '1';
								end if;
							end if;


						WHEN "00000010" => -- Command 2 : Led control register
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 1) then
									gpio_led_control_register_s <= BYTE_RX;
								end if;
							end if;

							-- Nothing to reply
							BYTE_TX <= (others => '0');

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 1) then
									valid_command_completed_s <= '1';
								end if;
							end if;


						WHEN "00000011" => -- Command 3 : Read sync time counter
							-- No data to read

							-- Authorize output on SPI MISO
							FORCEZ <= '0';

							-- Reply contains the latched sync_time counter
							if (unsigned(BYTE_RX_COUNT) = 1) then
								BYTE_TX <= sync_time_read_latch_s(31 downto 24);
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								BYTE_TX <= sync_time_read_latch_s(23 downto 16);
							elsif (unsigned(BYTE_RX_COUNT) = 3) then
								BYTE_TX <= sync_time_read_latch_s(15 downto 8);
							elsif (unsigned(BYTE_RX_COUNT) = 4) then
								BYTE_TX <= sync_time_read_latch_s(7 downto 0);
							else
								BYTE_TX <= (others => '0');
							end if;

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 4) then
									valid_command_completed_s <= '1';
								end if;
							end if;


						WHEN "00000100" => -- Command 4 : Read inputs GPIO, write outputs GPIO
							-- Check if data received flag is set
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 1) then
									gpio_output_data_register_s(15 downto 8) <= BYTE_RX;
								elsif (unsigned(BYTE_RX_COUNT) = 2) then
									gpio_output_data_register_s(7 downto 0) <= BYTE_RX;
								end if;
							end if;

							-- Authorize output on SPI MISO
							FORCEZ <= '0';

							if (unsigned(BYTE_RX_COUNT) = 1) then
								BYTE_TX <= gpio_input_data_register_s(15 downto 8);
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								BYTE_TX <= gpio_input_data_register_s(7 downto 0);
							else
								BYTE_TX <= (others => '0');
							end if;

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 2) then
									valid_command_completed_s <= '1';
								end if;
							end if;

						WHEN "00000101" => -- Command 5 : Read inputs GPIO

							-- Authorize output on SPI MISO
							FORCEZ <= '0';

							if (unsigned(BYTE_RX_COUNT) = 1) then
								BYTE_TX <= gpio_input_data_register_s(15 downto 8);
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								BYTE_TX <= gpio_input_data_register_s(7 downto 0);
							else
								BYTE_TX <= (others => '0');
							end if;

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 2) then
									valid_command_completed_s <= '1';
								end if;
							end if;


						WHEN "00000110" => -- Command 6 : PG pattern register
							-- Check if data received flag is set
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 1) then
									pg_pattern_register_select_s <= BYTE_RX(1 downto 0);
								elsif(unsigned(BYTE_RX_COUNT) = 2) then
									CASE pg_pattern_register_select_s IS
										WHEN "00" =>
											pg1_pattern_s <= BYTE_RX;
											pg1_pattern_load_en_s <= '1';
										WHEN "01" =>
											pg2_pattern_s <= BYTE_RX;
											pg2_pattern_load_en_s <= '1';
										WHEN "10" =>
											pg3_pattern_s <= BYTE_RX;
											pg3_pattern_load_en_s <= '1';
										WHEN OTHERS =>
											-- Nothing
									END CASE;
								end if;
							end if;

							-- Nothing to reply
							BYTE_TX <= (others => '0');

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 2) then
									valid_command_completed_s <= '1';
								end if;
							end if;


						WHEN "00000111" => -- Command 7 : PG phase accumulator and offset register
							-- Check if data received flag is set
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 1) then
									pg_write_fifo_register_s(97 downto 96) <= BYTE_RX(1 downto 0);
								elsif(unsigned(BYTE_RX_COUNT) = 2) then
									pg_write_fifo_register_s(95 downto 88) <= BYTE_RX;
									pg_fifo_full_flag_latch_s <= pg_fifo_full_flag_s;
								elsif(unsigned(BYTE_RX_COUNT) = 3) then
									pg_write_fifo_register_s(87 downto 80) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 4) then
									pg_write_fifo_register_s(79 downto 72) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 5) then
									pg_write_fifo_register_s(71 downto 64) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 6) then
									pg_write_fifo_register_s(63 downto 56) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 7) then
									pg_write_fifo_register_s(55 downto 48) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 8) then
									pg_write_fifo_register_s(47 downto 40) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 9) then
									pg_write_fifo_register_s(39 downto 32) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 10) then
									pg_write_fifo_register_s(31 downto 24) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 11) then
									pg_write_fifo_register_s(23 downto 16) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 12) then
									pg_write_fifo_register_s(15 downto 8) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 13) then
									pg_write_fifo_register_s(7 downto 0) <= BYTE_RX;
									if(pg_fifo_full_flag_latch_s = '0') then
										pg_write_fifo_register_ready_s <= '1';
									end if;
								end if;
							end if;

							-- Authorize output on SPI MISO
							FORCEZ <= '0';

							if (unsigned(BYTE_RX_COUNT) = 13) then
								-- Send the full flag state of the fifo in the LSB. If the full flag is set, the write will not occur
								BYTE_TX <= "0000000" & pg_fifo_full_flag_latch_s;
							else
								BYTE_TX <= (others => '0');
							end if;

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 13) then
									valid_command_completed_s <= '1';
								end if;
							end if;

						WHEN "00001000" => -- Command 8 : Ouput configuration register
							-- Check if data received flag is set
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 1) then
									gpio_output_config_register1_s(15 downto 8) <= BYTE_RX;
								elsif (unsigned(BYTE_RX_COUNT) = 2) then
									gpio_output_config_register1_s(7 downto 0) <= BYTE_RX;
								end if;
							end if;

							-- Authorize output on SPI MISO
							FORCEZ <= '0';
							-- Always send 0
							BYTE_TX <= (others => '0');

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 2) then
									valid_command_completed_s <= '1';
								end if;
							end if;

						WHEN "00001001" => -- Command 9 : PG phase accumulator and offset register for PG x and y
							-- Check if data received flag is set
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 1) then
									pg_write_fifo_register_s(97 downto 96) <= BYTE_RX(1 downto 0);
								elsif(unsigned(BYTE_RX_COUNT) = 2) then
									pg_write_fifo_register_s(95 downto 88) <= BYTE_RX;
									pg_fifo_full_flag_latch_s <= pg_fifo_full_flag_s;
								elsif(unsigned(BYTE_RX_COUNT) = 3) then
									pg_write_fifo_register_s(87 downto 80) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 4) then
									pg_write_fifo_register_s(79 downto 72) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 5) then
									pg_write_fifo_register_s(71 downto 64) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 6) then
									pg_write_fifo_register_s(63 downto 56) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 7) then
									pg_write_fifo_register_s(55 downto 48) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 8) then
									pg_write_fifo_register_s(47 downto 40) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 9) then
									pg_write_fifo_register_s(39 downto 32) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 10) then
									pg_write_fifo_register_s(31 downto 24) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 11) then
									pg_write_fifo_register_s(23 downto 16) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 12) then
									pg_write_fifo_register_s(15 downto 8) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 13) then
									pg_write_fifo_register_s(7 downto 0) <= BYTE_RX;
									if(pg_fifo_full_flag_latch_s = '0') then
										pg_write_fifo_register_ready_s <= '1';
									end if;
								elsif(unsigned(BYTE_RX_COUNT) = 14) then
									pg_write_fifo_register_s(97 downto 96) <= BYTE_RX(1 downto 0);
								elsif(unsigned(BYTE_RX_COUNT) = 15) then
									pg_write_fifo_register_s(95 downto 88) <= BYTE_RX;
									pg_fifo_full_flag_latch_s <= pg_fifo_full_flag_s;
								elsif(unsigned(BYTE_RX_COUNT) = 16) then
									pg_write_fifo_register_s(87 downto 80) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 17) then
									pg_write_fifo_register_s(79 downto 72) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 18) then
									pg_write_fifo_register_s(71 downto 64) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 19) then
									pg_write_fifo_register_s(63 downto 56) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 20) then
									pg_write_fifo_register_s(55 downto 48) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 21) then
									pg_write_fifo_register_s(47 downto 40) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 22) then
									pg_write_fifo_register_s(39 downto 32) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 23) then
									pg_write_fifo_register_s(31 downto 24) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 24) then
									pg_write_fifo_register_s(23 downto 16) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 25) then
									pg_write_fifo_register_s(15 downto 8) <= BYTE_RX;
								elsif(unsigned(BYTE_RX_COUNT) = 26) then
									pg_write_fifo_register_s(7 downto 0) <= BYTE_RX;
									if(pg_fifo_full_flag_latch_s = '0') then
										pg_write_fifo_register_ready_s <= '1';
									end if;
								end if;
							end if;

							-- Authorize output on SPI MISO
							FORCEZ <= '0';

							if (unsigned(BYTE_RX_COUNT) = 26) then
								-- Send the full flag state of the fifo in the LSB. If the full flag is set, the write will not occur
								BYTE_TX <= "0000000" & pg_fifo_full_flag_latch_s;
							else
								BYTE_TX <= (others => '0');
							end if;

							-- Command check
							if(BYTE_RX_READY_PULSE ='1') then
								if(unsigned(BYTE_RX_COUNT) = 26) then
									valid_command_completed_s <= '1';
								end if;
							end if;

						WHEN OTHERS =>
							BYTE_TX <= (others => '0');
							invalid_command_detected_s <= '1';

					END CASE;

				end if; -- if(COMMAND_READY = '1') then
			end if; -- if(RESET = '1') then
		end if; -- if(rising_edge(CLK) ) then

end process;
end Behavioral;

