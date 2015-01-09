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
				
				BYTE_RX 					: in 		STD_LOGIC_VECTOR(7 downto 0);
				BYTE_TX 					: out 	STD_LOGIC_VECTOR(7 downto 0);
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

-- Board informations (for the ID register)
constant CARD_ID : integer := 1; -- (0=Invalid, 1=AIO, 2=GPIO, 3=FSK-COM)
constant CARD_MAJOR_VERSION : integer := 1;
constant CARD_MINOR_VERSION : integer := 2;

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

-- GPIO fifo signals
--signal dac_fifo_rd_en_s : STD_LOGIC;
--signal dac_fifo_wr_en_s : STD_LOGIC;
--signal dac_fifo_data_in_s : std_logic_vector(55 downto 0); 
--signal dac_fifo_data_out_s : std_logic_vector(55 downto 0); 
--signal dac_fifo_empty_flag_s : STD_LOGIC;
--signal dac_fifo_full_flag_s : STD_LOGIC;
--
--signal dac_fifo_full_flag_latch_s : STD_LOGIC;
--
--signal dac_write_fifo_register_s : STD_LOGIC_VECTOR(55 DOWNTO 0); --data received form spi
--signal dac_write_fifo_register_ready_s : STD_LOGIC;
--
---- DAC fifo out state machine
--TYPE DAC_STATE_TYPE IS (idle,extract,time_and_busy_check,send_complete);
--SIGNAL dac_state_s   : DAC_STATE_TYPE := idle;

-- GPIO registers
signal gpio_input_data_register_s : STD_LOGIC_VECTOR(15 downto 0); 	-- Data register for GPIO input
signal gpio_output_data_register_s : STD_LOGIC_VECTOR(15 downto 0);  -- Data register for GPIO output

begin

--DAC_FIFO : fifo
--	Generic map(
--			ADDR_W	=> 6,					-- address width in bits
--			DATA_W 	=> 56, 				-- data width in bits
--			BUFF_L	=> 16,				-- buffer length must be less than address space as in  BUFF_L <or= 2^(ADDR_W)-1
--			ALMST_F	=> 3,					-- fifo flag for almost full regs away from empty fifo
--			ALMST_E	=> 3						-- fifo regs away from empty fifo
--			)
--	Port map( 
--			clk 					=> CLK,
--			n_reset 				=> not RESET,
--			rd_en 				=> dac_fifo_rd_en_s,
--			wr_en					=> dac_fifo_wr_en_s,
--			data_in 				=> dac_fifo_data_in_s,
--			data_out				=> dac_fifo_data_out_s,
--			data_count			=> open,
--			empty 				=> dac_fifo_empty_flag_s,
--			full					=> dac_fifo_full_flag_s,
--			almst_empty 		=> open,
--			almst_full 			=> open,
--			err					=> open
--);


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

process(CLK)
begin

	-- Update chip select on rising edge of the system clock only
	if(rising_edge(CLK) ) then


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
			OUTPUT <= gpio_output_data_register_s;
		end if;

--		-- Manage the dac write into the fifo
--		if(dac_write_fifo_register_ready_s = '1') then
--			dac_write_fifo_register_ready_s <= '0';
--			dac_fifo_wr_en_s <= '1';
--			dac_fifo_data_in_s <= dac_write_fifo_register_s;
--		else
--			dac_fifo_wr_en_s <= '0';
--		end if;

--		-- Processing of the DAC fifo output
--		if(RESET = '1') then
--			dac_state_s <= idle;
--		else
--			CASE dac_state_s IS
--				WHEN idle=> -- Wait here until data to extract form the fifo
--					if(dac_fifo_empty_flag_s = '0') then
--						dac_state_s <= extract;
--					end if;
--					dac_spi_master_en_s <= '0';
--				WHEN extract=>
--					dac_fifo_rd_en_s <= '1';
--					dac_state_s <= time_and_busy_check;
--				WHEN time_and_busy_check=>
--					-- Clear the fifo output eable signal (the data is still valid at it's output, but no more data is pump out)
--					dac_fifo_rd_en_s <= '0';
--					-- Check if time to sent the last element from the fifo (element has to be sent now, and spi is not busy)
--					if(SYNC_TIME >= dac_fifo_data_out_s(55 downto 24) and dac_spi_master_busy_s = '0') then
--						-- Build the spi command to send to the DAC
--						dac_spi_master_tx_data_s <= "00010" & dac_fifo_data_out_s(18 downto 0);
--						-- Trig the spi send to the dac
--						dac_spi_master_en_s <= '1';
--						-- Jump to next state
--						dac_state_s <= send_complete;
--					end if;
--				WHEN send_complete=>
--					dac_spi_master_en_s <= '0';
--					dac_state_s <= idle;
--			END CASE;
--		end if;
--

	
		-- Reset signal
		if(RESET = '1') then
			-- reset outputs to zero
		else
			-- if the command is now ready
			if(COMMAND_READY = '1') then
				CASE COMMAND IS
					WHEN "00000000" => -- Command 0 : Read card ID
						-- No receive data

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


--					WHEN "00000100" => -- Command 4 : Timed delayed dac output setup
--						-- Check if data received flag is set
--						if(BYTE_RX_READY_PULSE ='1') then
--							if(unsigned(BYTE_RX_COUNT) = 1) then
--								dac_write_fifo_register_s(55 downto 48) <= BYTE_RX;
--								dac_fifo_full_flag_latch_s <= dac_fifo_full_flag_s;
--							elsif (unsigned(BYTE_RX_COUNT) = 2) then
--								dac_write_fifo_register_s(47 downto 40) <= BYTE_RX;
--							elsif (unsigned(BYTE_RX_COUNT) = 3) then
--								dac_write_fifo_register_s(39 downto 32) <= BYTE_RX;
--							elsif (unsigned(BYTE_RX_COUNT) = 4) then
--								dac_write_fifo_register_s(31 downto 24) <= BYTE_RX;
--							elsif (unsigned(BYTE_RX_COUNT) = 5) then
--								dac_write_fifo_register_s(23 downto 16) <= BYTE_RX;
--							elsif (unsigned(BYTE_RX_COUNT) = 6) then
--								dac_write_fifo_register_s(15 downto 8) <= BYTE_RX;
--							elsif (unsigned(BYTE_RX_COUNT) = 7) then
--								dac_write_fifo_register_s(7 downto 0) <= BYTE_RX;
--								if(dac_fifo_full_flag_latch_s = '0') then
--									dac_write_fifo_register_ready_s <= '1';
--								end if;
--							end if;
--						end if; 
--						
--						if (unsigned(BYTE_RX_COUNT) = 7) then
--							-- Send the full flag state of the fifo in the LSB. If the full flag is set, the write will not occur
--							BYTE_TX <= "0000000" & dac_fifo_full_flag_latch_s;
--						else
--							BYTE_TX <= (others => '0');
--						end if;
--
--						-- Command check
--						if(BYTE_RX_READY_PULSE ='1') then
--							if(unsigned(BYTE_RX_COUNT) = 7) then
--								valid_command_completed_s <= '1';
--							end if;
--						end if;

					WHEN OTHERS =>
						BYTE_TX <= (others => '0');
						invalid_command_detected_s <= '1';
					
				END CASE;
			end if; -- if(COMMAND_READY = '1') then
		end if; -- if(RESET = '1') then
	end if; -- if(rising_edge(CLK) ) then

end process;
end Behavioral;

