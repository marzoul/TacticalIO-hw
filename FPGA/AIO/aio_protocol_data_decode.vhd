----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    20:20:05 11/03/2014 
-- Design Name: 
-- Module Name:    aio_protocol_data_decode - Behavioral 
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

entity aio_protocol_data_decode is
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

				-- SPI DAC
				DAC_BIN					: out  	STD_LOGIC;
				DAC_SCLK					: out  	STD_LOGIC;
				DAC_SDO					: in  	STD_LOGIC;
				DAC_N_LDAC				: out  	STD_LOGIC;
				DAC_D1					: in  	STD_LOGIC;
				DAC_N_RSTIN				: out  	STD_LOGIC;
				DAC_D0					: in  	STD_LOGIC;
				DAC_N_CLR				: out  	STD_LOGIC;
				DAC_SDIN					: out  	STD_LOGIC;
				DAC_N_SYNC				: out  	STD_LOGIC;

				-- SPI ADC
				ADC_OS					: out  	STD_LOGIC_VECTOR(2 downto 0);
				ADC_RANGE				: out  	STD_LOGIC;
				ADC_CONVST_B			: out  	STD_LOGIC;
				ADC_SCLK					: out  	STD_LOGIC;
				ADC_BUSY					: in  	STD_LOGIC;
				ADC_DOUT_A				: in  	STD_LOGIC;
				ADC_DOUT_B				: in  	STD_LOGIC;
				ADC_FRSTDATA			: in  	STD_LOGIC;
				ADC_N_CS					: out  	STD_LOGIC;
				ADC_RESET				: out  	STD_LOGIC;
				ADC_CONVST_A			: out  	STD_LOGIC;
				ADC_N_STBY				: out  	STD_LOGIC;

				-- SPARE IO
				SPARE1					: out  	STD_LOGIC;
				SPARE2					: out  	STD_LOGIC;
				SPARE3					: out  	STD_LOGIC;
				SPARE4					: out  	STD_LOGIC;
				SPARE5					: out  	STD_LOGIC;
				SPARE6					: out  	STD_LOGIC;
				SPARE7					: out  	STD_LOGIC;
				SPARE8					: out  	STD_LOGIC;
				SPARE9					: out  	STD_LOGIC;
				SPARE10					: out  	STD_LOGIC
				);
end aio_protocol_data_decode;

architecture Behavioral of aio_protocol_data_decode is

component spi_master
  GENERIC(
    slaves  : INTEGER := 4;  --number of spi slaves
    d_width : INTEGER := 2;
	 busy_release_clk_delay : INTEGER := 20 -- number of sck period before release the busy signal
	 ); --data bus width
  PORT(
    clock   : IN     STD_LOGIC;                             --system clock
    reset_n : IN     STD_LOGIC;                             --asynchronous reset
    enable  : IN     STD_LOGIC;                             --initiate transaction
    cpol    : IN     STD_LOGIC;                             --spi clock polarity
    cpha    : IN     STD_LOGIC;                             --spi clock phase
    cont    : IN     STD_LOGIC;                             --continuous mode command
    clk_div : IN     INTEGER;                               --system clock cycles per 1/2 period of sclk
    addr    : IN     INTEGER;                               --address of slave
    tx_data : IN     STD_LOGIC_VECTOR(d_width-1 DOWNTO 0);  --data to transmit
    miso    : IN     STD_LOGIC;                             --master in, slave out
    sclk    : BUFFER STD_LOGIC;                             --spi clock
    ss_n    : BUFFER STD_LOGIC_VECTOR(slaves-1 DOWNTO 0);   --slave select
    mosi    : OUT    STD_LOGIC;                             --master out, slave in
    busy    : OUT    STD_LOGIC;                             --busy / data ready signal
    rx_data : OUT    STD_LOGIC_VECTOR(d_width-1 DOWNTO 0)); --data received
END component;

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

signal aio_config_register_s : STD_LOGIC_VECTOR(7 downto 0); -- Configuration register for the AIO card
-- Bit 0 : Unused
-- Bit 1 : Unused
-- Bit 2 : Unused
-- Bit 3 : Unused
-- Bit 4 : Unused
-- Bit 5 : Unused
-- Bit 6 : Unused
-- Bit 7 : Unused

signal aio_led_control_register_s : STD_LOGIC_VECTOR(7 downto 0); -- Led control register for the AIO card
-- Bit 0 : LED1 on FPGA
-- Bit 1 : LED2 on FPGA
-- Bit 2 : LED3 on FPGA
-- Bit 3 to 7 : Unused

-- DAC SPI master control signal
signal dac_spi_master_en_s      : STD_LOGIC;                             --initiate transaction
signal dac_spi_master_tx_data_s : STD_LOGIC_VECTOR(23 DOWNTO 0);  --data to transmit
signal dac_spi_master_sclk_s    : STD_LOGIC;                             --spi clock
signal dac_spi_master_ss_n_s    : STD_LOGIC_VECTOR(0 DOWNTO 0);   --slave select
signal dac_spi_master_mosi_s    : STD_LOGIC;                             --master out, slave in
signal dac_spi_master_busy_s    : STD_LOGIC;                             --busy / data ready signal
signal dac_spi_master_rx_data_s : STD_LOGIC_VECTOR(23 DOWNTO 0); --data received

-- Dac write signals
signal dac_write_register_s : STD_LOGIC_VECTOR(23 DOWNTO 0); --data received

-- DAC fifo signals
signal dac_fifo_rd_en_s : STD_LOGIC;
signal dac_fifo_wr_en_s : STD_LOGIC;
signal dac_fifo_data_in_s : std_logic_vector(55 downto 0); 
signal dac_fifo_data_out_s : std_logic_vector(55 downto 0); 
signal dac_fifo_empty_flag_s : STD_LOGIC;
signal dac_fifo_full_flag_s : STD_LOGIC;

signal dac_fifo_full_flag_latch_s : STD_LOGIC;

signal dac_write_fifo_register_s : STD_LOGIC_VECTOR(55 DOWNTO 0); --data received form spi
signal dac_write_fifo_register_ready_s : STD_LOGIC;

-- DAC fifo out state machine
TYPE DAC_STATE_TYPE IS (idle,extract,time_and_busy_check,send_complete);
SIGNAL dac_state_s   : DAC_STATE_TYPE := idle;

signal dac_fifo_rd_data_ready_s : STD_LOGIC;

-- ADC SPI master signals

signal adc_spi_master_rx_data_s : STD_LOGIC_VECTOR(127 DOWNTO 0); --data received from adc shift register
signal adc_spi_master_rx_data_ready_s : STD_LOGIC; -- tell that the content of the register latch is the latestes one (cleared when pending conversion)
signal adc_spi_master_busy_s : STD_LOGIC;
signal adc_spi_master_busy_d_s : STD_LOGIC;
signal adc_read_result_trig_s: STD_LOGIC;
signal adc_n_cs_s : STD_LOGIC_VECTOR(0 downto 0);
signal adc_sclk_s : STD_LOGIC;
signal adc_dout_s : STD_LOGIC;



-- ADC SPI master signals Latch (when the CPU start a read, to not interfer with the current readback
signal adc_spi_master_rx_data_latch_s : STD_LOGIC_VECTOR(127 DOWNTO 0); --data received from adc shift register

-- ADC fifo signals
signal adc_fifo_rd_en_s : STD_LOGIC;
signal adc_fifo_wr_en_s : STD_LOGIC;
signal adc_fifo_data_in_s : std_logic_vector(31 downto 0); 
signal adc_fifo_data_out_s : std_logic_vector(31 downto 0); 
signal adc_fifo_empty_flag_s : STD_LOGIC;
signal adc_fifo_full_flag_s : STD_LOGIC;
signal adc_fifo_full_flag_latch_s : STD_LOGIC;

signal adc_write_fifo_register_s : STD_LOGIC_VECTOR(31 DOWNTO 0); --data received form spi
signal adc_write_fifo_register_ready_s : STD_LOGIC;

-- ADC conversion start signals
signal adc_conversion_trig_s : STD_LOGIC; -- signal that trig a conversion
signal adc_conversion_trig_extender_d_s : STD_LOGIC_VECTOR(4 downto 0);
signal adc_convst_s : STD_LOGIC;


-- ADC conversion data recover signals
signal adc_busy_d_s : STD_LOGIC_VECTOR(3 downto 0);

-- ADC fifo out state machine
TYPE ADC_STATE_TYPE IS (idle,extract,time_and_busy_check,send_complete);
SIGNAL adc_state_s   : ADC_STATE_TYPE := idle;

signal adc_fifo_rd_data_ready_s : STD_LOGIC;

begin

DAC_SPI_MASTER : spi_master
  GENERIC map(
    slaves  => 1,  --number of spi slaves
    d_width => 24,
	 busy_release_clk_delay => 15 -- number of sck period before release the busy signal (90ns min for AD5764)
	 ) --data bus width
  PORT map(
    clock   => CLK,				                             	--system clock
    reset_n => not RESET,		                             	--asynchronous reset
    enable  => dac_spi_master_en_s,                            	--initiate transaction
    cpol    => '0',                          					--spi clock polarity
    cpha    => '1',                          					--spi clock phase
    cont    => '0',                          					--continuous mode command
    clk_div => 4,				                              --system clock cycles per 1/2 period of sclk -- Set clock rate 100Mhz / (2*4) = 12.5Mhz
    addr    => 0,						                       		--address of slave
    tx_data => dac_spi_master_tx_data_s,								--data to transmit
    miso    => DAC_SDO,                           				--master in, slave out
    sclk    => dac_spi_master_sclk_s,                           --spi clock
    ss_n    => dac_spi_master_ss_n_s,								   --slave select
    mosi    => dac_spi_master_mosi_s,			                  --master out, slave in
    busy  	=> dac_spi_master_busy_s,                           --busy / data ready signal
    rx_data => dac_spi_master_rx_data_s									--data received
	);

ADC_SPI_MASTER : spi_master
  GENERIC map(
    slaves  => 1,  --number of spi slaves
    d_width => 128,
	 busy_release_clk_delay => 15 -- number of sck period before release the busy signal (90ns min for AD5764)
	 ) --data bus width
  PORT map(
    clock   => CLK,				                             	--system clock
    reset_n => not RESET,		                             	--asynchronous reset
    enable  => adc_read_result_trig_s,                      --initiate transaction
    cpol    => '1',                          					--spi clock polarity
    cpha    => '0',                          					--spi clock phase
    cont    => '0',                          					--continuous mode command
    clk_div => 4,				                              	--system clock cycles per 1/2 period of sclk -- Set clock rate 100Mhz / (2*4) = 12.5Mhz
    addr    => 0,						                       		--address of slave
    tx_data => (others => '0'),										--data to transmit
    miso    => adc_dout_s,                           			--master in, slave out
    sclk    => adc_sclk_s,                     					--spi clock
    ss_n    => adc_n_cs_s,												--slave select
    mosi    => open,			                  					--master out, slave in
    busy  	=> adc_spi_master_busy_s,                       --busy / data ready signal
    rx_data => adc_spi_master_rx_data_s							--data received
	);

DAC_FIFO : fifo
	Generic map(
			ADDR_W	=> 6,					-- address width in bits
			DATA_W 	=> 56, 				-- data width in bits
			BUFF_L	=> 16,				-- buffer length must be less than address space as in  BUFF_L <or= 2^(ADDR_W)-1
			ALMST_F	=> 3,					-- fifo flag for almost full regs away from empty fifo
			ALMST_E	=> 3						-- fifo regs away from empty fifo
			)
	Port map( 
			clk 					=> CLK,
			n_reset 				=> not RESET,
			rd_en 				=> dac_fifo_rd_en_s,
			wr_en					=> dac_fifo_wr_en_s,
			data_in 				=> dac_fifo_data_in_s,
			data_out				=> dac_fifo_data_out_s,
			data_count			=> open,
			empty 				=> dac_fifo_empty_flag_s,
			full					=> dac_fifo_full_flag_s,
			almst_empty 		=> open,
			almst_full 			=> open,
			err					=> open
);


ADC_FIFO : fifo
	Generic map(
			ADDR_W	=> 6,					-- address width in bits
			DATA_W 	=> 32, 				-- data width in bits
			BUFF_L	=> 16,				-- buffer length must be less than address space as in  BUFF_L <or= 2^(ADDR_W)-1
			ALMST_F	=> 3,					-- fifo flag for almost full regs away from empty fifo
			ALMST_E	=> 3						-- fifo regs away from empty fifo
			)
	Port map( 
			clk 					=> CLK,
			n_reset 				=> not RESET,
			rd_en 				=> adc_fifo_rd_en_s,
			wr_en					=> adc_fifo_wr_en_s,
			data_in 				=> adc_fifo_data_in_s,
			data_out				=> adc_fifo_data_out_s,
			data_count			=> open,
			empty 				=> adc_fifo_empty_flag_s,
			full					=> adc_fifo_full_flag_s,
			almst_empty 		=> open,
			almst_full 			=> open,
			err					=> open
);


VALID_COMMAND_RX <= valid_command_rx_s;
INVALID_COMMAND_RX <= invalid_command_rx_s;
INCOMPLETE_COMMAND_RX <= incomplete_command_rx_s;

-- user led display
LED1 <= aio_led_control_register_s(0);
LED2 <= aio_led_control_register_s(1);
LED3 <= aio_led_control_register_s(2);


-- time sync pulse synchro signal (named as fault signal)
FAULT <= 'Z'; -- Leave floating for now...

-- ADC configuration
ADC_OS <= "000";  -- Oversampling
ADC_RANGE <= '1'; -- Output range : 1 = +/- 10V , 0 = +/- 5V
ADC_RESET <= RESET; -- Reset of the ADC
ADC_CONVST_A <= adc_convst_s; -- Sampling start on channel A
ADC_CONVST_B <= adc_convst_s; -- Sampling start on channel B
ADC_N_STBY <= '1'; -- Do not put the adc in stby
ADC_N_CS <= adc_n_cs_s(0);
ADC_SCLK <= adc_sclk_s;
adc_dout_s <= ADC_DOUT_A;

-- DAC configuration
DAC_N_SYNC <= dac_spi_master_ss_n_s(0); --'1'; --dac_cs_s; -- connect the CS of the dac
DAC_N_RSTIN <= '1'; -- do not reset the dac
DAC_N_CLR <= '1'; -- dac register reset
DAC_SDIN <= dac_spi_master_mosi_s; --'0'; -- conect the MOSI signal
DAC_BIN <= '0'; -- set the dac as 2 complement output
DAC_SCLK <= dac_spi_master_sclk_s; --'0';
DAC_N_LDAC <= '0'; -- update the output when cs rise up

-- Debug signals
SPARE1 <= '0';
SPARE2 <= '0';
SPARE3 <= '0';
SPARE4 <= '0'; -- Do not use... dead io
SPARE5 <= '0';
SPARE6 <= '0';
SPARE7 <= '0';
SPARE8 <= '0';
SPARE9 <= '0';
SPARE10 <= '0';

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
			-- Nothing to do for now...
			sync_time_read_latch_s <= SYNC_TIME; -- Latch the time for a read in the spi command
		end if;

		-- Manage the dac write into the fifo
		if(dac_write_fifo_register_ready_s = '1') then
			dac_write_fifo_register_ready_s <= '0';
			dac_fifo_wr_en_s <= '1';
			dac_fifo_data_in_s <= dac_write_fifo_register_s;
		else
			dac_fifo_wr_en_s <= '0';
		end if;

		
		-- Manage the adc write into the fifo
		if(adc_write_fifo_register_ready_s = '1') then
			adc_write_fifo_register_ready_s <= '0';
			adc_fifo_wr_en_s <= '1';
			adc_fifo_data_in_s <= adc_write_fifo_register_s;
		else
			adc_fifo_wr_en_s <= '0';
		end if;


		-- Create delay on the ADC_BUSY signal to have a edge detect on it
		adc_busy_d_s(0) <= ADC_BUSY;
		adc_busy_d_s(1) <= adc_busy_d_s(0);
		adc_busy_d_s(2) <= adc_busy_d_s(1);
		adc_busy_d_s(3) <= adc_busy_d_s(2);
		
		-- When the ADC has finish a conversion (busy signal goes low) (plus a litle delay...) Trig a read of the results
		if(adc_busy_d_s(3) = '1' and adc_busy_d_s(2)='0') then
			adc_read_result_trig_s <= '1';		
		else
			adc_read_result_trig_s <= '0';
		end if;
		
		-- When adc spi master has finish recover data... rise a flag on falling edge of spi busy signal
		adc_spi_master_busy_d_s <= adc_spi_master_busy_s;

		if(adc_spi_master_busy_d_s = '1' and adc_spi_master_busy_s = '0') then
			adc_spi_master_rx_data_ready_s <= '1';
		end if;

		-- Need to extend the trig signal to meet the timing requirements of the IC
		adc_conversion_trig_extender_d_s(0) <= adc_conversion_trig_s;
		adc_conversion_trig_extender_d_s(1) <= adc_conversion_trig_extender_d_s(0);
		adc_conversion_trig_extender_d_s(2) <= adc_conversion_trig_extender_d_s(1);
		adc_conversion_trig_extender_d_s(3) <= adc_conversion_trig_extender_d_s(2);
		adc_conversion_trig_extender_d_s(4) <= adc_conversion_trig_extender_d_s(3);
		-- Big or of all of them, so the resulting signsal is a longer pulse (invert the rsult to generate the good polarity for the trig signal
		adc_convst_s <= not (adc_conversion_trig_extender_d_s(0) or adc_conversion_trig_extender_d_s(1) or adc_conversion_trig_extender_d_s(2) or adc_conversion_trig_extender_d_s(3) or adc_conversion_trig_extender_d_s(4));

		if(dac_fifo_rd_data_ready_s = '1') then
			-- Check if time to sent the last element from the fifo (element has to be sent now, and spi is not busy)
			if(SYNC_TIME >= dac_fifo_data_out_s(55 downto 24) and dac_spi_master_busy_s = '0') then
				-- Build the spi command to send to the DAC
				dac_spi_master_tx_data_s <= "00010" & dac_fifo_data_out_s(18 downto 0);
				-- Trig the spi send to the dac
				dac_spi_master_en_s <= '1';
				-- Jump to next state
				dac_state_s <= send_complete;
			end if;
		else
			dac_spi_master_en_s <= '0';
		end if;

		
		-- Processing of the DAC fifo output
		if(RESET = '1') then
			dac_state_s <= idle;
			dac_fifo_rd_data_ready_s <= '0';
		else
			CASE dac_state_s IS
				WHEN idle=> -- Wait here until data to extract form the fifo
					if(dac_fifo_empty_flag_s = '0') then
						dac_state_s <= extract;
					end if;
					dac_spi_master_en_s <= '0';
				WHEN extract=>
					dac_fifo_rd_en_s <= '1';
					dac_state_s <= time_and_busy_check;
				WHEN time_and_busy_check=>
					-- Clear the fifo output eable signal (the data is still valid at it's output, but no more data is pump out)
					dac_fifo_rd_en_s <= '0';
					
					dac_fifo_rd_data_ready_s <= '1';
				WHEN send_complete=>
					dac_fifo_rd_data_ready_s <= '0';
					--dac_spi_master_en_s <= '0';
					dac_state_s <= idle;
			END CASE;
		end if;

		if(adc_fifo_rd_data_ready_s = '1') then
			if(SYNC_TIME >= adc_fifo_data_out_s and ADC_BUSY = '0') then
				adc_conversion_trig_s <= '1';
				adc_state_s <= send_complete;
			end if;
		else
			adc_conversion_trig_s <= '0';
		end if;

		-- Processing of the ADC fifo output
		if(RESET = '1') then
			adc_state_s <= idle;
			adc_conversion_trig_s <= '0';
			adc_fifo_rd_data_ready_s <= '0';
		else
			CASE adc_state_s IS
				WHEN idle=> -- Wait here until data to extract form the fifo
					if(adc_fifo_empty_flag_s = '0') then
						adc_state_s <= extract;
					end if;
				WHEN extract=>
					adc_fifo_rd_en_s <= '1';
					adc_state_s <= time_and_busy_check;
				WHEN time_and_busy_check=>
					adc_fifo_rd_en_s <= '0';
					adc_fifo_rd_data_ready_s <= '1';
				WHEN send_complete=>
					adc_fifo_rd_data_ready_s <= '0';
					--adc_conversion_trig_s <= '0';
					adc_state_s <= idle;
					
			END CASE;
		end if;

	
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
								aio_config_register_s <= BYTE_RX;
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
								aio_led_control_register_s <= BYTE_RX;
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


					WHEN "00000100" => -- Command 4 : Timed delayed dac output setup
						-- Check if data received flag is set
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 1) then
								dac_write_fifo_register_s(55 downto 48) <= BYTE_RX;
								dac_fifo_full_flag_latch_s <= dac_fifo_full_flag_s;
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								dac_write_fifo_register_s(47 downto 40) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 3) then
								dac_write_fifo_register_s(39 downto 32) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 4) then
								dac_write_fifo_register_s(31 downto 24) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 5) then
								dac_write_fifo_register_s(23 downto 16) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 6) then
								dac_write_fifo_register_s(15 downto 8) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 7) then
								dac_write_fifo_register_s(7 downto 0) <= BYTE_RX;
								if(dac_fifo_full_flag_latch_s = '0') then
									dac_write_fifo_register_ready_s <= '1';
								end if;
							end if;
						end if; 
						
						if (unsigned(BYTE_RX_COUNT) = 7) then
							-- Send the full flag state of the fifo in the LSB. If the full flag is set, the write will not occur
							BYTE_TX <= "0000000" & dac_fifo_full_flag_latch_s;
						else
							BYTE_TX <= (others => '0');
						end if;

						-- Command check
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 7) then
								valid_command_completed_s <= '1';
							end if;
						end if;


					WHEN "00000101" => -- Command 5 : Timed delayed adc trig conversion
						-- Check if data received flag is set
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 1) then
								adc_write_fifo_register_s(31 downto 24) <= BYTE_RX;
								adc_fifo_full_flag_latch_s <= adc_fifo_full_flag_s;
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								adc_write_fifo_register_s(23 downto 16) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 3) then
								adc_write_fifo_register_s(15 downto 8) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 4) then
								adc_write_fifo_register_s(7 downto 0) <= BYTE_RX;
								if(adc_fifo_full_flag_latch_s = '0') then
									adc_write_fifo_register_ready_s <= '1';
								end if;
							end if;
						end if; 
						
						if (unsigned(BYTE_RX_COUNT) = 4) then
							-- Send the full flag state of the fifo in the LSB. If the full flag is set, the write will not occur
							BYTE_TX <= "0000000" & adc_fifo_full_flag_latch_s;
						else
							BYTE_TX <= (others => '0');
						end if;

						-- Command check
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 4) then
								valid_command_completed_s <= '1';
							end if;
						end if;

					WHEN "00000110" => -- Command 6 : Read last completed conversion result
						-- Nothing to write on the ADC
						
						if (unsigned(BYTE_RX_COUNT) = 1) then
								adc_spi_master_rx_data_latch_s <= adc_spi_master_rx_data_s;
								BYTE_TX <= "0000000" & adc_spi_master_rx_data_ready_s;
						elsif (unsigned(BYTE_RX_COUNT) = 2) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(127 downto 120);
						elsif (unsigned(BYTE_RX_COUNT) = 3) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(119 downto 112);
						elsif (unsigned(BYTE_RX_COUNT) = 4) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(111 downto 104);
						elsif (unsigned(BYTE_RX_COUNT) = 5) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(103 downto 96);
						elsif (unsigned(BYTE_RX_COUNT) = 6) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(95 downto 88);
						elsif (unsigned(BYTE_RX_COUNT) = 7) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(87 downto 80);
						elsif (unsigned(BYTE_RX_COUNT) = 8) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(79 downto 72);
						elsif (unsigned(BYTE_RX_COUNT) = 9) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(71 downto 64);
						elsif (unsigned(BYTE_RX_COUNT) = 10) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(63 downto 56);
						elsif (unsigned(BYTE_RX_COUNT) = 11) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(55 downto 48);
						elsif (unsigned(BYTE_RX_COUNT) = 12) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(47 downto 40);
						elsif (unsigned(BYTE_RX_COUNT) = 13) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(39 downto 32);
						elsif (unsigned(BYTE_RX_COUNT) = 14) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(31 downto 24);
						elsif (unsigned(BYTE_RX_COUNT) = 15) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(23 downto 16);
						elsif (unsigned(BYTE_RX_COUNT) = 16) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(15 downto 8);
						elsif (unsigned(BYTE_RX_COUNT) = 17) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(7 downto 0);
							adc_spi_master_rx_data_ready_s <= '0'; -- clear the data ready flag
						else
							BYTE_TX <= (others => '0');
						end if;

						-- Command check
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 17) then
								valid_command_completed_s <= '1';
							end if;
						end if;


					WHEN "00000111" => -- Command 7 : Read ADC NOW!
						-- Nothing to write on the ADC

						if (unsigned(BYTE_RX_COUNT) = 1) then	-- Byte 1 , add command in fifo with time 0 (will be run imemdiatly)
								BYTE_TX <= "00000000";
								adc_write_fifo_register_s <= (others => '0');
								adc_write_fifo_register_ready_s <= '1';
								adc_fifo_full_flag_latch_s <= adc_fifo_full_flag_s;
						elsif (unsigned(BYTE_RX_COUNT) = 2) then
							-- Send the full flag state of the fifo in the LSB. If the full flag is set, the write will not occur
							BYTE_TX <= "0000000" & adc_fifo_full_flag_latch_s;
						elsif (unsigned(BYTE_RX_COUNT) = 16) then
								adc_spi_master_rx_data_latch_s <= adc_spi_master_rx_data_s;
								BYTE_TX <= "0000000" & adc_spi_master_rx_data_ready_s;
						elsif (unsigned(BYTE_RX_COUNT) = 17) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(127 downto 120);
						elsif (unsigned(BYTE_RX_COUNT) = 18) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(119 downto 112);
						elsif (unsigned(BYTE_RX_COUNT) = 19) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(111 downto 104);
						elsif (unsigned(BYTE_RX_COUNT) = 20) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(103 downto 96);
						elsif (unsigned(BYTE_RX_COUNT) = 21) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(95 downto 88);
						elsif (unsigned(BYTE_RX_COUNT) = 22) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(87 downto 80);
						elsif (unsigned(BYTE_RX_COUNT) = 23) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(79 downto 72);
						elsif (unsigned(BYTE_RX_COUNT) = 24) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(71 downto 64);
						elsif (unsigned(BYTE_RX_COUNT) = 25) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(63 downto 56);
						elsif (unsigned(BYTE_RX_COUNT) = 26) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(55 downto 48);
						elsif (unsigned(BYTE_RX_COUNT) = 27) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(47 downto 40);
						elsif (unsigned(BYTE_RX_COUNT) = 28) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(39 downto 32);
						elsif (unsigned(BYTE_RX_COUNT) = 29) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(31 downto 24);
						elsif (unsigned(BYTE_RX_COUNT) = 30) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(23 downto 16);
						elsif (unsigned(BYTE_RX_COUNT) = 31) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(15 downto 8);
						elsif (unsigned(BYTE_RX_COUNT) = 32) then
							BYTE_TX <= adc_spi_master_rx_data_latch_s(7 downto 0);
							adc_spi_master_rx_data_ready_s <= '0'; -- clear the data ready flag
						else
							BYTE_TX <= (others => '0');
						end if;


						-- Command check
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 32) then
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

