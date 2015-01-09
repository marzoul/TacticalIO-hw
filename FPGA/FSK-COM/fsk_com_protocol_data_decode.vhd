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
use ieee.std_logic_unsigned.all;

entity fsk_com_protocol_data_decode is
    Port ( 
				-- System clock
				CLK 						: in		STD_LOGIC;
				
				-- System reset signal
				RESET 					: in 		STD_LOGIC;
				
				-- Time reference
				SYNC_TIME 				: in 		STD_LOGIC_VECTOR(31 downto 0);
				
				-- SPI Interface from cpu
				MOSI : in STD_LOGIC;
				SCK : in STD_LOGIC;
				CS : in STD_LOGIC;
				
				
				-- Protocol interface
				COMMAND 					: in 		STD_LOGIC_VECTOR(7 downto 0);
				COMMAND_READY 			: in 		STD_LOGIC;
				
				BYTE_RX 					: in 		STD_LOGIC_VECTOR(7 downto 0);
				BYTE_TX 					: out 	STD_LOGIC_VECTOR(7 downto 0);
				BYTE_RX_COUNT 			: in 		STD_LOGIC_VECTOR(7 downto 0);
				BYTE_RX_READY_PULSE 	: in 		STD_LOGIC;
				
				-- MISO source selector
				MISO_SRC_SEL 			: out STD_LOGIC_VECTOR(1 downto 0);

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
end fsk_com_protocol_data_decode;

architecture Behavioral of fsk_com_protocol_data_decode is

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

--component fifo
--	Generic(
--			ADDR_W	: integer	:= 4;					-- address width in bits
--			DATA_W 	: integer	:= 8; 				-- data width in bits
--			BUFF_L	: integer 	:=16;					-- buffer length must be less than address space as in  BUFF_L <or= 2^(ADDR_W)-1
--			ALMST_F	: integer 	:= 3;					-- fifo flag for almost full regs away from empty fifo
--			ALMST_E	: integer	:= 3						-- fifo regs away from empty fifo
--			);
--	Port ( 
--			clk 					: in std_logic;
--			n_reset 				: in std_logic;
--			rd_en 				: in std_logic; 		-- read enable 
--			wr_en					: in std_logic; 		-- write enable 
--			data_in 				: in std_logic_vector(DATA_W- 1 downto 0); 
--			data_out				: out std_logic_vector(DATA_W- 1 downto 0); 
--			data_count			: out std_logic_vector(ADDR_W downto 0);
--			empty 				: out std_logic; 
--			full					: out std_logic;
--			almst_empty 		: out std_logic; 
--			almst_full 			: out std_logic; 
--			err					: out std_logic
--);
--end component;

component dp_ram_8x256
  PORT (
    clka : IN STD_LOGIC;
    wea : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
    addra : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    dina : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    clkb : IN STD_LOGIC;
    addrb : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    doutb : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
  );
END component;


component phase_accumulator
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
end component;


-- Board informations (for the ID register)
constant CARD_ID : integer := 3; -- (0=Invalid, 1=AIO, 2=GPIO, 3=FSK-COM)
constant CARD_MAJOR_VERSION : integer := 1;
constant CARD_MINOR_VERSION : integer := 1;

-- Internal signals

signal valid_command_completed_s : STD_LOGIC;		-- Goes high at end of a valid command
signal invalid_command_detected_s : STD_LOGIC;		-- Goes high at end of a invalid command

signal valid_command_rx_s : STD_LOGIC;					-- valid command pulse
signal invalid_command_rx_s : STD_LOGIC;				-- invalid command pulse
signal incomplete_command_rx_s : STD_LOGIC;			-- incomplete command pulse

signal command_ready_d_s : STD_LOGIC; -- Delay of command ready to detect rising edge for latch of inputs

signal sync_time_read_latch_s : STD_LOGIC_VECTOR(31 downto 0); -- to read it in the spi protocol

signal config_register_s : STD_LOGIC_VECTOR(7 downto 0); -- Configuration register for the card
-- Bit 0 : Unused
-- Bit 1 : Unused
-- Bit 2 : Unused
-- Bit 3 : Unused
-- Bit 4 : Unused
-- Bit 5 : Unused
-- Bit 6 : Unused
-- Bit 7 : Unused

signal led_control_register_s : STD_LOGIC_VECTOR(7 downto 0); -- Led control register for the card
-- Bit 0 : LED1 on FPGA
-- Bit 1 : LED2 on FPGA
-- Bit 2 : LED3 on FPGA
-- Bit 3 to 7 : Unused

-- DDS Signals
signal dds_io_ud_clk_s : STD_LOGIC;

-- DDS Ram
signal dds_ram_wr_en_s		: STD_LOGIC_VECTOR(0 downto 0);
signal dds_ram_wr_addr_s	: STD_LOGIC_VECTOR(7 downto 0);
signal dds_ram_wr_data_s	: STD_LOGIC_VECTOR(7 downto 0);
signal dds_ram_rd_addr_s	: STD_LOGIC_VECTOR(7 downto 0);
signal dds_ram_rd_data_s	: STD_LOGIC_VECTOR(7 downto 0);


-- DDS control signal
signal dds_wait_for_end_of_ram_filling_s : STD_LOGIC;
signal dds_ram_last_byte_s : STD_LOGIC;
signal dds_data_rdy_to_send_s : STD_LOGIC;
signal dds_send_gain_zero_s : STD_LOGIC;
signal dds_send_continous_s : STD_LOGIC;

signal fsk_stop_time_s : STD_LOGIC_VECTOR(31 downto 0);
signal fsk_stop_time_ready_s : STD_LOGIC;



--
-- DDS fifo out state machine
--TYPE DDS_STATE_TYPE IS (idle,wait_for_time,apply_gain,apply_gain_d1,apply_gain_d2,output_byte,output_bit7,output_bit6,output_bit5,output_bit4,output_bit3,output_bit2,output_bit1,output_bit0,clear_gain,send_complete);
TYPE DDS_STATE_TYPE IS (idle,wait_for_time,time_to_send,apply_gain,apply_gain_d1,apply_gain_d2,output_bit7,output_bit6,output_bit5,output_bit4,output_bit3,output_bit2,output_bit1,output_bit0,send_complete,send_complete_d1);
SIGNAL dds_state_s   : DDS_STATE_TYPE := idle;
signal dds_state_wait_for_time_s : STD_LOGIC;
signal dds_time_to_send_s : STD_LOGIC;

-- DDS Transmit control register
signal dds_channel_gain : STD_LOGIC_VECTOR(15 downto 0);
signal dds_tx_time_s : STD_LOGIC_VECTOR(31 downto 0);
signal dds_bits_to_send_on_last_byte_s : STD_LOGIC_VECTOR(3 downto 0);

-- DDS SPI Mux signals
signal dds_spi_mux_s : STD_LOGIC; -- 0 = connected to CPU, 1 = connected to FPGA
signal uart1_spi_mux_s : STD_LOGIC; -- 0 = connected to CPU, 1 = connected to FPGA
signal uart2_spi_mux_s : STD_LOGIC; -- 0 = connected to CPU, 1 = connected to FPGA

signal dds_spi_sck_fpga_s : STD_LOGIC;
signal dds_spi_mosi_fpga_s : STD_LOGIC;
signal dds_spi_cs_fpga_s : STD_LOGIC_VECTOR(0 downto 0);

signal uart1_spi_sck_fpga_s : STD_LOGIC;
signal uart1_spi_mosi_fpga_s : STD_LOGIC;
signal uart1_spi_cs_fpga_s : STD_LOGIC_VECTOR(0 downto 0);

signal uart2_spi_sck_fpga_s : STD_LOGIC;
signal uart2_spi_mosi_fpga_s : STD_LOGIC;
signal uart2_spi_cs_fpga_s : STD_LOGIC_VECTOR(0 downto 0);

-- DDS SPI MAster control signals
signal dds_spi_master_en_s : STD_LOGIC;
signal dds_spi_master_busy_s : STD_LOGIC;
signal dds_spi_master_tx_data_s : STD_LOGIC_VECTOR(23 downto 0);

-- DDS bit frequency control
signal dds_bit_pulse_s : STD_LOGIC;
signal dds_phase_accumulator_load_en_s : STD_LOGIC;
signal dds_phase_accumulator_word_s : STD_LOGIC_VECTOR(31 downto 0);
signal dds_phase_accumulator_reset_s : STD_LOGIC;


-- UART1 Managment
-- UART1 SPI Master control signals
signal uart1_spi_master_en_s : STD_LOGIC;
signal uart1_spi_master_busy_s : STD_LOGIC;
signal uart1_spi_master_tx_data_s : STD_LOGIC_VECTOR(15 downto 0);

-- Data registers
signal uart1_mode_s : STD_LOGIC; -- 0 = RS232 1 = RS485
signal uart1_time_s : STD_LOGIC_VECTOR(31 downto 0);
signal uart1_time_ready_s : STD_LOGIC;

-- UART2 Managment
-- UART2 SPI Master control signals
signal uart2_spi_master_en_s : STD_LOGIC;
signal uart2_spi_master_busy_s : STD_LOGIC;
signal uart2_spi_master_tx_data_s : STD_LOGIC_VECTOR(15 downto 0);

-- Data registers
signal uart2_mode_s : STD_LOGIC; -- 0 = RS232 1 = RS485
signal uart2_time_s : STD_LOGIC_VECTOR(31 downto 0);
signal uart2_time_ready_s : STD_LOGIC;



begin

DDS_SPI_MASTER : spi_master
  GENERIC map(
    slaves  => 1,  --number of spi slaves
    d_width => 24,
	 busy_release_clk_delay => 0 -- number of sck period before release the busy signal (90ns min for AD5764)
	 ) --data bus width
  PORT map(
    clock   => CLK,				                             	--system clock
    reset_n => not RESET,		                             	--asynchronous reset
    enable  => dds_spi_master_en_s,                         --initiate transaction
    cpol    => '0',                          					--spi clock polarity
    cpha    => '0',                          					--spi clock phase
    cont    => '0',                          					--continuous mode command
    clk_div => 10,				                              --system clock cycles per 1/2 period of sclk
    addr    => 0,						                       		--address of slave
    tx_data => dds_spi_master_tx_data_s,							--data to transmit
    miso    => DDS_SDO,                           				--master in, slave out
    sclk    => dds_spi_sck_fpga_s,                           --spi clock
    ss_n    => dds_spi_cs_fpga_s,								   --slave select
    mosi    => dds_spi_mosi_fpga_s,			                  --master out, slave in
    busy  	=> dds_spi_master_busy_s,                       --busy / data ready signal
    rx_data => open														--data received
	);


UART1_SPI_MASTER : spi_master
  GENERIC map(
    slaves  => 1,  --number of spi slaves
    d_width => 16,
	 busy_release_clk_delay => 1 -- number of sck period before release the busy signal (90ns min for AD5764)
	 ) --data bus width
  PORT map(
    clock   => CLK,				                             	--system clock
    reset_n => not RESET,		                             	--asynchronous reset
    enable  => uart1_spi_master_en_s,                         --initiate transaction
    cpol    => '0',                          					--spi clock polarity
    cpha    => '0',                          					--spi clock phase
    cont    => '0',                          					--continuous mode command
    clk_div => 10,				                              --system clock cycles per 1/2 period of sclk
    addr    => 0,						                       		--address of slave
    tx_data => uart1_spi_master_tx_data_s,							--data to transmit
    miso    => UART1_MISO,                           				--master in, slave out
    sclk    => uart1_spi_sck_fpga_s,                           --spi clock
    ss_n    => uart1_spi_cs_fpga_s,								   --slave select
    mosi    => uart1_spi_mosi_fpga_s,			                  --master out, slave in
    busy  	=> uart1_spi_master_busy_s,                       --busy / data ready signal
    rx_data => open														--data received
	);

UART2_SPI_MASTER : spi_master
  GENERIC map(
    slaves  => 1,  --number of spi slaves
    d_width => 16,
	 busy_release_clk_delay => 1 -- number of sck period before release the busy signal (90ns min for AD5764)
	 ) --data bus width
  PORT map(
    clock   => CLK,				                             	--system clock
    reset_n => not RESET,		                             	--asynchronous reset
    enable  => uart2_spi_master_en_s,                         --initiate transaction
    cpol    => '0',                          					--spi clock polarity
    cpha    => '0',                          					--spi clock phase
    cont    => '0',                          					--continuous mode command
    clk_div => 10,				                              --system clock cycles per 1/2 period of sclk
    addr    => 0,						                       		--address of slave
    tx_data => uart2_spi_master_tx_data_s,							--data to transmit
    miso    => UART2_MISO,                           				--master in, slave out
    sclk    => uart2_spi_sck_fpga_s,                           --spi clock
    ss_n    => uart2_spi_cs_fpga_s,								   --slave select
    mosi    => uart2_spi_mosi_fpga_s,			                  --master out, slave in
    busy  	=> uart2_spi_master_busy_s,                       --busy / data ready signal
    rx_data => open														--data received
	);


--DDS_FIFO : fifo
--	Generic map(
--			ADDR_W	=> 7,					-- address width in bits
--			DATA_W 	=> 8, 				-- data width in bits
--			BUFF_L	=> 128,				-- buffer length must be less than address space as in  BUFF_L <or= 2^(ADDR_W)-1
--			ALMST_F	=> 3,					-- fifo flag for almost full regs away from empty fifo
--			ALMST_E	=> 3						-- fifo regs away from empty fifo
--			)
--	Port map( 
--			clk 					=> CLK,
--			n_reset 				=> not RESET,
--			rd_en 				=> dds_fifo_rd_en_s,
--			wr_en					=> dds_fifo_wr_en_s,
--			data_in 				=> dds_fifo_data_in_s,
--			data_out				=> dds_fifo_data_out_s,
--			data_count			=> open,
--			empty 				=> dds_fifo_empty_flag_s,
--			full					=> dds_fifo_full_flag_s,
--			almst_empty 		=> open,
--			almst_full 			=> open,
--			err					=> open
--);

DDS_RAM : dp_ram_8x256
  PORT map(
    clka 	=> CLK,
    wea 		=> dds_ram_wr_en_s,
    addra 	=> dds_ram_wr_addr_s,
    dina 	=> dds_ram_wr_data_s,
    clkb 	=> CLK,
    addrb 	=> dds_ram_rd_addr_s,
    doutb 	=> dds_ram_rd_data_s
  );



DDS_BITRATE_GEN : phase_accumulator
	 Generic map(
				phase_accumulator_length  => 32
	 )
    Port map ( 
				CLK 				=> CLK,

				RESET				=> dds_phase_accumulator_reset_s,

				PHASE_ACCUMULATOR_LOAD_EN  => dds_phase_accumulator_load_en_s,
				PHASE_ACCUMULATOR 			=> dds_phase_accumulator_word_s,
				
				CLK_OUT 							=> open,
				PULSE_OUT						=> dds_bit_pulse_s
			  );


VALID_COMMAND_RX <= valid_command_rx_s;
INVALID_COMMAND_RX <= invalid_command_rx_s;
INCOMPLETE_COMMAND_RX <= incomplete_command_rx_s;

-- user led display
LED1 <= led_control_register_s(0);
LED2 <= led_control_register_s(1);
LED3 <= led_control_register_s(2);


-- time sync pulse synchro signal (named as fault signal)
FAULT <= 'Z'; -- Leave floating for now...

-- UART Reset
UART1_N_RST 		<= not RESET;
UART2_N_RST 		<= not RESET;

-- DDS Reset
DDS_MASTER_RESET 	<= RESET;

-- DDS spi signals mux
DDS_SDIO <= MOSI when dds_spi_mux_s = '0' else dds_spi_mosi_fpga_s;
DDS_SCLK <= SCK when dds_spi_mux_s = '0' else dds_spi_sck_fpga_s;
DDS_N_CS <= CS when dds_spi_mux_s = '0' else dds_spi_cs_fpga_s(0);

-- UART1 spi signals mux
UART1_MOSI <= MOSI when uart1_spi_mux_s = '0' else uart1_spi_mosi_fpga_s;
UART1_SCK <= SCK when uart1_spi_mux_s = '0' else uart1_spi_sck_fpga_s;
UART1_N_CS <= CS when uart1_spi_mux_s = '0' else uart1_spi_cs_fpga_s(0);

-- UART2 spi signals mux
UART2_MOSI <= MOSI when uart2_spi_mux_s = '0' else uart2_spi_mosi_fpga_s;
UART2_SCK <= SCK when uart2_spi_mux_s = '0' else uart2_spi_sck_fpga_s;
UART2_N_CS <= CS when uart2_spi_mux_s = '0' else uart2_spi_cs_fpga_s(0);

-- DDS IO configuration
DDS_OSK 				<= '0';
DDS_IO_RESET 		<= '0';
DDS_FPGA_CLK 		<= '0';

-- Enable internal 1.8V regulator
UART1_LDOEN 		<= '1';
UART2_LDOEN 		<= '1';

-- Debug signals
SPARE1 <= '0';
SPARE2 <= '0';
SPARE3 <= '0';
SPARE4 <= '0';
SPARE5 <= '0';
SPARE6 <= '0';
SPARE7 <= '0';
SPARE8 <= '0';

--SPARE1 <= dds_fifo_empty_flag_s;
--SPARE2 <= dds_fifo_rd_en_s;
--SPARE3 <= dds_fifo_wr_en_s;
--SPARE4 <= dds_wait_for_end_of_fifo_filling_s;
--SPARE5 <= dds_state_wait_for_time_s;
--SPARE6 <= dds_state_output_byte_s;
--SPARE7 <= dds_spi_master_busy_s;
--SPARE8 <= dds_spi_master_en_s;


process(CLK)
begin

	-- Update chip select on rising edge of the system clock only
	if(rising_edge(CLK) ) then
		if(RESET = '1') then
			-- Things to reset here
			MISO_SRC_SEL <= "00"; -- Select the FGPA as the source sel for the MISO
			dds_phase_accumulator_load_en_s <= '0';
			dds_state_s <= idle;
			dds_ram_wr_en_s(0) <= '0';
			dds_wait_for_end_of_ram_filling_s <= '0';
			dds_phase_accumulator_reset_s <= '0'; -- Release the phase accumulator now !
			uart1_time_ready_s <= '0';
			uart2_time_ready_s <= '0';
			dds_io_ud_clk_s <= '0';
			dds_data_rdy_to_send_s <= '0';
			dds_send_gain_zero_s <= '0';
			dds_send_continous_s <= '0';
			
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
			-- Nothing to do for now...
			sync_time_read_latch_s <= SYNC_TIME; -- Latch the time for a read in the spi command
		end if;

		-- UART 1 check to send
		if(uart1_time_ready_s = '1') then
			if(SYNC_TIME >= uart1_time_s) then	-- If time is over
				uart1_spi_master_tx_data_s <= "10001001000" & uart1_mode_s & "0000"; -- Build the command to clear the rx disable flag in thr righr register
				uart1_spi_master_en_s <= '1'; -- Go send the command
				uart1_time_ready_s <= '0'; -- Clear the time ready bits
			end if;
		else
			uart1_spi_master_en_s <= '0';
		end if;

		-- UART 2 check to send
		if(uart2_time_ready_s = '1') then
			if(SYNC_TIME >= uart2_time_s) then	-- If time is over
				uart2_spi_master_tx_data_s <= "10001001000" & uart2_mode_s & "0000"; -- Build the command to clear the rx disable flag in thr righr register
				uart2_spi_master_en_s <= '1'; -- Go send the command
				uart2_time_ready_s <= '0'; -- Clear the time ready bits
			end if;
		else
			uart2_spi_master_en_s <= '0';
		end if;

		-- FSK check to stop
		if(fsk_stop_time_ready_s = '1') then
			if(SYNC_TIME >= fsk_stop_time_s) then	-- If time is over
				fsk_stop_time_ready_s <= '0'; -- Clear the time ready bits
				dds_send_continous_s <= '0'; -- Clear the continous send bit now. The fsk signal will stop at next cycle end
			end if;
		end if;


		-- DDS state machine decoding of state to reduce timings constraints
		if(dds_state_s = idle) then
			DDS_IO_UD_CLK		<= not dds_spi_master_busy_s;	-- When state machine is IDLE, give the full control to the CPU on the DDS
		else
			DDS_IO_UD_CLK		<= dds_io_ud_clk_s; --not dds_spi_master_busy_s;
		end if;


		-- DDS state machine decoding of state to reduce timings constraints
		if(dds_state_s = wait_for_time) then
			dds_state_wait_for_time_s <= '1';
		else
			dds_state_wait_for_time_s <= '0';
		end if;
		
		-- DDS state machine decoding of state to reduce timings constraints
		if(dds_state_s = time_to_send) then
			dds_time_to_send_s <= '1';
		else
			dds_time_to_send_s <= '0';
		end if;

		-- DDS state machine state execution
		if(dds_state_wait_for_time_s = '1') then
			if(SYNC_TIME >= dds_tx_time_s and dds_wait_for_end_of_ram_filling_s = '0') then
					dds_state_s <= time_to_send;
			end if;
		end if;

		if(dds_time_to_send_s = '1') then
			dds_spi_master_tx_data_s <= "0000100" & dds_channel_gain(15) & "0000" & dds_channel_gain(11 downto 0); -- Build the SPI command
			dds_spi_master_en_s <= '1';
			dds_state_s <= apply_gain;
		end if;


		-- DDS Last byte detection
		if((dds_ram_rd_addr_s+1) >= dds_ram_wr_addr_s) then -- We have reach the max, mark this read as the last one
			dds_ram_last_byte_s <= '1';
		else
			dds_ram_last_byte_s <= '0';
		end if;


		-- Processing of the DDS fifo output
		CASE dds_state_s IS
			WHEN idle=> -- Wait here until data to extract from the fifo
				dds_io_ud_clk_s <= '0';
				dds_ram_rd_addr_s <= (others => '0');	-- reset read address
				dds_phase_accumulator_reset_s <= '1'; -- Reset the phase accumulator now !
				if(dds_data_rdy_to_send_s = '1' and dds_wait_for_end_of_ram_filling_s = '0') then
					dds_data_rdy_to_send_s <= '0';
					dds_state_s <= wait_for_time;
				end if;
				
			WHEN wait_for_time=>	-- Wait here until it's time to send the data
			
			WHEN time_to_send=>	-- Prepare to send now

			WHEN apply_gain=>
				dds_spi_master_en_s <= '0';
				dds_state_s <= apply_gain_d1;
				
			WHEN apply_gain_d1=>
				dds_state_s <= apply_gain_d2;
			WHEN apply_gain_d2=>
				if(dds_spi_master_busy_s = '0') then	-- Wait for the SPI send to complete
					--dds_state_s <= send_gain_zero;
					dds_state_s <= output_bit7;
					dds_send_gain_zero_s <= '1';
					dds_io_ud_clk_s <= '1';	-- Update the IO value
					dds_phase_accumulator_reset_s <= '0'; -- Release the phase accumulator now !
				end if;
				
			WHEN output_bit7=>
				dds_io_ud_clk_s <= '0';	
				DDS_FSK_BPSK_HOLD <= dds_ram_rd_data_s(7);
				if(dds_bit_pulse_s = '1') then
					if(dds_ram_last_byte_s = '1' and dds_bits_to_send_on_last_byte_s = conv_std_logic_vector(1,4)) then
						dds_state_s <= send_complete;
					else
						dds_state_s <= output_bit6;
					end if;
				end if;

				-- Prepare for the gain zero at the end of the buffer
				if(dds_send_gain_zero_s = '1') then
					dds_send_gain_zero_s <= '0';
					--dds_io_ud_clk_s <= '1';	-- Update the IO value if first bit of the buffer
					dds_spi_master_tx_data_s(11 downto 0) <= (others =>'0'); -- Build the SPI command
					dds_spi_master_en_s <= '1';
				else
					dds_spi_master_en_s <= '0';					
					--dds_io_ud_clk_s <= '0';	
				end if;


			WHEN output_bit6=>
				DDS_FSK_BPSK_HOLD <= dds_ram_rd_data_s(6);
				if(dds_bit_pulse_s = '1') then
					if(dds_ram_last_byte_s = '1' and dds_bits_to_send_on_last_byte_s = conv_std_logic_vector(2,4)) then
						dds_state_s <= send_complete;
					else
						dds_state_s <= output_bit5;
					end if;
				end if;

			WHEN output_bit5=>
				DDS_FSK_BPSK_HOLD <= dds_ram_rd_data_s(5);
				if(dds_bit_pulse_s = '1') then
					if(dds_ram_last_byte_s = '1' and dds_bits_to_send_on_last_byte_s = conv_std_logic_vector(3,4)) then
						dds_state_s <= send_complete;
					else
						dds_state_s <= output_bit4;
					end if;
				end if;

			WHEN output_bit4=>
				DDS_FSK_BPSK_HOLD <= dds_ram_rd_data_s(4);
				if(dds_bit_pulse_s = '1') then
					if(dds_ram_last_byte_s = '1' and dds_bits_to_send_on_last_byte_s = conv_std_logic_vector(4,4)) then
						dds_state_s <= send_complete;
					else
						dds_state_s <= output_bit3;
					end if;
				end if;

			WHEN output_bit3=>
				DDS_FSK_BPSK_HOLD <= dds_ram_rd_data_s(3);
				if(dds_bit_pulse_s = '1') then
					if(dds_ram_last_byte_s = '1' and dds_bits_to_send_on_last_byte_s = conv_std_logic_vector(5,4)) then
						dds_state_s <= send_complete;
					else
						dds_state_s <= output_bit2;
					end if;
				end if;

			WHEN output_bit2=>
				DDS_FSK_BPSK_HOLD <= dds_ram_rd_data_s(2);
				if(dds_bit_pulse_s = '1') then
					if(dds_ram_last_byte_s = '1' and dds_bits_to_send_on_last_byte_s = conv_std_logic_vector(6,4)) then
						dds_state_s <= send_complete;
					else
						dds_state_s <= output_bit1;
					end if;
				end if;
			WHEN output_bit1=>
				DDS_FSK_BPSK_HOLD <= dds_ram_rd_data_s(1);
				if(dds_bit_pulse_s = '1') then
					if(dds_ram_last_byte_s = '1' and dds_bits_to_send_on_last_byte_s = conv_std_logic_vector(7,4)) then
						dds_state_s <= send_complete;
					else
						dds_state_s <= output_bit0;
					end if;
				end if;
			WHEN output_bit0=>
				DDS_FSK_BPSK_HOLD <= dds_ram_rd_data_s(0);
				if(dds_bit_pulse_s = '1') then
					if(dds_ram_last_byte_s = '1') then	-- If it was the last byte
						dds_state_s <= send_complete;
					else
						dds_ram_rd_addr_s <= dds_ram_rd_addr_s + 1;	-- Increment the byte address of the ram
						dds_state_s <= output_bit7;
					end if;
				end if;

			WHEN send_complete=>
				if(dds_send_continous_s = '1') then
					-- Go start over the sending
					dds_ram_rd_addr_s <= (others => '0');	-- reset read address
					dds_state_s <= output_bit7;
				elsif(dds_spi_master_busy_s = '0') then	-- Wait for busy signal to goes low before go to IDLE
					dds_io_ud_clk_s <= '1';					-- Update output on DDS now !
					dds_state_s <= send_complete_d1;
				end if;
			WHEN send_complete_d1=>
				dds_io_ud_clk_s <= '0';
				dds_state_s <= idle;
			
		END CASE;


			if(dds_ram_wr_en_s(0) = '1') then
				dds_ram_wr_en_s(0) <= '0';		-- Clear 
				dds_ram_wr_addr_s <= dds_ram_wr_addr_s + 1; -- Increment the write address
			end if;

	
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
								config_register_s <= BYTE_RX;
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
								led_control_register_s <= BYTE_RX;
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

					WHEN "00000100" => -- Command 4 : Direct talk with DDS
							valid_command_completed_s <= '1';
							MISO_SRC_SEL <= "11"; -- Select the DDS as the source sel for the MISO				
							dds_spi_mux_s <= '0'; -- 0 = connected to CPU, 1 = connected to FPGA
							uart1_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
							uart2_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA

					WHEN "00000101" => -- Command 5 : Direct talk with UART1
							valid_command_completed_s <= '1';
							MISO_SRC_SEL <= "01"; -- Select the DDS as the source sel for the MISO				
							dds_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
							uart1_spi_mux_s <= '0'; -- 0 = connected to CPU, 1 = connected to FPGA
							uart2_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA

					WHEN "00000110" => -- Command 6 : Direct talk with UART2
							valid_command_completed_s <= '1';
							MISO_SRC_SEL <= "10"; -- Select the DDS as the source sel for the MISO				
							dds_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
							uart1_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
							uart2_spi_mux_s <= '0'; -- 0 = connected to CPU, 1 = connected to FPGA

					WHEN "00000111" => -- Command 7 : Fill FSK RAM
						-- Check if data received flag is set
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 1) then			-- Grab the channel programmation value
								dds_wait_for_end_of_ram_filling_s <= '1';
								dds_data_rdy_to_send_s <= '0';
								dds_ram_wr_addr_s <= (others => '0');		-- Reset the write address
								dds_channel_gain(15 downto 8) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 2) then		
								dds_channel_gain(7 downto 0) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 3) then		-- Grab the time when the send is required
								dds_tx_time_s(31 downto 24) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 4) then
								dds_tx_time_s(23 downto 16) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 5) then
								dds_tx_time_s(15 downto 8) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 6) then
								dds_tx_time_s(7 downto 0) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 7) then
								dds_bits_to_send_on_last_byte_s <= BYTE_RX(3 downto 0);
								dds_send_continous_s <= BYTE_RX(4);	-- Continous mode enable
							else														-- The rest of the data will be fsk bits
								dds_ram_wr_data_s <= BYTE_RX;
								dds_data_rdy_to_send_s <= '1';
								dds_ram_wr_en_s(0) <= '1';							-- Trig the write into the ram
							end if;
--						else
--							if(dds_ram_wr_en_s(0) = '1') then
--								dds_ram_wr_en_s(0) <= '0';		-- Clear 
--								dds_ram_wr_addr_s <= dds_ram_wr_addr_s + 1; -- Increment the write address
--							end if;
						end if; 
						
					
						-- always reply 0
						BYTE_TX <= (others => '0');

						-- Command check
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 7) then
								valid_command_completed_s <= '1';
							end if;
						end if;

					WHEN "00001000" => -- Command 8 : Setup FSK bitrate
						-- Check if data received flag is set
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 1) then			-- Grab the channel programmation value
								dds_phase_accumulator_word_s(31 downto 24) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								dds_phase_accumulator_word_s(23 downto 16) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 3) then
								dds_phase_accumulator_word_s(15 downto 8) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 4) then
								dds_phase_accumulator_word_s(7 downto 0) <= BYTE_RX;
								dds_phase_accumulator_load_en_s <= '1';
							end if;
						end if; 
						
						-- always reply 0
						BYTE_TX <= (others => '0');

						-- Command check
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 4) then
								valid_command_completed_s <= '1';
							end if;
						end if;


					WHEN "00001001" => -- Command 9 : Send UART1 fifo at time
						-- Check if data received flag is set
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 1) then			-- Grab the channel programmation value
								uart1_time_s(31 downto 24) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								uart1_time_s(23 downto 16) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 3) then
								uart1_time_s(15 downto 8) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 4) then
								uart1_time_s(7 downto 0) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 5) then
								uart1_mode_s <= BYTE_RX(0);
								uart1_time_ready_s <= '1';
							end if;
						end if; 
						
						-- always reply 0
						BYTE_TX <= (others => '0');

						-- Command check
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 5) then
								valid_command_completed_s <= '1';
							end if;
						end if;

					WHEN "00001010" => -- Command 10 (0x0a) : Send UART2 fifo at time
						-- Check if data received flag is set
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 1) then			-- Grab the channel programmation value
								uart2_time_s(31 downto 24) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								uart2_time_s(23 downto 16) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 3) then
								uart2_time_s(15 downto 8) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 4) then
								uart2_time_s(7 downto 0) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 5) then
								uart2_mode_s <= BYTE_RX(0);
								uart2_time_ready_s <= '1';
							end if;
						end if; 
						
						-- always reply 0
						BYTE_TX <= (others => '0');

						-- Command check
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 5) then
								valid_command_completed_s <= '1';
							end if;
						end if;


					WHEN "00001011" => -- Command 11 (0x0b) : Stop FSK signal at time
						-- Check if data received flag is set
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 1) then			-- Grab the channel programmation value
								fsk_stop_time_s(31 downto 24) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 2) then
								fsk_stop_time_s(23 downto 16) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 3) then
								fsk_stop_time_s(15 downto 8) <= BYTE_RX;
							elsif (unsigned(BYTE_RX_COUNT) = 4) then
								fsk_stop_time_s(7 downto 0) <= BYTE_RX;
								fsk_stop_time_ready_s <= '1';
							end if;
						end if; 
						
						-- always reply 0
						BYTE_TX <= (others => '0');

						-- Command check
						if(BYTE_RX_READY_PULSE ='1') then
							if(unsigned(BYTE_RX_COUNT) = 4) then
								valid_command_completed_s <= '1';
							end if;
						end if;

					WHEN OTHERS =>
						MISO_SRC_SEL <= "00"; -- Select the FGPA as the source sel for the MISO				
						dds_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
						uart1_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
						uart2_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
						BYTE_TX <= (others => '0');
						invalid_command_detected_s <= '1';
						dds_phase_accumulator_load_en_s <= '0';
					
				END CASE;
			else -- if(COMMAND_READY = '1') then
				MISO_SRC_SEL <= "00"; -- Select the FGPA as the source sel for the MISO				
				dds_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
				uart1_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
				uart2_spi_mux_s <= '1'; -- 0 = connected to CPU, 1 = connected to FPGA
				dds_phase_accumulator_load_en_s <= '0';
				dds_wait_for_end_of_ram_filling_s <= '0';

			end if; -- if(COMMAND_READY = '1') then
		end if; -- if(RESET = '1') then
	end if; -- if(rising_edge(CLK) ) then

end process;
end Behavioral;

