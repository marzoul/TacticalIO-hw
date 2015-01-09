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
--use ieee.std_logic_unsigned.all;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;


-- this spi slave component is design to be used with CPOL=0 et CPHA=0

entity spi_slave_gpio is
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
end spi_slave_gpio;

architecture Behavioral of spi_slave_gpio is

-- Chip select flag and buffer
signal cs_d_s : STD_LOGIC_VECTOR(2 downto 0);
signal cs_falling_edge_s : STD_LOGIC;
signal cs_raising_edge_s : STD_LOGIC;
signal cs_s : STD_LOGIC;

-- SCK flag and buffer
signal sck_d_s : STD_LOGIC_VECTOR(2 downto 0);
signal sck_falling_edge_s : STD_LOGIC;
signal sck_raising_edge_s : STD_LOGIC;
signal sck_s : STD_LOGIC;

-- MOSI buffering and delay to fit with SCK
signal mosi_d_s : STD_LOGIC_VECTOR(3 downto 0);

-- Shift register
signal out_shift_reg_s : STD_LOGIC_VECTOR(byte_size-1 downto 0);
signal in_shift_reg_s : STD_LOGIC_VECTOR(byte_size-1 downto 0);

TYPE STATE_TYPE IS (init,outputByte);
SIGNAL state_s   : STATE_TYPE;

signal bitcount_s : UNSIGNED(7 downto 0); 		--bit counter
signal byte_count_s : UNSIGNED(7 downto 0); 		-- byte counter
signal byte_rx_ready_pulse_s : STD_LOGIC;
-- Signal to load the data from the parallel interface (this signal is generated internally)
signal byte_tx_load_d_s : STD_LOGIC_VECTOR(5 downto 0);
signal miso_s : STD_LOGIC;

signal sck_filter_s : STD_LOGIC;
signal sck_filter_d_s : STD_LOGIC;

signal cs_filter_s : STD_LOGIC;
signal cs_filter_d_s : STD_LOGIC;
begin

-- MISO pin
MISO <= 'Z' when (CS = '1')else miso_s;

-- Assign the byte_count_s to the output
BYTE_RX_COUNT <= conv_std_logic_vector(byte_count_s,8);
BYTE_RX_READY_PULSE <= byte_rx_ready_pulse_s;
process(CLK)
begin

	-- Update chip select on rising edge of the system clock only
	if(rising_edge(CLK) ) then
		
		-- Buf the CS signal to avoid metastability
		cs_d_s(0) <= CS;
		cs_d_s(1) <= cs_d_s(0);
		cs_d_s(2) <= cs_d_s(1);
		
		-- CS filter
		cs_filter_s <= cs_d_s(0) or cs_d_s(1) or cs_d_s(2);
		cs_filter_d_s <= cs_filter_s;
		
		miso_s <= out_shift_reg_s(byte_size-1);
		
		-- Falling edge detect of CS
		if(cs_filter_s = '0' and cs_filter_d_s = '1') then
			cs_falling_edge_s <= '1';
		else
			cs_falling_edge_s <= '0';
		end if;
		
		-- Raising edge detect of CS
		if(cs_filter_s = '1' and cs_filter_d_s = '0') then
			cs_raising_edge_s <= '1';
		else
			cs_raising_edge_s <= '0';
		end if;


		-- Buf the SCK signal to avoid metastability
		sck_d_s(0) <= SCK;
		sck_d_s(1) <= sck_d_s(0);
		sck_d_s(2) <= sck_d_s(1);

		-- filter the SCK a little bit
		sck_filter_s <= sck_d_s(0) or sck_d_s(1) or sck_d_s(2);
		sck_filter_d_s <= sck_filter_s;
		
		-- MOSI delay
		mosi_d_s(0) <= MOSI;
		mosi_d_s(1) <= mosi_d_s(0);
		mosi_d_s(2) <= mosi_d_s(1);
		mosi_d_s(3) <= mosi_d_s(2);
		
		
		-- Falling edge detect of SCK (Do not add to much delay here, or the data will not be read at time when the master read it)
		if(sck_filter_s = '0' and sck_filter_d_s = '1') then
			sck_falling_edge_s <= '1';
		else
			sck_falling_edge_s <= '0';
		end if;
		
		-- Raising edge detect of SCK
		if(sck_filter_s = '1' and sck_filter_d_s = '0') then
			sck_raising_edge_s <= '1';
		else
			sck_raising_edge_s <= '0';
		end if;

		
		-- Init state machine on CS low detect ad init bitcount
		if(cs_falling_edge_s = '1') then
			state_s <= init;
			byte_count_s <= (others => '0'); -- init byte counter
		end if;

		-- Manage the MISO output
		--if(cs_s = '1') then
--			MISO <= 'Z';
		--else
--			MISO <= out_shift_reg_s(byte_size-1);
		--end if;

		-- Generate a pulse to load the data from the parallel interface on two condition :
		-- 1 : If a falling edge on SCK and end of byte
		-- 2 : on Cs falling edge
		if((sck_falling_edge_s = '1' and bitcount_s >= byte_size) or cs_falling_edge_s = '1') then
			byte_tx_load_d_s(0) <= '1';
		else
			byte_tx_load_d_s(0) <= '0';
		end if;
		
		-- Delay array
		byte_tx_load_d_s(1) <= byte_tx_load_d_s(0);
		byte_tx_load_d_s(2) <= byte_tx_load_d_s(1);
		byte_tx_load_d_s(3) <= byte_tx_load_d_s(2);
		byte_tx_load_d_s(4) <= byte_tx_load_d_s(3);
		byte_tx_load_d_s(5) <= byte_tx_load_d_s(4);

		-- Load the TX data in shift register on rising edge of tx load signal
		if(byte_tx_load_d_s(5) = '1') then
			out_shift_reg_s <= BYTE_TX;		-- Latch the tx byte register
		end if;
		
		-- Transmission state machine
		if(cs_filter_s = '0') then

			-- Increment the byte counter one clock after the byte ready pulse (to leave time to read the correct value
			if(byte_rx_ready_pulse_s = '1') then
				byte_count_s <= byte_count_s + '1';		-- increment the byte count
			end if;

			CASE state_s IS

				WHEN init=>
				
					state_s <= outputByte; 				-- Process to next byte
					bitcount_s <= (others => '0'); 	-- init bit counter
					byte_rx_ready_pulse_s <= '0';		-- Clear the byte ready flag

				WHEN outputByte=>

					-- Raising edge of SCK, new data is comming
					if(sck_raising_edge_s = '1') then
						--in_shift_reg_s <= in_shift_reg_s(byte_size-2 downto 0) & MOSI;		-- get the data from the master
						in_shift_reg_s <= in_shift_reg_s(byte_size-2 downto 0) & mosi_d_s(3);	-- get the data from the master
						out_shift_reg_s <= out_shift_reg_s(byte_size-2 downto 0) & '0';	-- Reply with the content from the tx register
						bitcount_s <= bitcount_s + '1';												-- Increment the bit counter
					end if;
					
					-- Falling edge of SCK
					if(sck_falling_edge_s = '1') then
						-- shift output data here
						--out_shift_reg_s <= out_shift_reg_s(byte_size-2 downto 0) & '0';	-- Reply with the content from the tx register

						-- If the bit count is over the byte size
						if( bitcount_s >= byte_size) then
							state_s <= init;									-- Return to load init
							BYTE_RX <= in_shift_reg_s;					-- Publish the shift register to the byte out
							byte_rx_ready_pulse_s <= '1';				-- Publish the data ready flag
						end if;
					end if;
					
			END CASE;
		end if;
	end if;

end process;
end Behavioral;

