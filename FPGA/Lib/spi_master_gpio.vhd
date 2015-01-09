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

entity spi_master_gpio is
	 Generic (
		byte_size : positive := 8,
		chip_select_count : positive := 2
		);
    Port ( 
				-- System clock
				CLK : in  STD_LOGIC;
				
				-- SPI clock divider
				SPI_CLK_DIVIDER : in STD_LOGIC_VECTOR(4 downto 0);			-- Binary divider of CLK SRC
				
				-- Phase and polarity
				CPOL : in STD_LOGIC;
				CPHA : in STD_LOGIC;
				
				-- SPI Interface
				MOSI : out STD_LOGIC;
				MISO : in STD_LOGIC;
				SCK : out STD_LOGIC;
				CS : out STD_LOGIC_VECTOR(chip_select_count-1 downto 0);
				
				-- Parallel interface
				CS_SELECT : in STD_LOGIC_VECTOR(2 downto 0);					-- Chip select selector
				BYTE_TX : in STD_LOGIC_VECTOR(byte_size-1 downto 0);		-- Byte to send
				BYTE_RX : out STD_LOGIC_VECTOR(byte_size-1 downto 0);		-- Byte received
				BYTE_LOAD_PULSE : out STD_LOGIC;									-- Pulse when time to load/latch byte
				BYTE_TO_SEND_COUNT : in STD_LOGIC_VECTOR(4 downto 0);		-- Number of byte to send (0 to 31)
				SEND_TRIG : in STD_LOGIC;											-- Send trig signal
				BUSY : out STD_LOGIC													-- Busy indicator
				);
end spi_master_gpio;

architecture Behavioral of spi_master_gpio is

-- SPI clock divider


-- Shift register
signal out_shift_reg_s : STD_LOGIC_VECTOR(byte_size-1 downto 0);
signal in_shift_reg_s : STD_LOGIC_VECTOR(byte_size-1 downto 0);

TYPE STATE_TYPE IS (init,outputByte);
SIGNAL state_s   : STATE_TYPE;


begin

process(CLK)
begin

	-- Update chip select on rising edge of the system clock only
	if(rising_edge(CLK) ) then
		
	end if; -- if(rising_edge(CLK) ) then

end process;
end Behavioral;

