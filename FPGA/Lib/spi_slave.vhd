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

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity spi_slave is
	 Generic (
		bitcount : positive := 16
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
				parallelOut : out STD_LOGIC_VECTOR(bitcount-1 downto 0);
				parallelIn : in STD_LOGIC_VECTOR(bitcount-1 downto 0)
				);
end spi_slave;

architecture Behavioral of spi_slave is

-- Chip select flag and buffer
signal cs_d_s : STD_LOGIC_VECTOR(2 downto 0);
signal cs_falling_edge : STD_LOGIC;
signal cs_raising_edge : STD_LOGIC;
signal cs_s : STD_LOGIC;

-- SCK flag and buffer
signal sck_d_s : STD_LOGIC_VECTOR(2 downto 0);
signal sck_falling_edge : STD_LOGIC;
signal sck_raising_edge : STD_LOGIC;
signal sck_s : STD_LOGIC;

-- Shift register
signal out_shift_reg_s : STD_LOGIC_VECTOR(bitcount-1 downto 0);
signal in_shift_reg_s : STD_LOGIC_VECTOR(bitcount-1 downto 0);


begin

process(CLK)
begin

	-- Update chip select on rising edge of the system clock only
	if(rising_edge(CLK) ) then
		
		-- Buf the CS signal to avoid metastability
		cs_d_s(0) <= CS;
		cs_d_s(1) <= cs_d_s(0);
		cs_d_s(2) <= cs_d_s(1);
		
		-- Falling edge detect of CS
		if(cs_d_s(2) = '0' and cs_d_s(1) = '1') then
			cs_falling_edge <= '1';
		else
			cs_falling_edge <= '0';
		end if;
		
		-- Raising edge detect of CS
		if(cs_d_s(2) = '1' and cs_d_s(1) = '0') then
			cs_raising_edge <= '1';
		else
			cs_raising_edge <= '0';
		end if;


		-- Buf the SCK signal to avoid metastability
		sck_d_s(0) <= SCK;
		sck_d_s(1) <= sck_d_s(0);
		sck_d_s(2) <= sck_d_s(1);
		
		-- Falling edge detect of SCK
		if(sck_d_s(2) = '0' and sck_d_s(1) = '1') then
			sck_falling_edge <= '1';
		else
			sck_falling_edge <= '0';
		end if;
		
		-- Raising edge detect of SCK
		if(sck_d_s(2) = '1' and sck_d_s(1) = '0') then
			sck_raising_edge <= '1';
		else
			sck_raising_edge <= '0';
		end if;

		
		-- Input shift register
		
		if(cs = '0') then
			if(sck_raising_edge = '1') then
				out_shift_reg_s <= MOSI & out_shift_reg_s((bitcount - 1) downto 1);
			end if;
			
			MISO <= MOSI;
			
		else
			parallelOut <= out_shift_reg_s;
			
			MISO <= 'Z';
			
		end if;
		--MOSI
		
		
		
		
	end if;

end process;
end Behavioral;

