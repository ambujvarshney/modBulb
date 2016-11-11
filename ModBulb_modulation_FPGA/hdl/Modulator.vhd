library ieee;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.Command_Pkg.all;

entity Modulator is
port (
	-- system clock
	CLK         : in std_logic;
	-- system reset - active low
    RESET_N     : in std_logic := '0';
	-- memory read enable
    MEM_RE		: out std_logic := '0';
	-- memory data output
    MEM_IN  	: in word_t := (others => '0');
	-- memory enable
	MEM_WE      : buffer std_logic := '0';
	-- RAM address
	MEM_ADDR    : buffer std_logic_vector(MEM_ADDR_WIDTH - 1 downto 0) := (others => '0');
	-- spi read ack
	SPI_DVLD	: in std_logic := '0';
	-- spi data output
	SPI_IN		: in word_t := (others => '0');
	-- signal output
	SIGNAL_OUT  : out std_logic := DEF_LI_VAL
);
end Modulator;
architecture architecture_Modulator of Modulator is
	-- SPI_DVLD handled
	signal spi_dvld_handled : std_logic := '0';
	-- registers to store modulation parameters
	signal registers : registers_t;
	-- control register (bits 0-2 store modulation scheme and bit 3 indicates if last word is 16-bit or 8-bit)
	signal ctrl_reg : std_logic_vector(3 downto 0) := (others => '0');
	-- the address of the current register that's awaiting new data
	signal reg_addr  : integer range 0 to PARAMS_CNT - 1 := 0;
	-- memory write enable
	signal mem_wen : std_logic := '0';
	-- the address of where to store the current word in memory
	signal recv_addr : word_t := (others => '0');
	-- the address of which word to read from memory
	signal mod_addr : word_t := (others => '0');
	-- enable the modulation
	signal mod_enabled : std_logic := '0';
	-- modulation is currently running
	signal mod_active : std_logic := '0';
	-- indicates that the last bit in a packet is sent
	signal last_bit_sent : std_logic := '0';
	-- the index of the next bit to transmit
	signal bit_idx : integer range 0 to WORD_SIZE - 1 := 0;
	-- the index of the next PPM slot
	signal ppm_slot_idx : integer range 0 to 2 ** BYTE_SIZE - 1 := 0;
	-- the number of the sent packets
	signal packet_cntr : integer range 0 to 2 ** WORD_SIZE - 1 := 0;
	-- signal value
	signal sig_o : std_logic := DEF_LI_VAL;
	
	-- timers
	-- timer A periodic/one-shot
	-- reload value
	signal timA_rld: integer range 0 to 2 ** WORD_SIZE - 1:= 0;
	-- counter
	signal timA_ctr: integer range 0 to 2 ** WORD_SIZE - 1:= 0;
	-- enable flag
	signal timA_en : std_logic := '0';
	-- timeout flag
	signal timA_to : std_logic := '0';
	-- timer is decremented once every 1ms
	signal timA_mod_ms : std_logic := '0';
	-- 1ms timer
	signal timA_ms_ctr : integer range 0 to TICK_MS;
	-- timer B PWM
	-- reload value
	signal timB_rld: integer range 0 to 2 ** WORD_SIZE - 1:= 0;
	-- timer match value
	signal timB_mch: integer range 0 to 2 ** WORD_SIZE - 1:= 0;
	-- counter
	signal timB_ctr: integer range 0 to 2 ** WORD_SIZE - 1:= 0;
	-- enable flag
	signal timB_en : std_logic := '0';
	-- PWM output
	signal timB_out: std_logic := '0';
begin
			
	process(CLK, RESET_N)
	begin
		if RESET_N = '0' then
		-- reset signals
			reg_addr <= 0;
			mem_wen <= '0';
			mod_enabled <= '0';
			mod_active <= '0';
			timA_en  <= '0';
			timA_to  <= '0';
			timB_en  <= '0';
			timB_out <= DEF_LI_VAL;
		elsif rising_edge(CLK) then
			
			-- spi_dvld_handled logic
			if SPI_DVLD = '1' and spi_dvld_handled = '0' then
				spi_dvld_handled <= '1';
			elsif SPI_DVLD = '0' then
				spi_dvld_handled <= '0';
			end if;
			
			-- SPI reception handler
			if SPI_DVLD = '1' and spi_dvld_handled = '0' then
				
				if recv_addr = std_logic_vector(unsigned(registers(REG_LEN)) - 1) then
					mod_enabled <= '1';
				elsif mod_enabled = '1' then
					mod_enabled <= '0';
					mod_active <= '0';
				end if;
			
				if recv_addr = std_logic_vector(unsigned(registers(REG_LEN)) - 1) then
					mem_wen <= '0';
				elsif reg_addr = PARAMS_CNT - 1 then
					mem_wen <= '1';
				end if;
			
				if mem_wen = '0' then
					case reg_addr is
					when REG_BIR => 
						timA_rld <= to_integer(unsigned(SPI_IN));
					when REG_CTRL =>
						ctrl_reg <= SPI_IN(3 downto 0);
					when others =>
						registers(reg_addr - (PARAMS_CNT - REGS_CNT)) <= SPI_IN;
					end case;
					if reg_addr < PARAMS_CNT - 1 then
						reg_addr <= reg_addr + 1;
					else
						reg_addr <= 0;
					end if;
				end if;
				
				if mem_wen = '1' and recv_addr < std_logic_vector(unsigned(registers(REG_LEN)) - 1) then
					recv_addr <= std_logic_vector(unsigned(recv_addr) + 1);
				else		
					recv_addr <= (others => '0');
				end if;
			end if;
			
			-- timA logic
			if timA_en = '1' then
				if timA_ctr > 0 then
					if timA_mod_ms = '1' then
						if timA_ms_ctr < TICK_MS then
							timA_ms_ctr <= timA_ms_ctr + 1;
						else
							timA_ms_ctr <= 0;
							timA_ctr <= timA_ctr - 1;
						end if;
					else
						timA_ctr <= timA_ctr - 1;
					end if;
				else
					timA_to <= '1';
					timA_ctr <= timA_rld;
					if timA_mod_ms = '1' then
						timA_mod_ms <= '0';
					end if;
				end if;
			end if;
			
			-- timB logic
			if timB_en = '1' then
				if timB_ctr = 0 then
					timB_out <= DEF_LI_VAL;
				elsif timB_ctr = timB_mch then
					timB_out <= not DEF_LI_VAL;
				end if;
			
				if timB_ctr = 0 then
					timB_ctr <= timB_rld;
				else
					timB_ctr <= timB_ctr - 1;
				end if;
			end if;
			
			-- modulation logic
			if mod_enabled = '1' then
				
				-- common modulation logic
				if timA_to = '1' then
					timA_to <= '0';
				end if;
				
				if timA_en = '0' and mod_active = '1' then
					-- set delay to start modulating again afterwards
					timA_ctr <= to_integer(unsigned(registers(REG_DLY)));
					timA_ms_ctr <= 0;
					timA_mod_ms <= '1';
					timA_en <= '1';
					bit_idx <= 0;
					mod_addr <= (others => '0');
					ppm_slot_idx <= 0;
				end if;
			
				if mod_active = '0' then
					-- initialize signals
					mod_active <= '1';
					bit_idx <= 0;
					mod_addr <= (others => '0');
					ppm_slot_idx <= 0;
					packet_cntr <= 0;
					timA_mod_ms <= '0';
					
					timA_ctr <= timA_rld;
					timA_en <= '1';
					timA_to <= '0';
				elsif timA_to = '1' and last_bit_sent = '1' then
					-- deinitialize signals
					packet_cntr <= packet_cntr + 1;
					timA_en <= '0';
					timB_en <= '0';
					if packet_cntr = to_integer(unsigned(registers(REG_CNT))) - 1 and registers(REG_CNT) /= (registers(REG_CNT)'range => '0') then
						mod_active <= '0';
						mod_enabled <= '0';
					end if;
				end if;
				
				-- OOK modulator
				if mod_active = '1' and timA_to = '1' and ctrl_reg(2 downto 0) = MOD_OOK then
					if last_bit_sent = '0' then
						sig_o <= MEM_IN(WORD_SIZE - 1 - bit_idx);
						if bit_idx < WORD_SIZE - 1 then
							bit_idx <= bit_idx + 1;
						else
							bit_idx <= 0;
							mod_addr <= std_logic_vector(unsigned(mod_addr) + 1);
						end if;
					else
						sig_o <= DEF_LI_VAL;
					end if;
				end if;
				
				
				-- BFSK modulator
				if mod_active = '1' and timA_to = '1' and ctrl_reg(2 downto 0) = MOD_BFSK then
					if last_bit_sent = '0' then
						if timB_en = '0' then
							timB_en <= '1';
						end if;
						if MEM_IN(WORD_SIZE - 1 - bit_idx) = '1' then
							timB_rld <= to_integer(unsigned(registers(REG_FR2)));
							timB_mch <= to_integer(unsigned(registers(REG_MT2)));
						else 
							timB_rld <= to_integer(unsigned(registers(REG_FR1)));
							timB_mch <= to_integer(unsigned(registers(REG_MT1)));
						end if;
						if bit_idx < WORD_SIZE - 1 then
							bit_idx <= bit_idx + 1;
						else
							bit_idx <= 0;
							mod_addr <= std_logic_vector(unsigned(mod_addr) + 1);
						end if;
					end if;
				end if;
					
				-- PPM modulator
				if mod_active = '1' and timA_to = '1' and ctrl_reg(2 downto 0) = MOD_PPM then
					if last_bit_sent = '0' then
						if to_integer(unsigned(registers(REG_BS))) = 1 then
							if (MEM_IN(15 - bit_idx) = '0' and ppm_slot_idx = 0) or (MEM_IN(15 - bit_idx) = '1' and ppm_slot_idx = 1) then
								sig_o <= not DEF_LI_VAL;
							else
								sig_o <= DEF_LI_VAL;
							end if;
						elsif to_integer(unsigned(registers(REG_BS))) = 2 then
							if (bit_idx = 0 and to_integer(unsigned(MEM_IN(15 downto 14))) = ppm_slot_idx) or
									(bit_idx = 2 and to_integer(unsigned(MEM_IN(13 downto 12))) = ppm_slot_idx) or
									(bit_idx = 4 and to_integer(unsigned(MEM_IN(11 downto 10))) = ppm_slot_idx) or
									(bit_idx = 6 and to_integer(unsigned(MEM_IN(9 downto 8))) = ppm_slot_idx) or
									(bit_idx = 8 and to_integer(unsigned(MEM_IN(7 downto 6))) = ppm_slot_idx) or
									(bit_idx = 10 and to_integer(unsigned(MEM_IN(5 downto 4))) = ppm_slot_idx) or
									(bit_idx = 12 and to_integer(unsigned(MEM_IN(3 downto 2))) = ppm_slot_idx) or
									(bit_idx = 14 and to_integer(unsigned(MEM_IN(1 downto 0))) = ppm_slot_idx) then
								sig_o <= not DEF_LI_VAL;
							else
								sig_o <= DEF_LI_VAL;
							end if;
						elsif to_integer(unsigned(registers(REG_BS))) = 4 then
							if (bit_idx = 0 and to_integer(unsigned(MEM_IN(15 downto 12))) = ppm_slot_idx) or
									(bit_idx = 4 and to_integer(unsigned(MEM_IN(11 downto 8))) = ppm_slot_idx) or 
									(bit_idx = 8 and to_integer(unsigned(MEM_IN(7 downto 4))) = ppm_slot_idx) or 
									(bit_idx = 12 and to_integer(unsigned(MEM_IN(3 downto 0))) = ppm_slot_idx) then
								sig_o <= not DEF_LI_VAL;
							else
								sig_o <= DEF_LI_VAL;
							end if;
						elsif to_integer(unsigned(registers(REG_BS))) = 8 then
							if (bit_idx = 0 and to_integer(unsigned(MEM_IN(15 downto 8))) = ppm_slot_idx) or
									(bit_idx = 8 and to_integer(unsigned(MEM_IN(7 downto 0))) = ppm_slot_idx) then
								sig_o <= not DEF_LI_VAL;
							else
								sig_o <= DEF_LI_VAL;
							end if;
						end if;
						
					
						if ppm_slot_idx < to_integer(unsigned(registers(REG_SLT))) - 1 then
							ppm_slot_idx <= ppm_slot_idx + 1;
						else
							ppm_slot_idx <= 0;
							if bit_idx < WORD_SIZE - to_integer(unsigned(registers(REG_BS))) then
								bit_idx <= bit_idx + to_integer(unsigned(registers(REG_BS)));
							else
								bit_idx <= 0;
								mod_addr <= std_logic_vector(unsigned(mod_addr) + 1);
							end if;
						end if;
					else
						sig_o <= DEF_LI_VAL;
					end if;
				end if;
			end if;
		end if;
	end process;
	
	MEM_RE <= not mod_enabled;
	MEM_WE <= mem_wen;
	MEM_ADDR <= recv_addr(MEM_ADDR_WIDTH - 1 downto 0) when mem_wen = '1' else 
				mod_addr(MEM_ADDR_WIDTH - 1 downto 0) when mod_enabled = '1' else (MEM_ADDR'range => '0');
	SIGNAL_OUT <= DEF_LI_VAL when timA_en /= '1' or (ctrl_reg(2 downto 0) = MOD_BFSK and timB_en /= '1') else 
								timB_out when ctrl_reg(2 downto 0) = MOD_BFSK else
								sig_o;
	last_bit_sent <= '1' when ((ctrl_reg(3) = '0' and mod_addr = registers(REG_LEN)) or 
						(ctrl_reg(3) = '1' and mod_addr = std_logic_vector(unsigned(registers(REG_LEN)) - 1)
						and bit_idx = BYTE_SIZE)) else '0';
					
					
	
end architecture_Modulator;
