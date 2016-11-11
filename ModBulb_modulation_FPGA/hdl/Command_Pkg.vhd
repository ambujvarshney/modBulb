

library IEEE;

use IEEE.std_logic_1164.all;

package Command_Pkg is
	constant BYTE_SIZE		: integer := 8;
	constant BYTES_IN_WORD  : integer := 2;
	constant WORD_SIZE   	: integer := BYTES_IN_WORD * BYTE_SIZE;
	constant MEM_ADDR_WIDTH	: integer := 9;
    constant CMD_IND     	: std_logic_vector(BYTE_SIZE - 1 downto 0) := x"EA";
	
	constant REG_BIR		: integer := 0;
	constant REG_CTRL		: integer := 1;
	constant REG_LEN		: integer := 0;
	constant REG_CNT		: integer := 1;
	constant REG_DLY		: integer := 2;
	
	constant REG_FR1		: integer := 3;
	constant REG_FR2		: integer := 4;
	constant REG_MT1		: integer := 5;
	constant REG_MT2		: integer := 6;
	
	constant REG_SLT		: integer := 3;
	constant REG_BS			: integer := 4;
	
	constant MOD_OOK		: std_logic_vector(2 downto 0) := "001";
	constant MOD_BFSK		: std_logic_vector(2 downto 0) := "010";
	constant MOD_PPM		: std_logic_vector(2 downto 0) := "011";
	
	
	
	constant REGS_CNT		: integer := 7;
	constant PARAMS_CNT		: integer := 9;
	
	constant DEF_LI_VAL		: std_logic := '0';
	
	constant TICK_MS		: integer := 20000;
	
	subtype byte_t is std_logic_vector(BYTE_SIZE - 1 downto 0);
	subtype word_t is std_logic_vector(WORD_SIZE - 1 downto 0);
	
	type registers_t is array(REGS_CNT - 1 downto 0) of word_t;
end Command_Pkg;