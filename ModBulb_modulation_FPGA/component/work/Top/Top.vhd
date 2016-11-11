----------------------------------------------------------------------
-- Created by SmartDesign Fri Nov 11 11:39:45 2016
-- Version: v11.7 11.7.0.119
----------------------------------------------------------------------

----------------------------------------------------------------------
-- Libraries
----------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

library igloo;
use igloo.all;
library work;
use work.Command_Pkg.all;
use work.Command_Pkg.all;
----------------------------------------------------------------------
-- Top entity declaration
----------------------------------------------------------------------
entity Top is
    -- Port list
    port(
        -- Inputs
        CLK        : in  std_logic;
        RESET_N    : in  std_logic;
        spi_mosi_i : in  std_logic;
        spi_sck_i  : in  std_logic;
        spi_ssel_i : in  std_logic;
        -- Outputs
        MOD_EN     : out std_logic;
        SIGNAL_OUT : out std_logic;
        spi_miso_o : out std_logic
        );
end Top;
----------------------------------------------------------------------
-- Top architecture body
----------------------------------------------------------------------
architecture RTL of Top is
----------------------------------------------------------------------
-- Component declarations
----------------------------------------------------------------------
-- AND2
component AND2
    -- Port list
    port(
        -- Inputs
        A : in  std_logic;
        B : in  std_logic;
        -- Outputs
        Y : out std_logic
        );
end component;
-- Modulator
component Modulator
    -- Port list
    port(
        -- Inputs
        CLK        : in  std_logic;
        MEM_IN     : in  word_t;
        RESET_N    : in  std_logic;
        SPI_DVLD   : in  std_logic;
        SPI_IN     : in  word_t;
        -- Outputs
        MEM_ADDR   :  buffer std_logic_vector(8 downto 0) ;
        MEM_RE     : out std_logic;
        MEM_WE     :  buffer std_logic ;
        SIGNAL_OUT : out std_logic
        );
end component;
-- RAM
component RAM
    -- Port list
    port(
        -- Inputs
        RADDR : in  std_logic_vector(8 downto 0);
        REN   : in  std_logic;
        RESET : in  std_logic;
        RWCLK : in  std_logic;
        WADDR : in  std_logic_vector(8 downto 0);
        WD    : in  std_logic_vector(15 downto 0);
        WEN   : in  std_logic;
        -- Outputs
        RD    : out std_logic_vector(15 downto 0)
        );
end component;
-- spi_slave
component spi_slave
    -- Port list
    port(
        -- Inputs
        clk_i      : in  std_logic;
        di_i       : in  std_logic_vector(15 downto 0);
        spi_mosi_i : in  std_logic;
        spi_sck_i  : in  std_logic;
        spi_ssel_i : in  std_logic;
        wren_i     : in  std_logic;
        -- Outputs
        di_req_o   : out std_logic;
        do_o       : out std_logic_vector(15 downto 0);
        do_valid_o : out std_logic;
        spi_miso_o : out std_logic;
        wr_ack_o   : out std_logic
        );
end component;
----------------------------------------------------------------------
-- Signal declarations
----------------------------------------------------------------------
signal AND2_0_Y               : std_logic;
signal MOD_EN_net_0           : std_logic;
signal Modulator_0_MEM_ADDR   : std_logic_vector(8 downto 0);
signal Modulator_0_MEM_WE     : std_logic;
signal RAM_0_RD               : std_logic_vector(15 downto 0);
signal SIGNAL_OUT_net_0       : std_logic;
signal spi_miso_o_net_0       : std_logic;
signal spi_slave_0_do_o       : std_logic_vector(15 downto 0);
signal spi_slave_0_do_valid_o : std_logic;
signal SIGNAL_OUT_net_1       : std_logic;
signal MOD_EN_net_1           : std_logic;
signal spi_miso_o_net_1       : std_logic;
signal MEM_IN_0               : word_t;
signal SPI_IN_0               : word_t;
----------------------------------------------------------------------
-- TiedOff Signals
----------------------------------------------------------------------
signal di_i_const_net_0       : std_logic_vector(15 downto 0);
signal GND_net                : std_logic;

begin
----------------------------------------------------------------------
-- Constant assignments
----------------------------------------------------------------------
 di_i_const_net_0 <= B"0000000000000000";
 GND_net          <= '0';
----------------------------------------------------------------------
-- Top level output port assignments
----------------------------------------------------------------------
 SIGNAL_OUT_net_1 <= SIGNAL_OUT_net_0;
 SIGNAL_OUT       <= SIGNAL_OUT_net_1;
 MOD_EN_net_1     <= MOD_EN_net_0;
 MOD_EN           <= MOD_EN_net_1;
 spi_miso_o_net_1 <= spi_miso_o_net_0;
 spi_miso_o       <= spi_miso_o_net_1;
----------------------------------------------------------------------
-- Top level input port assignments
----------------------------------------------------------------------
 MEM_IN_0         <= word_t( RAM_0_RD );
 SPI_IN_0         <= word_t( spi_slave_0_do_o );
----------------------------------------------------------------------
-- Component instances
----------------------------------------------------------------------
-- AND2_0
AND2_0 : AND2
    port map( 
        -- Inputs
        A => spi_slave_0_do_valid_o,
        B => Modulator_0_MEM_WE,
        -- Outputs
        Y => AND2_0_Y 
        );
-- Modulator_0
Modulator_0 : Modulator
    port map( 
        -- Inputs
        CLK        => CLK,
        RESET_N    => RESET_N,
        MEM_IN     => MEM_IN_0,
        SPI_DVLD   => spi_slave_0_do_valid_o,
        SPI_IN     => SPI_IN_0,
        -- Outputs
        MEM_RE     => MOD_EN_net_0,
        MEM_WE     => Modulator_0_MEM_WE,
        MEM_ADDR   => Modulator_0_MEM_ADDR,
        SIGNAL_OUT => SIGNAL_OUT_net_0 
        );
-- RAM_0
RAM_0 : RAM
    port map( 
        -- Inputs
        WD    => SPI_IN_0,
        WEN   => AND2_0_Y,
        REN   => MOD_EN_net_0,
        WADDR => Modulator_0_MEM_ADDR,
        RADDR => Modulator_0_MEM_ADDR,
        RWCLK => CLK,
        RESET => RESET_N,
        -- Outputs
        RD    => RAM_0_RD 
        );
-- spi_slave_0
spi_slave_0 : spi_slave
    port map( 
        -- Inputs
        clk_i      => CLK,
        spi_ssel_i => spi_ssel_i,
        spi_sck_i  => spi_sck_i,
        spi_mosi_i => spi_mosi_i,
        di_i       => di_i_const_net_0,
        wren_i     => GND_net,
        -- Outputs
        spi_miso_o => spi_miso_o_net_0,
        di_req_o   => OPEN,
        wr_ack_o   => OPEN,
        do_valid_o => spi_slave_0_do_valid_o,
        do_o       => spi_slave_0_do_o 
        );

end RTL;
