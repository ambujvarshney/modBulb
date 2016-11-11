-- Version: v11.7 11.7.0.119

library ieee;
use ieee.std_logic_1164.all;
library igloo;
use igloo.all;

entity RAM is

    port( WD    : in    std_logic_vector(15 downto 0);
          RD    : out   std_logic_vector(15 downto 0);
          WEN   : in    std_logic;
          REN   : in    std_logic;
          WADDR : in    std_logic_vector(8 downto 0);
          RADDR : in    std_logic_vector(8 downto 0);
          RWCLK : in    std_logic;
          RESET : in    std_logic
        );

end RAM;

architecture DEF_ARCH of RAM is 

  component MX2
    port( A : in    std_logic := 'U';
          B : in    std_logic := 'U';
          S : in    std_logic := 'U';
          Y : out   std_logic
        );
  end component;

  component INV
    port( A : in    std_logic := 'U';
          Y : out   std_logic
        );
  end component;

  component BUFF
    port( A : in    std_logic := 'U';
          Y : out   std_logic
        );
  end component;

  component DFN1
    port( D   : in    std_logic := 'U';
          CLK : in    std_logic := 'U';
          Q   : out   std_logic
        );
  end component;

  component OR2
    port( A : in    std_logic := 'U';
          B : in    std_logic := 'U';
          Y : out   std_logic
        );
  end component;

  component RAM512X18
    generic (MEMORYFILE:string := "");

    port( RADDR8 : in    std_logic := 'U';
          RADDR7 : in    std_logic := 'U';
          RADDR6 : in    std_logic := 'U';
          RADDR5 : in    std_logic := 'U';
          RADDR4 : in    std_logic := 'U';
          RADDR3 : in    std_logic := 'U';
          RADDR2 : in    std_logic := 'U';
          RADDR1 : in    std_logic := 'U';
          RADDR0 : in    std_logic := 'U';
          WADDR8 : in    std_logic := 'U';
          WADDR7 : in    std_logic := 'U';
          WADDR6 : in    std_logic := 'U';
          WADDR5 : in    std_logic := 'U';
          WADDR4 : in    std_logic := 'U';
          WADDR3 : in    std_logic := 'U';
          WADDR2 : in    std_logic := 'U';
          WADDR1 : in    std_logic := 'U';
          WADDR0 : in    std_logic := 'U';
          WD17   : in    std_logic := 'U';
          WD16   : in    std_logic := 'U';
          WD15   : in    std_logic := 'U';
          WD14   : in    std_logic := 'U';
          WD13   : in    std_logic := 'U';
          WD12   : in    std_logic := 'U';
          WD11   : in    std_logic := 'U';
          WD10   : in    std_logic := 'U';
          WD9    : in    std_logic := 'U';
          WD8    : in    std_logic := 'U';
          WD7    : in    std_logic := 'U';
          WD6    : in    std_logic := 'U';
          WD5    : in    std_logic := 'U';
          WD4    : in    std_logic := 'U';
          WD3    : in    std_logic := 'U';
          WD2    : in    std_logic := 'U';
          WD1    : in    std_logic := 'U';
          WD0    : in    std_logic := 'U';
          RW0    : in    std_logic := 'U';
          RW1    : in    std_logic := 'U';
          WW0    : in    std_logic := 'U';
          WW1    : in    std_logic := 'U';
          PIPE   : in    std_logic := 'U';
          REN    : in    std_logic := 'U';
          WEN    : in    std_logic := 'U';
          RCLK   : in    std_logic := 'U';
          WCLK   : in    std_logic := 'U';
          RESET  : in    std_logic := 'U';
          RD17   : out   std_logic;
          RD16   : out   std_logic;
          RD15   : out   std_logic;
          RD14   : out   std_logic;
          RD13   : out   std_logic;
          RD12   : out   std_logic;
          RD11   : out   std_logic;
          RD10   : out   std_logic;
          RD9    : out   std_logic;
          RD8    : out   std_logic;
          RD7    : out   std_logic;
          RD6    : out   std_logic;
          RD5    : out   std_logic;
          RD4    : out   std_logic;
          RD3    : out   std_logic;
          RD2    : out   std_logic;
          RD1    : out   std_logic;
          RD0    : out   std_logic
        );
  end component;

  component OR2A
    port( A : in    std_logic := 'U';
          B : in    std_logic := 'U';
          Y : out   std_logic
        );
  end component;

  component DFN1E1C0
    port( D   : in    std_logic := 'U';
          CLK : in    std_logic := 'U';
          CLR : in    std_logic := 'U';
          E   : in    std_logic := 'U';
          Q   : out   std_logic
        );
  end component;

  component GND
    port(Y : out std_logic); 
  end component;

  component VCC
    port(Y : out std_logic); 
  end component;

    signal WEAP, \ADDRA_FF2[0]\, \ADDRB_FF1[0]\, \ADDRB_FF2[0]\, 
        \READB_EN_2[0]\, \ENABLE_ADDRA[0]\, \ENABLE_ADDRA[1]\, 
        \ENABLE_ADDRB[0]\, \ENABLE_ADDRB[1]\, \BLKA_EN[0]\, 
        \BLKB_EN[0]\, \BLKA_EN[1]\, \BLKB_EN[1]\, \READA_EN[0]\, 
        \READB_EN[0]\, \READA_EN[1]\, \READB_EN[1]\, 
        \READB_EN_2[1]\, \QX_TEMPR0[0]\, \QX_TEMPR0[1]\, 
        \QX_TEMPR0[2]\, \QX_TEMPR0[3]\, \QX_TEMPR0[4]\, 
        \QX_TEMPR0[5]\, \QX_TEMPR0[6]\, \QX_TEMPR0[7]\, 
        \QX_TEMPR1[0]\, \QX_TEMPR1[1]\, \QX_TEMPR1[2]\, 
        \QX_TEMPR1[3]\, \QX_TEMPR1[4]\, \QX_TEMPR1[5]\, 
        \QX_TEMPR1[6]\, \QX_TEMPR1[7]\, \QX_TEMPR0[8]\, 
        \QX_TEMPR0[9]\, \QX_TEMPR0[10]\, \QX_TEMPR0[11]\, 
        \QX_TEMPR0[12]\, \QX_TEMPR0[13]\, \QX_TEMPR0[14]\, 
        \QX_TEMPR0[15]\, \QX_TEMPR1[8]\, \QX_TEMPR1[9]\, 
        \QX_TEMPR1[10]\, \QX_TEMPR1[11]\, \QX_TEMPR1[12]\, 
        \QX_TEMPR1[13]\, \QX_TEMPR1[14]\, \QX_TEMPR1[15]\, 
        MX2_4_Y, MX2_0_Y, MX2_2_Y, MX2_14_Y, MX2_5_Y, MX2_7_Y, 
        MX2_11_Y, MX2_1_Y, BUFF_2_Y, BUFF_3_Y, MX2_10_Y, MX2_8_Y, 
        MX2_12_Y, MX2_3_Y, MX2_15_Y, MX2_6_Y, MX2_9_Y, MX2_13_Y, 
        BUFF_0_Y, BUFF_1_Y, \VCC\, \GND\ : std_logic;
    signal GND_power_net1 : std_logic;
    signal VCC_power_net1 : std_logic;

begin 

    \GND\ <= GND_power_net1;
    \VCC\ <= VCC_power_net1;

    \MX2_RD[4]\ : MX2
      port map(A => \QX_TEMPR0[4]\, B => \QX_TEMPR1[4]\, S => 
        BUFF_2_Y, Y => RD(4));
    
    \INV_ENABLE_ADDRB[1]\ : INV
      port map(A => RADDR(8), Y => \ENABLE_ADDRB[1]\);
    
    BUFF_3 : BUFF
      port map(A => \ADDRB_FF2[0]\, Y => BUFF_3_Y);
    
    \MX2_RD[3]\ : MX2
      port map(A => \QX_TEMPR0[3]\, B => \QX_TEMPR1[3]\, S => 
        BUFF_2_Y, Y => RD(3));
    
    \MX2_RD[6]\ : MX2
      port map(A => \QX_TEMPR0[6]\, B => \QX_TEMPR1[6]\, S => 
        BUFF_2_Y, Y => RD(6));
    
    \READEN_BFF1[0]\ : DFN1
      port map(D => \READB_EN[0]\, CLK => RWCLK, Q => 
        \READB_EN_2[0]\);
    
    MX2_9 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_1_Y, Y => 
        MX2_9_Y);
    
    BUFF_1 : BUFF
      port map(A => \ADDRB_FF2[0]\, Y => BUFF_1_Y);
    
    MX2_0 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_3_Y, Y => 
        MX2_0_Y);
    
    \ORB_GATE[1]\ : OR2
      port map(A => \ENABLE_ADDRB[1]\, B => REN, Y => 
        \BLKB_EN[1]\);
    
    \MX2_RD[12]\ : MX2
      port map(A => \QX_TEMPR0[12]\, B => \QX_TEMPR1[12]\, S => 
        BUFF_0_Y, Y => RD(12));
    
    MX2_11 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_3_Y, Y => 
        MX2_11_Y);
    
    \ORA_GATE[1]\ : OR2
      port map(A => \ENABLE_ADDRA[1]\, B => WEAP, Y => 
        \BLKA_EN[1]\);
    
    MX2_6 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_1_Y, Y => 
        MX2_6_Y);
    
    RAM_R0C0 : RAM512X18
      port map(RADDR8 => \GND\, RADDR7 => RADDR(7), RADDR6 => 
        RADDR(6), RADDR5 => RADDR(5), RADDR4 => RADDR(4), RADDR3
         => RADDR(3), RADDR2 => RADDR(2), RADDR1 => RADDR(1), 
        RADDR0 => RADDR(0), WADDR8 => \GND\, WADDR7 => WADDR(7), 
        WADDR6 => WADDR(6), WADDR5 => WADDR(5), WADDR4 => 
        WADDR(4), WADDR3 => WADDR(3), WADDR2 => WADDR(2), WADDR1
         => WADDR(1), WADDR0 => WADDR(0), WD17 => \GND\, WD16 => 
        WD(15), WD15 => WD(14), WD14 => WD(13), WD13 => WD(12), 
        WD12 => WD(11), WD11 => WD(10), WD10 => WD(9), WD9 => 
        WD(8), WD8 => \GND\, WD7 => WD(7), WD6 => WD(6), WD5 => 
        WD(5), WD4 => WD(4), WD3 => WD(3), WD2 => WD(2), WD1 => 
        WD(1), WD0 => WD(0), RW0 => \GND\, RW1 => \VCC\, WW0 => 
        \GND\, WW1 => \VCC\, PIPE => \VCC\, REN => \BLKB_EN[0]\, 
        WEN => \BLKA_EN[0]\, RCLK => RWCLK, WCLK => RWCLK, RESET
         => RESET, RD17 => OPEN, RD16 => \QX_TEMPR0[15]\, RD15
         => \QX_TEMPR0[14]\, RD14 => \QX_TEMPR0[13]\, RD13 => 
        \QX_TEMPR0[12]\, RD12 => \QX_TEMPR0[11]\, RD11 => 
        \QX_TEMPR0[10]\, RD10 => \QX_TEMPR0[9]\, RD9 => 
        \QX_TEMPR0[8]\, RD8 => OPEN, RD7 => \QX_TEMPR0[7]\, RD6
         => \QX_TEMPR0[6]\, RD5 => \QX_TEMPR0[5]\, RD4 => 
        \QX_TEMPR0[4]\, RD3 => \QX_TEMPR0[3]\, RD2 => 
        \QX_TEMPR0[2]\, RD1 => \QX_TEMPR0[1]\, RD0 => 
        \QX_TEMPR0[0]\);
    
    \MX2_RD[5]\ : MX2
      port map(A => \QX_TEMPR0[5]\, B => \QX_TEMPR1[5]\, S => 
        BUFF_2_Y, Y => RD(5));
    
    \MX2_RD[2]\ : MX2
      port map(A => \QX_TEMPR0[2]\, B => \QX_TEMPR1[2]\, S => 
        BUFF_2_Y, Y => RD(2));
    
    \ORB_READ_EN_GATE[1]\ : OR2A
      port map(A => REN, B => \ENABLE_ADDRB[1]\, Y => 
        \READB_EN[1]\);
    
    \ORA_READ_EN_GATE[1]\ : OR2A
      port map(A => WEAP, B => \ENABLE_ADDRA[1]\, Y => 
        \READA_EN[1]\);
    
    \BUFF_ENABLE_ADDRB[0]\ : BUFF
      port map(A => RADDR(8), Y => \ENABLE_ADDRB[0]\);
    
    \ORB_READ_EN_GATE[0]\ : OR2A
      port map(A => REN, B => \ENABLE_ADDRB[0]\, Y => 
        \READB_EN[0]\);
    
    \ORA_READ_EN_GATE[0]\ : OR2A
      port map(A => WEAP, B => \ENABLE_ADDRA[0]\, Y => 
        \READA_EN[0]\);
    
    \INV_ENABLE_ADDRA[1]\ : INV
      port map(A => WADDR(8), Y => \ENABLE_ADDRA[1]\);
    
    MX2_3 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_1_Y, Y => 
        MX2_3_Y);
    
    \BFF2[0]\ : DFN1E1C0
      port map(D => \ADDRB_FF1[0]\, CLK => RWCLK, CLR => RESET, E
         => \READB_EN_2[0]\, Q => \ADDRB_FF2[0]\);
    
    MX2_10 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_1_Y, Y => 
        MX2_10_Y);
    
    RAM_R1C0 : RAM512X18
      port map(RADDR8 => \GND\, RADDR7 => RADDR(7), RADDR6 => 
        RADDR(6), RADDR5 => RADDR(5), RADDR4 => RADDR(4), RADDR3
         => RADDR(3), RADDR2 => RADDR(2), RADDR1 => RADDR(1), 
        RADDR0 => RADDR(0), WADDR8 => \GND\, WADDR7 => WADDR(7), 
        WADDR6 => WADDR(6), WADDR5 => WADDR(5), WADDR4 => 
        WADDR(4), WADDR3 => WADDR(3), WADDR2 => WADDR(2), WADDR1
         => WADDR(1), WADDR0 => WADDR(0), WD17 => \GND\, WD16 => 
        WD(15), WD15 => WD(14), WD14 => WD(13), WD13 => WD(12), 
        WD12 => WD(11), WD11 => WD(10), WD10 => WD(9), WD9 => 
        WD(8), WD8 => \GND\, WD7 => WD(7), WD6 => WD(6), WD5 => 
        WD(5), WD4 => WD(4), WD3 => WD(3), WD2 => WD(2), WD1 => 
        WD(1), WD0 => WD(0), RW0 => \GND\, RW1 => \VCC\, WW0 => 
        \GND\, WW1 => \VCC\, PIPE => \VCC\, REN => \BLKB_EN[1]\, 
        WEN => \BLKA_EN[1]\, RCLK => RWCLK, WCLK => RWCLK, RESET
         => RESET, RD17 => OPEN, RD16 => \QX_TEMPR1[15]\, RD15
         => \QX_TEMPR1[14]\, RD14 => \QX_TEMPR1[13]\, RD13 => 
        \QX_TEMPR1[12]\, RD12 => \QX_TEMPR1[11]\, RD11 => 
        \QX_TEMPR1[10]\, RD10 => \QX_TEMPR1[9]\, RD9 => 
        \QX_TEMPR1[8]\, RD8 => OPEN, RD7 => \QX_TEMPR1[7]\, RD6
         => \QX_TEMPR1[6]\, RD5 => \QX_TEMPR1[5]\, RD4 => 
        \QX_TEMPR1[4]\, RD3 => \QX_TEMPR1[3]\, RD2 => 
        \QX_TEMPR1[2]\, RD1 => \QX_TEMPR1[1]\, RD0 => 
        \QX_TEMPR1[0]\);
    
    \MX2_RD[1]\ : MX2
      port map(A => \QX_TEMPR0[1]\, B => \QX_TEMPR1[1]\, S => 
        BUFF_2_Y, Y => RD(1));
    
    \AFF1[0]\ : DFN1E1C0
      port map(D => WADDR(8), CLK => RWCLK, CLR => RESET, E => 
        \READA_EN[0]\, Q => \ADDRA_FF2[0]\);
    
    \MX2_RD[7]\ : MX2
      port map(A => \QX_TEMPR0[7]\, B => \QX_TEMPR1[7]\, S => 
        BUFF_2_Y, Y => RD(7));
    
    MX2_4 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_3_Y, Y => 
        MX2_4_Y);
    
    \BFF1[0]\ : DFN1
      port map(D => RADDR(8), CLK => RWCLK, Q => \ADDRB_FF1[0]\);
    
    BUFF_2 : BUFF
      port map(A => \ADDRB_FF2[0]\, Y => BUFF_2_Y);
    
    \ORB_GATE[0]\ : OR2
      port map(A => \ENABLE_ADDRB[0]\, B => REN, Y => 
        \BLKB_EN[0]\);
    
    \BUFF_ENABLE_ADDRA[0]\ : BUFF
      port map(A => WADDR(8), Y => \ENABLE_ADDRA[0]\);
    
    BUFF_0 : BUFF
      port map(A => \ADDRB_FF2[0]\, Y => BUFF_0_Y);
    
    MX2_5 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_3_Y, Y => 
        MX2_5_Y);
    
    \MX2_RD[9]\ : MX2
      port map(A => \QX_TEMPR0[9]\, B => \QX_TEMPR1[9]\, S => 
        BUFF_0_Y, Y => RD(9));
    
    \ORA_GATE[0]\ : OR2
      port map(A => \ENABLE_ADDRA[0]\, B => WEAP, Y => 
        \BLKA_EN[0]\);
    
    MX2_15 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_1_Y, Y => 
        MX2_15_Y);
    
    MX2_8 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_1_Y, Y => 
        MX2_8_Y);
    
    \MX2_RD[0]\ : MX2
      port map(A => \QX_TEMPR0[0]\, B => \QX_TEMPR1[0]\, S => 
        BUFF_2_Y, Y => RD(0));
    
    \READEN_BFF1[1]\ : DFN1
      port map(D => \READB_EN[1]\, CLK => RWCLK, Q => 
        \READB_EN_2[1]\);
    
    \MX2_RD[15]\ : MX2
      port map(A => \QX_TEMPR0[15]\, B => \QX_TEMPR1[15]\, S => 
        BUFF_0_Y, Y => RD(15));
    
    \MX2_RD[8]\ : MX2
      port map(A => \QX_TEMPR0[8]\, B => \QX_TEMPR1[8]\, S => 
        BUFF_0_Y, Y => RD(8));
    
    MX2_2 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_3_Y, Y => 
        MX2_2_Y);
    
    \MX2_RD[10]\ : MX2
      port map(A => \QX_TEMPR0[10]\, B => \QX_TEMPR1[10]\, S => 
        BUFF_0_Y, Y => RD(10));
    
    MX2_7 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_3_Y, Y => 
        MX2_7_Y);
    
    MX2_13 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_1_Y, Y => 
        MX2_13_Y);
    
    WEBUBBLEA : INV
      port map(A => WEN, Y => WEAP);
    
    \MX2_RD[14]\ : MX2
      port map(A => \QX_TEMPR0[14]\, B => \QX_TEMPR1[14]\, S => 
        BUFF_0_Y, Y => RD(14));
    
    MX2_1 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_3_Y, Y => 
        MX2_1_Y);
    
    MX2_14 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_3_Y, Y => 
        MX2_14_Y);
    
    \MX2_RD[13]\ : MX2
      port map(A => \QX_TEMPR0[13]\, B => \QX_TEMPR1[13]\, S => 
        BUFF_0_Y, Y => RD(13));
    
    \MX2_RD[11]\ : MX2
      port map(A => \QX_TEMPR0[11]\, B => \QX_TEMPR1[11]\, S => 
        BUFF_0_Y, Y => RD(11));
    
    MX2_12 : MX2
      port map(A => \GND\, B => \GND\, S => BUFF_1_Y, Y => 
        MX2_12_Y);
    
    GND_power_inst1 : GND
      port map( Y => GND_power_net1);

    VCC_power_inst1 : VCC
      port map( Y => VCC_power_net1);


end DEF_ARCH; 

-- _Disclaimer: Please leave the following comments in the file, they are for internal purposes only._


-- _GEN_File_Contents_

-- Version:11.7.0.119
-- ACTGENU_CALL:1
-- BATCH:T
-- FAM:PA3LCLP
-- OUTFORMAT:VHDL
-- LPMTYPE:LPM_RAM
-- LPM_HINT:TWO
-- INSERT_PAD:NO
-- INSERT_IOREG:NO
-- GEN_BHV_VHDL_VAL:F
-- GEN_BHV_VERILOG_VAL:F
-- MGNTIMER:F
-- MGNCMPL:T
-- DESDIR:C:/Users/Abdullah/Projects/FPGA/Workplace/ModBulb_modulation_FPGA/smartgen\RAM
-- GEN_BEHV_MODULE:F
-- SMARTGEN_DIE:UM2X2M1NLP
-- SMARTGEN_PACKAGE:vq100
-- AGENIII_IS_SUBPROJECT_LIBERO:T
-- WWIDTH:16
-- WDEPTH:512
-- RWIDTH:16
-- RDEPTH:512
-- CLKS:1
-- CLOCK_PN:RWCLK
-- RESET_PN:RESET
-- RESET_POLARITY:0
-- INIT_RAM:F
-- DEFAULT_WORD:0x0000
-- CASCADE:1
-- LP_POLARITY:2
-- FF_POLARITY:2
-- WCLK_EDGE:RISE
-- PMODE2:1
-- DATA_IN_PN:WD
-- WADDRESS_PN:WADDR
-- WE_PN:WEN
-- DATA_OUT_PN:RD
-- RADDRESS_PN:RADDR
-- RE_PN:REN
-- WE_POLARITY:1
-- RE_POLARITY:0
-- PTYPE:1

-- _End_Comments_

