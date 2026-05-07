
-- VHDL netlist produced by program ldbanno, Version Diamond (64-bit) 3.14.0.75.2

-- ldbanno -n VHDL -o PROJ500_PDU_FPGA_SELFTEST_mapvho.vho -w -neg -gui -msgset C:/Users/charl/Documents/TEMP BACKUPS/17.03.26/PROJ500 (Humanoid)/FPGA/PROJ500_PDU_FPGA/promote.xml PROJ500_PDU_FPGA_SELFTEST_map.ncd 
-- Netlist created on Mon Apr 20 18:52:02 2026
-- Netlist written on Mon Apr 20 18:52:03 2026
-- Design is for device LCMXO2-640HC
-- Design is for package QFN48
-- Design is for performance grade 5

-- entity lut4
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity lut4 is
    port (A: in Std_logic; B: in Std_logic; C: in Std_logic; D: in Std_logic; 
          Z: out Std_logic);

    ATTRIBUTE Vital_Level0 OF lut4 : ENTITY IS TRUE;

  end lut4;

  architecture Structure of lut4 is
  begin
    INST10: ROM16X1A
      generic map (initval => X"6996")
      port map (AD0=>A, AD1=>B, AD2=>C, AD3=>D, DO0=>Z);
  end Structure;

-- entity lut40001
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity lut40001 is
    port (A: in Std_logic; B: in Std_logic; C: in Std_logic; D: in Std_logic; 
          Z: out Std_logic);

    ATTRIBUTE Vital_Level0 OF lut40001 : ENTITY IS TRUE;

  end lut40001;

  architecture Structure of lut40001 is
  begin
    INST10: ROM16X1A
      generic map (initval => X"6666")
      port map (AD0=>A, AD1=>B, AD2=>C, AD3=>D, DO0=>Z);
  end Structure;

-- entity gnd
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity gnd is
    port (PWR0: out Std_logic);

    ATTRIBUTE Vital_Level0 OF gnd : ENTITY IS TRUE;

  end gnd;

  architecture Structure of gnd is
  begin
    INST1: VLO
      port map (Z=>PWR0);
  end Structure;

-- entity SLICE_0
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SLICE_0 is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SLICE_0";

      tipd_D1  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_C1  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_B1  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_A1  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_B0  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_A0  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_D1_F1	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_C1_F1	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_B1_F1	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_A1_F1	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_B0_F0	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_A0_F0	 : VitalDelayType01 := (0 ns, 0 ns));

    port (D1: in Std_logic; C1: in Std_logic; B1: in Std_logic; 
          A1: in Std_logic; B0: in Std_logic; A0: in Std_logic; 
          F0: out Std_logic; F1: out Std_logic);

    ATTRIBUTE Vital_Level0 OF SLICE_0 : ENTITY IS TRUE;

  end SLICE_0;

  architecture Structure of SLICE_0 is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal D1_ipd 	: std_logic := 'X';
    signal C1_ipd 	: std_logic := 'X';
    signal B1_ipd 	: std_logic := 'X';
    signal A1_ipd 	: std_logic := 'X';
    signal B0_ipd 	: std_logic := 'X';
    signal A0_ipd 	: std_logic := 'X';
    signal F0_out 	: std_logic := 'X';
    signal F1_out 	: std_logic := 'X';

    signal GNDI: Std_logic;
    component lut4
      port (A: in Std_logic; B: in Std_logic; C: in Std_logic; D: in Std_logic; 
            Z: out Std_logic);
    end component;
    component lut40001
      port (A: in Std_logic; B: in Std_logic; C: in Std_logic; D: in Std_logic; 
            Z: out Std_logic);
    end component;
    component gnd
      port (PWR0: out Std_logic);
    end component;
  begin
    SPARE_OUT_1: lut4
      port map (A=>A1_ipd, B=>B1_ipd, C=>C1_ipd, D=>D1_ipd, Z=>F1_out);
    SPARE_OUT_1_6: lut40001
      port map (A=>A0_ipd, B=>B0_ipd, C=>GNDI, D=>GNDI, Z=>F0_out);
    DRIVEGND: gnd
      port map (PWR0=>GNDI);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(D1_ipd, D1, tipd_D1);
      VitalWireDelay(C1_ipd, C1, tipd_C1);
      VitalWireDelay(B1_ipd, B1, tipd_B1);
      VitalWireDelay(A1_ipd, A1, tipd_A1);
      VitalWireDelay(B0_ipd, B0, tipd_B0);
      VitalWireDelay(A0_ipd, A0, tipd_A0);
    END BLOCK;

    VitalBehavior : PROCESS (D1_ipd, C1_ipd, B1_ipd, A1_ipd, B0_ipd, A0_ipd, 
      F0_out, F1_out)
    VARIABLE F0_zd         	: std_logic := 'X';
    VARIABLE F0_GlitchData 	: VitalGlitchDataType;
    VARIABLE F1_zd         	: std_logic := 'X';
    VARIABLE F1_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    F0_zd 	:= F0_out;
    F1_zd 	:= F1_out;

    VitalPathDelay01 (
      OutSignal => F0, OutSignalName => "F0", OutTemp => F0_zd,
      Paths      => (0 => (InputChangeTime => B0_ipd'last_event,
                           PathDelay => tpd_B0_F0,
                           PathCondition => TRUE),
                     1 => (InputChangeTime => A0_ipd'last_event,
                           PathDelay => tpd_A0_F0,
                           PathCondition => TRUE)),
      GlitchData => F0_GlitchData,
      Mode       => ondetect, XOn => XOn, MsgOn => MsgOn);
    VitalPathDelay01 (
      OutSignal => F1, OutSignalName => "F1", OutTemp => F1_zd,
      Paths      => (0 => (InputChangeTime => D1_ipd'last_event,
                           PathDelay => tpd_D1_F1,
                           PathCondition => TRUE),
                     1 => (InputChangeTime => C1_ipd'last_event,
                           PathDelay => tpd_C1_F1,
                           PathCondition => TRUE),
                     2 => (InputChangeTime => B1_ipd'last_event,
                           PathDelay => tpd_B1_F1,
                           PathCondition => TRUE),
                     3 => (InputChangeTime => A1_ipd'last_event,
                           PathDelay => tpd_A1_F1,
                           PathCondition => TRUE)),
      GlitchData => F1_GlitchData,
      Mode       => ondetect, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity SLICE_1
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SLICE_1 is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SLICE_1";

      tipd_D1  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_C1  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_B1  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_A1  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_D0  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_C0  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_B0  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_A0  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_D1_F1	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_C1_F1	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_B1_F1	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_A1_F1	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_D0_F0	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_C0_F0	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_B0_F0	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_A0_F0	 : VitalDelayType01 := (0 ns, 0 ns));

    port (D1: in Std_logic; C1: in Std_logic; B1: in Std_logic; 
          A1: in Std_logic; D0: in Std_logic; C0: in Std_logic; 
          B0: in Std_logic; A0: in Std_logic; F0: out Std_logic; 
          F1: out Std_logic);

    ATTRIBUTE Vital_Level0 OF SLICE_1 : ENTITY IS TRUE;

  end SLICE_1;

  architecture Structure of SLICE_1 is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal D1_ipd 	: std_logic := 'X';
    signal C1_ipd 	: std_logic := 'X';
    signal B1_ipd 	: std_logic := 'X';
    signal A1_ipd 	: std_logic := 'X';
    signal D0_ipd 	: std_logic := 'X';
    signal C0_ipd 	: std_logic := 'X';
    signal B0_ipd 	: std_logic := 'X';
    signal A0_ipd 	: std_logic := 'X';
    signal F0_out 	: std_logic := 'X';
    signal F1_out 	: std_logic := 'X';

    component lut4
      port (A: in Std_logic; B: in Std_logic; C: in Std_logic; D: in Std_logic; 
            Z: out Std_logic);
    end component;
  begin
    SPARE_OUT_1_8: lut4
      port map (A=>A1_ipd, B=>B1_ipd, C=>C1_ipd, D=>D1_ipd, Z=>F1_out);
    SPARE_OUT_1_9: lut4
      port map (A=>A0_ipd, B=>B0_ipd, C=>C0_ipd, D=>D0_ipd, Z=>F0_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(D1_ipd, D1, tipd_D1);
      VitalWireDelay(C1_ipd, C1, tipd_C1);
      VitalWireDelay(B1_ipd, B1, tipd_B1);
      VitalWireDelay(A1_ipd, A1, tipd_A1);
      VitalWireDelay(D0_ipd, D0, tipd_D0);
      VitalWireDelay(C0_ipd, C0, tipd_C0);
      VitalWireDelay(B0_ipd, B0, tipd_B0);
      VitalWireDelay(A0_ipd, A0, tipd_A0);
    END BLOCK;

    VitalBehavior : PROCESS (D1_ipd, C1_ipd, B1_ipd, A1_ipd, D0_ipd, C0_ipd, 
      B0_ipd, A0_ipd, F0_out, F1_out)
    VARIABLE F0_zd         	: std_logic := 'X';
    VARIABLE F0_GlitchData 	: VitalGlitchDataType;
    VARIABLE F1_zd         	: std_logic := 'X';
    VARIABLE F1_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    F0_zd 	:= F0_out;
    F1_zd 	:= F1_out;

    VitalPathDelay01 (
      OutSignal => F0, OutSignalName => "F0", OutTemp => F0_zd,
      Paths      => (0 => (InputChangeTime => D0_ipd'last_event,
                           PathDelay => tpd_D0_F0,
                           PathCondition => TRUE),
                     1 => (InputChangeTime => C0_ipd'last_event,
                           PathDelay => tpd_C0_F0,
                           PathCondition => TRUE),
                     2 => (InputChangeTime => B0_ipd'last_event,
                           PathDelay => tpd_B0_F0,
                           PathCondition => TRUE),
                     3 => (InputChangeTime => A0_ipd'last_event,
                           PathDelay => tpd_A0_F0,
                           PathCondition => TRUE)),
      GlitchData => F0_GlitchData,
      Mode       => ondetect, XOn => XOn, MsgOn => MsgOn);
    VitalPathDelay01 (
      OutSignal => F1, OutSignalName => "F1", OutTemp => F1_zd,
      Paths      => (0 => (InputChangeTime => D1_ipd'last_event,
                           PathDelay => tpd_D1_F1,
                           PathCondition => TRUE),
                     1 => (InputChangeTime => C1_ipd'last_event,
                           PathDelay => tpd_C1_F1,
                           PathCondition => TRUE),
                     2 => (InputChangeTime => B1_ipd'last_event,
                           PathDelay => tpd_B1_F1,
                           PathCondition => TRUE),
                     3 => (InputChangeTime => A1_ipd'last_event,
                           PathDelay => tpd_A1_F1,
                           PathCondition => TRUE)),
      GlitchData => F1_GlitchData,
      Mode       => ondetect, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity lut40002
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity lut40002 is
    port (A: in Std_logic; B: in Std_logic; C: in Std_logic; D: in Std_logic; 
          Z: out Std_logic);

    ATTRIBUTE Vital_Level0 OF lut40002 : ENTITY IS TRUE;

  end lut40002;

  architecture Structure of lut40002 is
  begin
    INST10: ROM16X1A
      generic map (initval => X"FFFF")
      port map (AD0=>A, AD1=>B, AD2=>C, AD3=>D, DO0=>Z);
  end Structure;

-- entity SLICE_2
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SLICE_2 is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SLICE_2";

      tipd_D0  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_C0  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_B0  	: VitalDelayType01 := (0 ns, 0 ns);
      tipd_A0  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_D0_F0	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_C0_F0	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_B0_F0	 : VitalDelayType01 := (0 ns, 0 ns);
      tpd_A0_F0	 : VitalDelayType01 := (0 ns, 0 ns));

    port (D0: in Std_logic; C0: in Std_logic; B0: in Std_logic; 
          A0: in Std_logic; F0: out Std_logic; F1: out Std_logic);

    ATTRIBUTE Vital_Level0 OF SLICE_2 : ENTITY IS TRUE;

  end SLICE_2;

  architecture Structure of SLICE_2 is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal D0_ipd 	: std_logic := 'X';
    signal C0_ipd 	: std_logic := 'X';
    signal B0_ipd 	: std_logic := 'X';
    signal A0_ipd 	: std_logic := 'X';
    signal F0_out 	: std_logic := 'X';
    signal F1_out 	: std_logic := 'X';

    signal GNDI: Std_logic;
    component lut4
      port (A: in Std_logic; B: in Std_logic; C: in Std_logic; D: in Std_logic; 
            Z: out Std_logic);
    end component;
    component gnd
      port (PWR0: out Std_logic);
    end component;
    component lut40002
      port (A: in Std_logic; B: in Std_logic; C: in Std_logic; D: in Std_logic; 
            Z: out Std_logic);
    end component;
  begin
    VCC: lut40002
      port map (A=>GNDI, B=>GNDI, C=>GNDI, D=>GNDI, Z=>F1_out);
    DRIVEGND: gnd
      port map (PWR0=>GNDI);
    SPARE_OUT_1_7: lut4
      port map (A=>A0_ipd, B=>B0_ipd, C=>C0_ipd, D=>D0_ipd, Z=>F0_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(D0_ipd, D0, tipd_D0);
      VitalWireDelay(C0_ipd, C0, tipd_C0);
      VitalWireDelay(B0_ipd, B0, tipd_B0);
      VitalWireDelay(A0_ipd, A0, tipd_A0);
    END BLOCK;

    VitalBehavior : PROCESS (D0_ipd, C0_ipd, B0_ipd, A0_ipd, F0_out, F1_out)
    VARIABLE F0_zd         	: std_logic := 'X';
    VARIABLE F0_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    F0_zd 	:= F0_out;
    F1 	<= F1_out;

    VitalPathDelay01 (
      OutSignal => F0, OutSignalName => "F0", OutTemp => F0_zd,
      Paths      => (0 => (InputChangeTime => D0_ipd'last_event,
                           PathDelay => tpd_D0_F0,
                           PathCondition => TRUE),
                     1 => (InputChangeTime => C0_ipd'last_event,
                           PathDelay => tpd_C0_F0,
                           PathCondition => TRUE),
                     2 => (InputChangeTime => B0_ipd'last_event,
                           PathDelay => tpd_B0_F0,
                           PathCondition => TRUE),
                     3 => (InputChangeTime => A0_ipd'last_event,
                           PathDelay => tpd_A0_F0,
                           PathCondition => TRUE)),
      GlitchData => F0_GlitchData,
      Mode       => ondetect, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity xo2iobuf
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity xo2iobuf is
    port (I: in Std_logic; PAD: out Std_logic);

    ATTRIBUTE Vital_Level0 OF xo2iobuf : ENTITY IS TRUE;

  end xo2iobuf;

  architecture Structure of xo2iobuf is
  begin
    INST5: OB
      port map (I=>I, O=>PAD);
  end Structure;

-- entity FAULT_LATCHB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity FAULT_LATCHB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "FAULT_LATCHB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_FAULTLATCH	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; FAULTLATCH: out Std_logic);

    ATTRIBUTE Vital_Level0 OF FAULT_LATCHB : ENTITY IS TRUE;

  end FAULT_LATCHB;

  architecture Structure of FAULT_LATCHB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal FAULTLATCH_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    FAULT_LATCH_pad: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>FAULTLATCH_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, FAULTLATCH_out)
    VARIABLE FAULTLATCH_zd         	: std_logic := 'X';
    VARIABLE FAULTLATCH_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    FAULTLATCH_zd 	:= FAULTLATCH_out;

    VitalPathDelay01 (

        OutSignal => FAULTLATCH, OutSignalName => "FAULTLATCH", OutTemp => FAULTLATCH_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_FAULTLATCH,
                           PathCondition => TRUE)),
      GlitchData => FAULTLATCH_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity xo2iobuf0003
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity xo2iobuf0003 is
    port (Z: out Std_logic; PAD: in Std_logic);

    ATTRIBUTE Vital_Level0 OF xo2iobuf0003 : ENTITY IS TRUE;

  end xo2iobuf0003;

  architecture Structure of xo2iobuf0003 is
  begin
    INST1: IB
      port map (I=>PAD, O=>Z);
  end Structure;

-- entity SW_COMPUTEB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SW_COMPUTEB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SW_COMPUTEB";

      tipd_SWCOMPUTE  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_SWCOMPUTE_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_SWCOMPUTE 	: VitalDelayType := 0 ns;
      tpw_SWCOMPUTE_posedge	: VitalDelayType := 0 ns;
      tpw_SWCOMPUTE_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; SWCOMPUTE: in Std_logic);

    ATTRIBUTE Vital_Level0 OF SW_COMPUTEB : ENTITY IS TRUE;

  end SW_COMPUTEB;

  architecture Structure of SW_COMPUTEB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal SWCOMPUTE_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    SW_COMPUTE_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>SWCOMPUTE_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(SWCOMPUTE_ipd, SWCOMPUTE, tipd_SWCOMPUTE);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, SWCOMPUTE_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_SWCOMPUTE_SWCOMPUTE          	: x01 := '0';
    VARIABLE periodcheckinfo_SWCOMPUTE	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => SWCOMPUTE_ipd,
        TestSignalName => "SWCOMPUTE",
        Period => tperiod_SWCOMPUTE,
        PulseWidthHigh => tpw_SWCOMPUTE_posedge,
        PulseWidthLow => tpw_SWCOMPUTE_negedge,
        PeriodData => periodcheckinfo_SWCOMPUTE,
        Violation => tviol_SWCOMPUTE_SWCOMPUTE,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => SWCOMPUTE_ipd'last_event,
                           PathDelay => tpd_SWCOMPUTE_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity SPARE_OUT_1B
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SPARE_OUT_1B is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SPARE_OUT_1B";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_SPAREOUT1	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; SPAREOUT1: out Std_logic);

    ATTRIBUTE Vital_Level0 OF SPARE_OUT_1B : ENTITY IS TRUE;

  end SPARE_OUT_1B;

  architecture Structure of SPARE_OUT_1B is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal SPAREOUT1_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    SPARE_OUT_1_pad: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>SPAREOUT1_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, SPAREOUT1_out)
    VARIABLE SPAREOUT1_zd         	: std_logic := 'X';
    VARIABLE SPAREOUT1_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    SPAREOUT1_zd 	:= SPAREOUT1_out;

    VitalPathDelay01 (
      OutSignal => SPAREOUT1, OutSignalName => "SPAREOUT1", OutTemp => SPAREOUT1_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_SPAREOUT1,
                           PathCondition => TRUE)),
      GlitchData => SPAREOUT1_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity SPARE_OUT_0B
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SPARE_OUT_0B is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SPARE_OUT_0B";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_SPAREOUT0	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; SPAREOUT0: out Std_logic);

    ATTRIBUTE Vital_Level0 OF SPARE_OUT_0B : ENTITY IS TRUE;

  end SPARE_OUT_0B;

  architecture Structure of SPARE_OUT_0B is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal SPAREOUT0_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    SPARE_OUT_0_pad: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>SPAREOUT0_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, SPAREOUT0_out)
    VARIABLE SPAREOUT0_zd         	: std_logic := 'X';
    VARIABLE SPAREOUT0_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    SPAREOUT0_zd 	:= SPAREOUT0_out;

    VitalPathDelay01 (
      OutSignal => SPAREOUT0, OutSignalName => "SPAREOUT0", OutTemp => SPAREOUT0_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_SPAREOUT0,
                           PathCondition => TRUE)),
      GlitchData => SPAREOUT0_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity DUMP_ENB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity DUMP_ENB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "DUMP_ENB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_DUMPEN	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; DUMPEN: out Std_logic);

    ATTRIBUTE Vital_Level0 OF DUMP_ENB : ENTITY IS TRUE;

  end DUMP_ENB;

  architecture Structure of DUMP_ENB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal DUMPEN_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    DUMP_EN_pad: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>DUMPEN_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, DUMPEN_out)
    VARIABLE DUMPEN_zd         	: std_logic := 'X';
    VARIABLE DUMPEN_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    DUMPEN_zd 	:= DUMPEN_out;

    VitalPathDelay01 (
      OutSignal => DUMPEN, OutSignalName => "DUMPEN", OutTemp => DUMPEN_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_DUMPEN,
                           PathCondition => TRUE)),
      GlitchData => DUMPEN_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity PRECHARGE_REQB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity PRECHARGE_REQB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "PRECHARGE_REQB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_PRECHARGEREQ	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; PRECHARGEREQ: out Std_logic);

    ATTRIBUTE Vital_Level0 OF PRECHARGE_REQB : ENTITY IS TRUE;

  end PRECHARGE_REQB;

  architecture Structure of PRECHARGE_REQB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal PRECHARGEREQ_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    PRECHARGE_REQ_pad: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>PRECHARGEREQ_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, PRECHARGEREQ_out)
    VARIABLE PRECHARGEREQ_zd         	: std_logic := 'X';
    VARIABLE PRECHARGEREQ_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    PRECHARGEREQ_zd 	:= PRECHARGEREQ_out;

    VitalPathDelay01 (

        OutSignal => PRECHARGEREQ, OutSignalName => "PRECHARGEREQ", OutTemp => PRECHARGEREQ_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_PRECHARGEREQ,
                           PathCondition => TRUE)),
      GlitchData => PRECHARGEREQ_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity ARM_PERMITB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity ARM_PERMITB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "ARM_PERMITB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_ARMPERMIT	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; ARMPERMIT: out Std_logic);

    ATTRIBUTE Vital_Level0 OF ARM_PERMITB : ENTITY IS TRUE;

  end ARM_PERMITB;

  architecture Structure of ARM_PERMITB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal ARMPERMIT_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    ARM_PERMIT_pad: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>ARMPERMIT_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, ARMPERMIT_out)
    VARIABLE ARMPERMIT_zd         	: std_logic := 'X';
    VARIABLE ARMPERMIT_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    ARMPERMIT_zd 	:= ARMPERMIT_out;

    VitalPathDelay01 (
      OutSignal => ARMPERMIT, OutSignalName => "ARMPERMIT", OutTemp => ARMPERMIT_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_ARMPERMIT,
                           PathCondition => TRUE)),
      GlitchData => ARMPERMIT_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity OVUV_OKB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity OVUV_OKB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "OVUV_OKB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_OVUVOK	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; OVUVOK: out Std_logic);

    ATTRIBUTE Vital_Level0 OF OVUV_OKB : ENTITY IS TRUE;

  end OVUV_OKB;

  architecture Structure of OVUV_OKB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal OVUVOK_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    OVUV_OK_pad: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>OVUVOK_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, OVUVOK_out)
    VARIABLE OVUVOK_zd         	: std_logic := 'X';
    VARIABLE OVUVOK_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    OVUVOK_zd 	:= OVUVOK_out;

    VitalPathDelay01 (
      OutSignal => OVUVOK, OutSignalName => "OVUVOK", OutTemp => OVUVOK_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_OVUVOK,
                           PathCondition => TRUE)),
      GlitchData => OVUVOK_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity STATE_CODE_1_B
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity STATE_CODE_1_B is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "STATE_CODE_1_B";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_STATECODE1	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; STATECODE1: out Std_logic);

    ATTRIBUTE Vital_Level0 OF STATE_CODE_1_B : ENTITY IS TRUE;

  end STATE_CODE_1_B;

  architecture Structure of STATE_CODE_1_B is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal STATECODE1_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    STATE_CODE_pad_1: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>STATECODE1_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, STATECODE1_out)
    VARIABLE STATECODE1_zd         	: std_logic := 'X';
    VARIABLE STATECODE1_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    STATECODE1_zd 	:= STATECODE1_out;

    VitalPathDelay01 (

        OutSignal => STATECODE1, OutSignalName => "STATECODE1", OutTemp => STATECODE1_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_STATECODE1,
                           PathCondition => TRUE)),
      GlitchData => STATECODE1_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity STATE_CODE_0_B
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity STATE_CODE_0_B is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "STATE_CODE_0_B";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_STATECODE0	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; STATECODE0: out Std_logic);

    ATTRIBUTE Vital_Level0 OF STATE_CODE_0_B : ENTITY IS TRUE;

  end STATE_CODE_0_B;

  architecture Structure of STATE_CODE_0_B is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal STATECODE0_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    STATE_CODE_pad_0: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>STATECODE0_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, STATECODE0_out)
    VARIABLE STATECODE0_zd         	: std_logic := 'X';
    VARIABLE STATECODE0_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    STATECODE0_zd 	:= STATECODE0_out;

    VitalPathDelay01 (

        OutSignal => STATECODE0, OutSignalName => "STATECODE0", OutTemp => STATECODE0_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_STATECODE0,
                           PathCondition => TRUE)),
      GlitchData => STATECODE0_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity FAULT_CODE_3_B
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity FAULT_CODE_3_B is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "FAULT_CODE_3_B";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_FAULTCODE3	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; FAULTCODE3: out Std_logic);

    ATTRIBUTE Vital_Level0 OF FAULT_CODE_3_B : ENTITY IS TRUE;

  end FAULT_CODE_3_B;

  architecture Structure of FAULT_CODE_3_B is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal FAULTCODE3_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    FAULT_CODE_pad_3: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>FAULTCODE3_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, FAULTCODE3_out)
    VARIABLE FAULTCODE3_zd         	: std_logic := 'X';
    VARIABLE FAULTCODE3_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    FAULTCODE3_zd 	:= FAULTCODE3_out;

    VitalPathDelay01 (

        OutSignal => FAULTCODE3, OutSignalName => "FAULTCODE3", OutTemp => FAULTCODE3_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_FAULTCODE3,
                           PathCondition => TRUE)),
      GlitchData => FAULTCODE3_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity FAULT_CODE_2_B
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity FAULT_CODE_2_B is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "FAULT_CODE_2_B";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_FAULTCODE2	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; FAULTCODE2: out Std_logic);

    ATTRIBUTE Vital_Level0 OF FAULT_CODE_2_B : ENTITY IS TRUE;

  end FAULT_CODE_2_B;

  architecture Structure of FAULT_CODE_2_B is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal FAULTCODE2_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    FAULT_CODE_pad_2: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>FAULTCODE2_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, FAULTCODE2_out)
    VARIABLE FAULTCODE2_zd         	: std_logic := 'X';
    VARIABLE FAULTCODE2_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    FAULTCODE2_zd 	:= FAULTCODE2_out;

    VitalPathDelay01 (

        OutSignal => FAULTCODE2, OutSignalName => "FAULTCODE2", OutTemp => FAULTCODE2_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_FAULTCODE2,
                           PathCondition => TRUE)),
      GlitchData => FAULTCODE2_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity FAULT_CODE_1_B
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity FAULT_CODE_1_B is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "FAULT_CODE_1_B";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_FAULTCODE1	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; FAULTCODE1: out Std_logic);

    ATTRIBUTE Vital_Level0 OF FAULT_CODE_1_B : ENTITY IS TRUE;

  end FAULT_CODE_1_B;

  architecture Structure of FAULT_CODE_1_B is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal FAULTCODE1_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    FAULT_CODE_pad_1: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>FAULTCODE1_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, FAULTCODE1_out)
    VARIABLE FAULTCODE1_zd         	: std_logic := 'X';
    VARIABLE FAULTCODE1_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    FAULTCODE1_zd 	:= FAULTCODE1_out;

    VitalPathDelay01 (

        OutSignal => FAULTCODE1, OutSignalName => "FAULTCODE1", OutTemp => FAULTCODE1_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_FAULTCODE1,
                           PathCondition => TRUE)),
      GlitchData => FAULTCODE1_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity FAULT_CODE_0_B
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity FAULT_CODE_0_B is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "FAULT_CODE_0_B";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_FAULTCODE0	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; FAULTCODE0: out Std_logic);

    ATTRIBUTE Vital_Level0 OF FAULT_CODE_0_B : ENTITY IS TRUE;

  end FAULT_CODE_0_B;

  architecture Structure of FAULT_CODE_0_B is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal FAULTCODE0_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    FAULT_CODE_pad_0: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>FAULTCODE0_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, FAULTCODE0_out)
    VARIABLE FAULTCODE0_zd         	: std_logic := 'X';
    VARIABLE FAULTCODE0_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    FAULTCODE0_zd 	:= FAULTCODE0_out;

    VitalPathDelay01 (

        OutSignal => FAULTCODE0, OutSignalName => "FAULTCODE0", OutTemp => FAULTCODE0_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_FAULTCODE0,
                           PathCondition => TRUE)),
      GlitchData => FAULTCODE0_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity xo2iobuf0004
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity xo2iobuf0004 is
    port (I: in Std_logic; PAD: out Std_logic);

    ATTRIBUTE Vital_Level0 OF xo2iobuf0004 : ENTITY IS TRUE;

  end xo2iobuf0004;

  architecture Structure of xo2iobuf0004 is
  begin
    INST5: OB
      port map (I=>I, O=>PAD);
  end Structure;

-- entity K_ENB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity K_ENB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "K_ENB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_KEN	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; KEN: out Std_logic);

    ATTRIBUTE Vital_Level0 OF K_ENB : ENTITY IS TRUE;

  end K_ENB;

  architecture Structure of K_ENB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal KEN_out 	: std_logic := 'X';

    component xo2iobuf0004
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    K_EN_pad: xo2iobuf0004
      port map (I=>PADDO_ipd, PAD=>KEN_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, KEN_out)
    VARIABLE KEN_zd         	: std_logic := 'X';
    VARIABLE KEN_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    KEN_zd 	:= KEN_out;

    VitalPathDelay01 (
      OutSignal => KEN, OutSignalName => "KEN", OutTemp => KEN_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_KEN,
                           PathCondition => TRUE)),
      GlitchData => KEN_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity K_SELB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity K_SELB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "K_SELB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_KSEL	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; KSEL: out Std_logic);

    ATTRIBUTE Vital_Level0 OF K_SELB : ENTITY IS TRUE;

  end K_SELB;

  architecture Structure of K_SELB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal KSEL_out 	: std_logic := 'X';

    component xo2iobuf0004
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    K_SEL_pad: xo2iobuf0004
      port map (I=>PADDO_ipd, PAD=>KSEL_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, KSEL_out)
    VARIABLE KSEL_zd         	: std_logic := 'X';
    VARIABLE KSEL_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    KSEL_zd 	:= KSEL_out;

    VitalPathDelay01 (
      OutSignal => KSEL, OutSignalName => "KSEL", OutTemp => KSEL_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_KSEL,
                           PathCondition => TRUE)),
      GlitchData => KSEL_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity xo2iobuf0005
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity xo2iobuf0005 is
    port (I: in Std_logic; PAD: out Std_logic);

    ATTRIBUTE Vital_Level0 OF xo2iobuf0005 : ENTITY IS TRUE;

  end xo2iobuf0005;

  architecture Structure of xo2iobuf0005 is
  begin
    INST5: OB
      port map (I=>I, O=>PAD);
  end Structure;

-- entity COMPUTE_ENB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity COMPUTE_ENB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "COMPUTE_ENB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_COMPUTEEN	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; COMPUTEEN: out Std_logic);

    ATTRIBUTE Vital_Level0 OF COMPUTE_ENB : ENTITY IS TRUE;

  end COMPUTE_ENB;

  architecture Structure of COMPUTE_ENB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal COMPUTEEN_out 	: std_logic := 'X';

    component xo2iobuf0005
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    COMPUTE_EN_pad: xo2iobuf0005
      port map (I=>PADDO_ipd, PAD=>COMPUTEEN_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, COMPUTEEN_out)
    VARIABLE COMPUTEEN_zd         	: std_logic := 'X';
    VARIABLE COMPUTEEN_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    COMPUTEEN_zd 	:= COMPUTEEN_out;

    VitalPathDelay01 (
      OutSignal => COMPUTEEN, OutSignalName => "COMPUTEEN", OutTemp => COMPUTEEN_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_COMPUTEEN,
                           PathCondition => TRUE)),
      GlitchData => COMPUTEEN_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity MOTOR_ENB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity MOTOR_ENB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "MOTOR_ENB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_MOTOREN	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; MOTOREN: out Std_logic);

    ATTRIBUTE Vital_Level0 OF MOTOR_ENB : ENTITY IS TRUE;

  end MOTOR_ENB;

  architecture Structure of MOTOR_ENB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal MOTOREN_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    MOTOR_EN_pad: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>MOTOREN_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, MOTOREN_out)
    VARIABLE MOTOREN_zd         	: std_logic := 'X';
    VARIABLE MOTOREN_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    MOTOREN_zd 	:= MOTOREN_out;

    VitalPathDelay01 (
      OutSignal => MOTOREN, OutSignalName => "MOTOREN", OutTemp => MOTOREN_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_MOTOREN,
                           PathCondition => TRUE)),
      GlitchData => MOTOREN_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity PRECHARGE_LATCHB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity PRECHARGE_LATCHB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "PRECHARGE_LATCHB";

      tipd_PADDO  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PADDO_PRECHARGELATCH	 : VitalDelayType01 := (0 ns, 0 ns));

    port (PADDO: in Std_logic; PRECHARGELATCH: out Std_logic);

    ATTRIBUTE Vital_Level0 OF PRECHARGE_LATCHB : ENTITY IS TRUE;

  end PRECHARGE_LATCHB;

  architecture Structure of PRECHARGE_LATCHB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDO_ipd 	: std_logic := 'X';
    signal PRECHARGELATCH_out 	: std_logic := 'X';

    component xo2iobuf
      port (I: in Std_logic; PAD: out Std_logic);
    end component;
  begin
    PRECHARGE_LATCH_pad: xo2iobuf
      port map (I=>PADDO_ipd, PAD=>PRECHARGELATCH_out);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PADDO_ipd, PADDO, tipd_PADDO);
    END BLOCK;

    VitalBehavior : PROCESS (PADDO_ipd, PRECHARGELATCH_out)
    VARIABLE PRECHARGELATCH_zd         	: std_logic := 'X';
    VARIABLE PRECHARGELATCH_GlitchData 	: VitalGlitchDataType;


    BEGIN

    IF (TimingChecksOn) THEN

    END IF;

    PRECHARGELATCH_zd 	:= PRECHARGELATCH_out;

    VitalPathDelay01 (

        OutSignal => PRECHARGELATCH, OutSignalName => "PRECHARGELATCH", OutTemp => PRECHARGELATCH_zd,
      Paths      => (0 => (InputChangeTime => PADDO_ipd'last_event,
                           PathDelay => tpd_PADDO_PRECHARGELATCH,
                           PathCondition => TRUE)),
      GlitchData => PRECHARGELATCH_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity SW_REMOTE_DISARMB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SW_REMOTE_DISARMB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SW_REMOTE_DISARMB";

      tipd_SWREMOTEDISARM  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_SWREMOTEDISARM_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_SWREMOTEDISARM 	: VitalDelayType := 0 ns;
      tpw_SWREMOTEDISARM_posedge	: VitalDelayType := 0 ns;
      tpw_SWREMOTEDISARM_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; SWREMOTEDISARM: in Std_logic);

    ATTRIBUTE Vital_Level0 OF SW_REMOTE_DISARMB : ENTITY IS TRUE;

  end SW_REMOTE_DISARMB;

  architecture Structure of SW_REMOTE_DISARMB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal SWREMOTEDISARM_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    SW_REMOTE_DISARM_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>SWREMOTEDISARM_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(SWREMOTEDISARM_ipd, SWREMOTEDISARM, tipd_SWREMOTEDISARM);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, SWREMOTEDISARM_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_SWREMOTEDISARM_SWREMOTEDISARM          	: x01 := '0';
    VARIABLE periodcheckinfo_SWREMOTEDISARM	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => SWREMOTEDISARM_ipd,
        TestSignalName => "SWREMOTEDISARM",
        Period => tperiod_SWREMOTEDISARM,
        PulseWidthHigh => tpw_SWREMOTEDISARM_posedge,
        PulseWidthLow => tpw_SWREMOTEDISARM_negedge,
        PeriodData => periodcheckinfo_SWREMOTEDISARM,
        Violation => tviol_SWREMOTEDISARM_SWREMOTEDISARM,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => SWREMOTEDISARM_ipd'last_event,
                           PathDelay => tpd_SWREMOTEDISARM_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity FB_CLOSEDB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity FB_CLOSEDB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "FB_CLOSEDB";

      tipd_FBCLOSED  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_FBCLOSED_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_FBCLOSED 	: VitalDelayType := 0 ns;
      tpw_FBCLOSED_posedge	: VitalDelayType := 0 ns;
      tpw_FBCLOSED_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; FBCLOSED: in Std_logic);

    ATTRIBUTE Vital_Level0 OF FB_CLOSEDB : ENTITY IS TRUE;

  end FB_CLOSEDB;

  architecture Structure of FB_CLOSEDB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal FBCLOSED_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    FB_CLOSED_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>FBCLOSED_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(FBCLOSED_ipd, FBCLOSED, tipd_FBCLOSED);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, FBCLOSED_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_FBCLOSED_FBCLOSED          	: x01 := '0';
    VARIABLE periodcheckinfo_FBCLOSED	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => FBCLOSED_ipd,
        TestSignalName => "FBCLOSED",
        Period => tperiod_FBCLOSED,
        PulseWidthHigh => tpw_FBCLOSED_posedge,
        PulseWidthLow => tpw_FBCLOSED_negedge,
        PeriodData => periodcheckinfo_FBCLOSED,
        Violation => tviol_FBCLOSED_FBCLOSED,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => FBCLOSED_ipd'last_event,
                           PathDelay => tpd_FBCLOSED_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity PRECHARGE_OKB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity PRECHARGE_OKB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "PRECHARGE_OKB";

      tipd_PRECHARGEOK  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_PRECHARGEOK_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_PRECHARGEOK 	: VitalDelayType := 0 ns;
      tpw_PRECHARGEOK_posedge	: VitalDelayType := 0 ns;
      tpw_PRECHARGEOK_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; PRECHARGEOK: in Std_logic);

    ATTRIBUTE Vital_Level0 OF PRECHARGE_OKB : ENTITY IS TRUE;

  end PRECHARGE_OKB;

  architecture Structure of PRECHARGE_OKB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal PRECHARGEOK_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    PRECHARGE_OK_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>PRECHARGEOK_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(PRECHARGEOK_ipd, PRECHARGEOK, tipd_PRECHARGEOK);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, PRECHARGEOK_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_PRECHARGEOK_PRECHARGEOK          	: x01 := '0';
    VARIABLE periodcheckinfo_PRECHARGEOK	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => PRECHARGEOK_ipd,
        TestSignalName => "PRECHARGEOK",
        Period => tperiod_PRECHARGEOK,
        PulseWidthHigh => tpw_PRECHARGEOK_posedge,
        PulseWidthLow => tpw_PRECHARGEOK_negedge,
        PeriodData => periodcheckinfo_PRECHARGEOK,
        Violation => tviol_PRECHARGEOK_PRECHARGEOK,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => PRECHARGEOK_ipd'last_event,
                           PathDelay => tpd_PRECHARGEOK_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity MCU_CMD_FAULTB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity MCU_CMD_FAULTB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "MCU_CMD_FAULTB";

      tipd_MCUCMDFAULT  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_MCUCMDFAULT_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_MCUCMDFAULT 	: VitalDelayType := 0 ns;
      tpw_MCUCMDFAULT_posedge	: VitalDelayType := 0 ns;
      tpw_MCUCMDFAULT_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; MCUCMDFAULT: in Std_logic);

    ATTRIBUTE Vital_Level0 OF MCU_CMD_FAULTB : ENTITY IS TRUE;

  end MCU_CMD_FAULTB;

  architecture Structure of MCU_CMD_FAULTB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal MCUCMDFAULT_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    MCU_CMD_FAULT_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>MCUCMDFAULT_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(MCUCMDFAULT_ipd, MCUCMDFAULT, tipd_MCUCMDFAULT);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, MCUCMDFAULT_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_MCUCMDFAULT_MCUCMDFAULT          	: x01 := '0';
    VARIABLE periodcheckinfo_MCUCMDFAULT	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => MCUCMDFAULT_ipd,
        TestSignalName => "MCUCMDFAULT",
        Period => tperiod_MCUCMDFAULT,
        PulseWidthHigh => tpw_MCUCMDFAULT_posedge,
        PulseWidthLow => tpw_MCUCMDFAULT_negedge,
        PeriodData => periodcheckinfo_MCUCMDFAULT,
        Violation => tviol_MCUCMDFAULT_MCUCMDFAULT,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => MCUCMDFAULT_ipd'last_event,
                           PathDelay => tpd_MCUCMDFAULT_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity MCU_CMD_ARMB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity MCU_CMD_ARMB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "MCU_CMD_ARMB";

      tipd_MCUCMDARM  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_MCUCMDARM_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_MCUCMDARM 	: VitalDelayType := 0 ns;
      tpw_MCUCMDARM_posedge	: VitalDelayType := 0 ns;
      tpw_MCUCMDARM_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; MCUCMDARM: in Std_logic);

    ATTRIBUTE Vital_Level0 OF MCU_CMD_ARMB : ENTITY IS TRUE;

  end MCU_CMD_ARMB;

  architecture Structure of MCU_CMD_ARMB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal MCUCMDARM_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    MCU_CMD_ARM_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>MCUCMDARM_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(MCUCMDARM_ipd, MCUCMDARM, tipd_MCUCMDARM);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, MCUCMDARM_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_MCUCMDARM_MCUCMDARM          	: x01 := '0';
    VARIABLE periodcheckinfo_MCUCMDARM	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => MCUCMDARM_ipd,
        TestSignalName => "MCUCMDARM",
        Period => tperiod_MCUCMDARM,
        PulseWidthHigh => tpw_MCUCMDARM_posedge,
        PulseWidthLow => tpw_MCUCMDARM_negedge,
        PeriodData => periodcheckinfo_MCUCMDARM,
        Violation => tviol_MCUCMDARM_MCUCMDARM,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => MCUCMDARM_ipd'last_event,
                           PathDelay => tpd_MCUCMDARM_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity MCU_ALIVEB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity MCU_ALIVEB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "MCU_ALIVEB";

      tipd_MCUALIVE  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_MCUALIVE_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_MCUALIVE 	: VitalDelayType := 0 ns;
      tpw_MCUALIVE_posedge	: VitalDelayType := 0 ns;
      tpw_MCUALIVE_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; MCUALIVE: in Std_logic);

    ATTRIBUTE Vital_Level0 OF MCU_ALIVEB : ENTITY IS TRUE;

  end MCU_ALIVEB;

  architecture Structure of MCU_ALIVEB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal MCUALIVE_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    MCU_ALIVE_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>MCUALIVE_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(MCUALIVE_ipd, MCUALIVE, tipd_MCUALIVE);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, MCUALIVE_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_MCUALIVE_MCUALIVE          	: x01 := '0';
    VARIABLE periodcheckinfo_MCUALIVE	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => MCUALIVE_ipd,
        TestSignalName => "MCUALIVE",
        Period => tperiod_MCUALIVE,
        PulseWidthHigh => tpw_MCUALIVE_posedge,
        PulseWidthLow => tpw_MCUALIVE_negedge,
        PeriodData => periodcheckinfo_MCUALIVE,
        Violation => tviol_MCUALIVE_MCUALIVE,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => MCUALIVE_ipd'last_event,
                           PathDelay => tpd_MCUALIVE_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity VBUS_OVB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity VBUS_OVB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "VBUS_OVB";

      tipd_VBUSOV  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_VBUSOV_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_VBUSOV 	: VitalDelayType := 0 ns;
      tpw_VBUSOV_posedge	: VitalDelayType := 0 ns;
      tpw_VBUSOV_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; VBUSOV: in Std_logic);

    ATTRIBUTE Vital_Level0 OF VBUS_OVB : ENTITY IS TRUE;

  end VBUS_OVB;

  architecture Structure of VBUS_OVB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal VBUSOV_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    VBUS_OV_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>VBUSOV_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(VBUSOV_ipd, VBUSOV, tipd_VBUSOV);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, VBUSOV_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_VBUSOV_VBUSOV          	: x01 := '0';
    VARIABLE periodcheckinfo_VBUSOV	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => VBUSOV_ipd,
        TestSignalName => "VBUSOV",
        Period => tperiod_VBUSOV,
        PulseWidthHigh => tpw_VBUSOV_posedge,
        PulseWidthLow => tpw_VBUSOV_negedge,
        PeriodData => periodcheckinfo_VBUSOV,
        Violation => tviol_VBUSOV_VBUSOV,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => VBUSOV_ipd'last_event,
                           PathDelay => tpd_VBUSOV_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity UVB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity UVB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "UVB";

      tipd_UVS  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_UVS_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_UVS 	: VitalDelayType := 0 ns;
      tpw_UVS_posedge	: VitalDelayType := 0 ns;
      tpw_UVS_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; UVS: in Std_logic);

    ATTRIBUTE Vital_Level0 OF UVB : ENTITY IS TRUE;

  end UVB;

  architecture Structure of UVB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal UVS_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    UV_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>UVS_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(UVS_ipd, UVS, tipd_UVS);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, UVS_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_UVS_UVS          	: x01 := '0';
    VARIABLE periodcheckinfo_UVS	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => UVS_ipd,
        TestSignalName => "UVS",
        Period => tperiod_UVS,
        PulseWidthHigh => tpw_UVS_posedge,
        PulseWidthLow => tpw_UVS_negedge,
        PeriodData => periodcheckinfo_UVS,
        Violation => tviol_UVS_UVS,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => UVS_ipd'last_event,
                           PathDelay => tpd_UVS_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity OVB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity OVB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "OVB";

      tipd_OVS  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_OVS_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_OVS 	: VitalDelayType := 0 ns;
      tpw_OVS_posedge	: VitalDelayType := 0 ns;
      tpw_OVS_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; OVS: in Std_logic);

    ATTRIBUTE Vital_Level0 OF OVB : ENTITY IS TRUE;

  end OVB;

  architecture Structure of OVB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal OVS_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    OV_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>OVS_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(OVS_ipd, OVS, tipd_OVS);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, OVS_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_OVS_OVS          	: x01 := '0';
    VARIABLE periodcheckinfo_OVS	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => OVS_ipd,
        TestSignalName => "OVS",
        Period => tperiod_OVS,
        PulseWidthHigh => tpw_OVS_posedge,
        PulseWidthLow => tpw_OVS_negedge,
        PeriodData => periodcheckinfo_OVS,
        Violation => tviol_OVS_OVS,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => OVS_ipd'last_event,
                           PathDelay => tpd_OVS_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity ESTOP_OKB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity ESTOP_OKB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "ESTOP_OKB";

      tipd_ESTOPOK  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_ESTOPOK_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_ESTOPOK 	: VitalDelayType := 0 ns;
      tpw_ESTOPOK_posedge	: VitalDelayType := 0 ns;
      tpw_ESTOPOK_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; ESTOPOK: in Std_logic);

    ATTRIBUTE Vital_Level0 OF ESTOP_OKB : ENTITY IS TRUE;

  end ESTOP_OKB;

  architecture Structure of ESTOP_OKB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal ESTOPOK_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    ESTOP_OK_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>ESTOPOK_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(ESTOPOK_ipd, ESTOPOK, tipd_ESTOPOK);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, ESTOPOK_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_ESTOPOK_ESTOPOK          	: x01 := '0';
    VARIABLE periodcheckinfo_ESTOPOK	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => ESTOPOK_ipd,
        TestSignalName => "ESTOPOK",
        Period => tperiod_ESTOPOK,
        PulseWidthHigh => tpw_ESTOPOK_posedge,
        PulseWidthLow => tpw_ESTOPOK_negedge,
        PeriodData => periodcheckinfo_ESTOPOK,
        Violation => tviol_ESTOPOK_ESTOPOK,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => ESTOPOK_ipd'last_event,
                           PathDelay => tpd_ESTOPOK_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity SW_REMOTE_ARMB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SW_REMOTE_ARMB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SW_REMOTE_ARMB";

      tipd_SWREMOTEARM  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_SWREMOTEARM_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_SWREMOTEARM 	: VitalDelayType := 0 ns;
      tpw_SWREMOTEARM_posedge	: VitalDelayType := 0 ns;
      tpw_SWREMOTEARM_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; SWREMOTEARM: in Std_logic);

    ATTRIBUTE Vital_Level0 OF SW_REMOTE_ARMB : ENTITY IS TRUE;

  end SW_REMOTE_ARMB;

  architecture Structure of SW_REMOTE_ARMB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal SWREMOTEARM_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    SW_REMOTE_ARM_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>SWREMOTEARM_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(SWREMOTEARM_ipd, SWREMOTEARM, tipd_SWREMOTEARM);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, SWREMOTEARM_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_SWREMOTEARM_SWREMOTEARM          	: x01 := '0';
    VARIABLE periodcheckinfo_SWREMOTEARM	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => SWREMOTEARM_ipd,
        TestSignalName => "SWREMOTEARM",
        Period => tperiod_SWREMOTEARM,
        PulseWidthHigh => tpw_SWREMOTEARM_posedge,
        PulseWidthLow => tpw_SWREMOTEARM_negedge,
        PeriodData => periodcheckinfo_SWREMOTEARM,
        Violation => tviol_SWREMOTEARM_SWREMOTEARM,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => SWREMOTEARM_ipd'last_event,
                           PathDelay => tpd_SWREMOTEARM_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity SW_RST_FAULTB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SW_RST_FAULTB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SW_RST_FAULTB";

      tipd_SWRSTFAULT  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_SWRSTFAULT_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_SWRSTFAULT 	: VitalDelayType := 0 ns;
      tpw_SWRSTFAULT_posedge	: VitalDelayType := 0 ns;
      tpw_SWRSTFAULT_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; SWRSTFAULT: in Std_logic);

    ATTRIBUTE Vital_Level0 OF SW_RST_FAULTB : ENTITY IS TRUE;

  end SW_RST_FAULTB;

  architecture Structure of SW_RST_FAULTB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal SWRSTFAULT_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    SW_RST_FAULT_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>SWRSTFAULT_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(SWRSTFAULT_ipd, SWRSTFAULT, tipd_SWRSTFAULT);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, SWRSTFAULT_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_SWRSTFAULT_SWRSTFAULT          	: x01 := '0';
    VARIABLE periodcheckinfo_SWRSTFAULT	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => SWRSTFAULT_ipd,
        TestSignalName => "SWRSTFAULT",
        Period => tperiod_SWRSTFAULT,
        PulseWidthHigh => tpw_SWRSTFAULT_posedge,
        PulseWidthLow => tpw_SWRSTFAULT_negedge,
        PeriodData => periodcheckinfo_SWRSTFAULT,
        Violation => tviol_SWRSTFAULT_SWRSTFAULT,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => SWRSTFAULT_ipd'last_event,
                           PathDelay => tpd_SWRSTFAULT_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity SW_ARMB
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity SW_ARMB is
    -- miscellaneous vital GENERICs
    GENERIC (
      TimingChecksOn	: boolean := TRUE;
      XOn           	: boolean := FALSE;
      MsgOn         	: boolean := TRUE;
      InstancePath  	: string := "SW_ARMB";

      tipd_SWARM  	: VitalDelayType01 := (0 ns, 0 ns);
      tpd_SWARM_PADDI	 : VitalDelayType01 := (0 ns, 0 ns);
      tperiod_SWARM 	: VitalDelayType := 0 ns;
      tpw_SWARM_posedge	: VitalDelayType := 0 ns;
      tpw_SWARM_negedge	: VitalDelayType := 0 ns);

    port (PADDI: out Std_logic; SWARM: in Std_logic);

    ATTRIBUTE Vital_Level0 OF SW_ARMB : ENTITY IS TRUE;

  end SW_ARMB;

  architecture Structure of SW_ARMB is
    ATTRIBUTE Vital_Level0 OF Structure : ARCHITECTURE IS TRUE;

    signal PADDI_out 	: std_logic := 'X';
    signal SWARM_ipd 	: std_logic := 'X';

    component xo2iobuf0003
      port (Z: out Std_logic; PAD: in Std_logic);
    end component;
  begin
    SW_ARM_pad: xo2iobuf0003
      port map (Z=>PADDI_out, PAD=>SWARM_ipd);

    --  INPUT PATH DELAYs
    WireDelay : BLOCK
    BEGIN
      VitalWireDelay(SWARM_ipd, SWARM, tipd_SWARM);
    END BLOCK;

    VitalBehavior : PROCESS (PADDI_out, SWARM_ipd)
    VARIABLE PADDI_zd         	: std_logic := 'X';
    VARIABLE PADDI_GlitchData 	: VitalGlitchDataType;

    VARIABLE tviol_SWARM_SWARM          	: x01 := '0';
    VARIABLE periodcheckinfo_SWARM	: VitalPeriodDataType;

    BEGIN

    IF (TimingChecksOn) THEN
      VitalPeriodPulseCheck (
        TestSignal => SWARM_ipd,
        TestSignalName => "SWARM",
        Period => tperiod_SWARM,
        PulseWidthHigh => tpw_SWARM_posedge,
        PulseWidthLow => tpw_SWARM_negedge,
        PeriodData => periodcheckinfo_SWARM,
        Violation => tviol_SWARM_SWARM,
        MsgOn => MsgOn, XOn => XOn,
        HeaderMsg => InstancePath,
        CheckEnabled => TRUE,
        MsgSeverity => warning);

    END IF;

    PADDI_zd 	:= PADDI_out;

    VitalPathDelay01 (
      OutSignal => PADDI, OutSignalName => "PADDI", OutTemp => PADDI_zd,
      Paths      => (0 => (InputChangeTime => SWARM_ipd'last_event,
                           PathDelay => tpd_SWARM_PADDI,
                           PathCondition => TRUE)),
      GlitchData => PADDI_GlitchData,
      Mode       => vitaltransport, XOn => XOn, MsgOn => MsgOn);

    END PROCESS;

  end Structure;

-- entity pdu_selftest_mxo2
  library IEEE, vital2000, MACHXO2;
  use IEEE.STD_LOGIC_1164.all;
  use vital2000.vital_timing.all;
  use MACHXO2.COMPONENTS.ALL;

  entity pdu_selftest_mxo2 is
    port (SW_COMPUTE: in Std_logic; SW_ARM: in Std_logic; 
          SW_RST_FAULT: in Std_logic; SW_REMOTE_ARM: in Std_logic; 
          ESTOP_OK: in Std_logic; OV: in Std_logic; UV: in Std_logic; 
          VBUS_OV: in Std_logic; MCU_ALIVE: in Std_logic; 
          MCU_CMD_ARM: in Std_logic; MCU_CMD_FAULT: in Std_logic; 
          PRECHARGE_OK: in Std_logic; FB_CLOSED: in Std_logic; 
          SW_REMOTE_DISARM: in Std_logic; i2c1_scl: in Std_logic; 
          i2c1_sda: in Std_logic; FAULT_LATCH: out Std_logic; 
          PRECHARGE_LATCH: out Std_logic; MOTOR_EN: out Std_logic; 
          COMPUTE_EN: out Std_logic; K_SEL: out Std_logic; K_EN: out Std_logic; 
          FAULT_CODE: out Std_logic_vector (3 downto 0); 
          STATE_CODE: out Std_logic_vector (1 downto 0); 
          OVUV_OK: out Std_logic; ARM_PERMIT: out Std_logic; 
          PRECHARGE_REQ: out Std_logic; DUMP_EN: out Std_logic; 
          SPARE_OUT_0: out Std_logic; SPARE_OUT_1: out Std_logic);



  end pdu_selftest_mxo2;

  architecture Structure of pdu_selftest_mxo2 is
    signal SPARE_OUT_1_9: Std_logic;
    signal SPARE_OUT_1_8: Std_logic;
    signal SPARE_OUT_1_7: Std_logic;
    signal SPARE_OUT_1_6: Std_logic;
    signal VBUS_OV_c: Std_logic;
    signal UV_c: Std_logic;
    signal SPARE_OUT_1_c: Std_logic;
    signal SW_ARM_c: Std_logic;
    signal PRECHARGE_OK_c: Std_logic;
    signal OV_c: Std_logic;
    signal MCU_CMD_FAULT_c: Std_logic;
    signal SW_RST_FAULT_c: Std_logic;
    signal SW_REMOTE_DISARM_c: Std_logic;
    signal SW_REMOTE_ARM_c: Std_logic;
    signal SW_COMPUTE_c: Std_logic;
    signal MCU_CMD_ARM_c: Std_logic;
    signal MCU_ALIVE_c: Std_logic;
    signal FB_CLOSED_c: Std_logic;
    signal ESTOP_OK_c: Std_logic;
    signal VCC: Std_logic;
    signal VCCI: Std_logic;
    component SLICE_0
      port (D1: in Std_logic; C1: in Std_logic; B1: in Std_logic; 
            A1: in Std_logic; B0: in Std_logic; A0: in Std_logic; 
            F0: out Std_logic; F1: out Std_logic);
    end component;
    component SLICE_1
      port (D1: in Std_logic; C1: in Std_logic; B1: in Std_logic; 
            A1: in Std_logic; D0: in Std_logic; C0: in Std_logic; 
            B0: in Std_logic; A0: in Std_logic; F0: out Std_logic; 
            F1: out Std_logic);
    end component;
    component SLICE_2
      port (D0: in Std_logic; C0: in Std_logic; B0: in Std_logic; 
            A0: in Std_logic; F0: out Std_logic; F1: out Std_logic);
    end component;
    component FAULT_LATCHB
      port (PADDO: in Std_logic; FAULTLATCH: out Std_logic);
    end component;
    component SW_COMPUTEB
      port (PADDI: out Std_logic; SWCOMPUTE: in Std_logic);
    end component;
    component SPARE_OUT_1B
      port (PADDO: in Std_logic; SPAREOUT1: out Std_logic);
    end component;
    component SPARE_OUT_0B
      port (PADDO: in Std_logic; SPAREOUT0: out Std_logic);
    end component;
    component DUMP_ENB
      port (PADDO: in Std_logic; DUMPEN: out Std_logic);
    end component;
    component PRECHARGE_REQB
      port (PADDO: in Std_logic; PRECHARGEREQ: out Std_logic);
    end component;
    component ARM_PERMITB
      port (PADDO: in Std_logic; ARMPERMIT: out Std_logic);
    end component;
    component OVUV_OKB
      port (PADDO: in Std_logic; OVUVOK: out Std_logic);
    end component;
    component STATE_CODE_1_B
      port (PADDO: in Std_logic; STATECODE1: out Std_logic);
    end component;
    component STATE_CODE_0_B
      port (PADDO: in Std_logic; STATECODE0: out Std_logic);
    end component;
    component FAULT_CODE_3_B
      port (PADDO: in Std_logic; FAULTCODE3: out Std_logic);
    end component;
    component FAULT_CODE_2_B
      port (PADDO: in Std_logic; FAULTCODE2: out Std_logic);
    end component;
    component FAULT_CODE_1_B
      port (PADDO: in Std_logic; FAULTCODE1: out Std_logic);
    end component;
    component FAULT_CODE_0_B
      port (PADDO: in Std_logic; FAULTCODE0: out Std_logic);
    end component;
    component K_ENB
      port (PADDO: in Std_logic; KEN: out Std_logic);
    end component;
    component K_SELB
      port (PADDO: in Std_logic; KSEL: out Std_logic);
    end component;
    component COMPUTE_ENB
      port (PADDO: in Std_logic; COMPUTEEN: out Std_logic);
    end component;
    component MOTOR_ENB
      port (PADDO: in Std_logic; MOTOREN: out Std_logic);
    end component;
    component PRECHARGE_LATCHB
      port (PADDO: in Std_logic; PRECHARGELATCH: out Std_logic);
    end component;
    component SW_REMOTE_DISARMB
      port (PADDI: out Std_logic; SWREMOTEDISARM: in Std_logic);
    end component;
    component FB_CLOSEDB
      port (PADDI: out Std_logic; FBCLOSED: in Std_logic);
    end component;
    component PRECHARGE_OKB
      port (PADDI: out Std_logic; PRECHARGEOK: in Std_logic);
    end component;
    component MCU_CMD_FAULTB
      port (PADDI: out Std_logic; MCUCMDFAULT: in Std_logic);
    end component;
    component MCU_CMD_ARMB
      port (PADDI: out Std_logic; MCUCMDARM: in Std_logic);
    end component;
    component MCU_ALIVEB
      port (PADDI: out Std_logic; MCUALIVE: in Std_logic);
    end component;
    component VBUS_OVB
      port (PADDI: out Std_logic; VBUSOV: in Std_logic);
    end component;
    component UVB
      port (PADDI: out Std_logic; UVS: in Std_logic);
    end component;
    component OVB
      port (PADDI: out Std_logic; OVS: in Std_logic);
    end component;
    component ESTOP_OKB
      port (PADDI: out Std_logic; ESTOPOK: in Std_logic);
    end component;
    component SW_REMOTE_ARMB
      port (PADDI: out Std_logic; SWREMOTEARM: in Std_logic);
    end component;
    component SW_RST_FAULTB
      port (PADDI: out Std_logic; SWRSTFAULT: in Std_logic);
    end component;
    component SW_ARMB
      port (PADDI: out Std_logic; SWARM: in Std_logic);
    end component;
  begin
    SLICE_0I: SLICE_0
      port map (D1=>SPARE_OUT_1_9, C1=>SPARE_OUT_1_8, B1=>SPARE_OUT_1_7, 
                A1=>SPARE_OUT_1_6, B0=>VBUS_OV_c, A0=>UV_c, F0=>SPARE_OUT_1_6, 
                F1=>SPARE_OUT_1_c);
    SLICE_1I: SLICE_1
      port map (D1=>SW_ARM_c, C1=>PRECHARGE_OK_c, B1=>OV_c, 
                A1=>MCU_CMD_FAULT_c, D0=>SW_RST_FAULT_c, 
                C0=>SW_REMOTE_DISARM_c, B0=>SW_REMOTE_ARM_c, A0=>SW_COMPUTE_c, 
                F0=>SPARE_OUT_1_9, F1=>SPARE_OUT_1_8);
    SLICE_2I: SLICE_2
      port map (D0=>MCU_CMD_ARM_c, C0=>MCU_ALIVE_c, B0=>FB_CLOSED_c, 
                A0=>ESTOP_OK_c, F0=>SPARE_OUT_1_7, F1=>VCC);
    FAULT_LATCHI: FAULT_LATCHB
      port map (PADDO=>VCC, FAULTLATCH=>FAULT_LATCH);
    SW_COMPUTEI: SW_COMPUTEB
      port map (PADDI=>SW_COMPUTE_c, SWCOMPUTE=>SW_COMPUTE);
    SPARE_OUT_1I: SPARE_OUT_1B
      port map (PADDO=>SPARE_OUT_1_c, SPAREOUT1=>SPARE_OUT_1);
    SPARE_OUT_0I: SPARE_OUT_0B
      port map (PADDO=>VCC, SPAREOUT0=>SPARE_OUT_0);
    DUMP_ENI: DUMP_ENB
      port map (PADDO=>VCC, DUMPEN=>DUMP_EN);
    PRECHARGE_REQI: PRECHARGE_REQB
      port map (PADDO=>VCC, PRECHARGEREQ=>PRECHARGE_REQ);
    ARM_PERMITI: ARM_PERMITB
      port map (PADDO=>VCC, ARMPERMIT=>ARM_PERMIT);
    OVUV_OKI: OVUV_OKB
      port map (PADDO=>VCC, OVUVOK=>OVUV_OK);
    STATE_CODE_1_I: STATE_CODE_1_B
      port map (PADDO=>VCC, STATECODE1=>STATE_CODE(1));
    STATE_CODE_0_I: STATE_CODE_0_B
      port map (PADDO=>VCC, STATECODE0=>STATE_CODE(0));
    FAULT_CODE_3_I: FAULT_CODE_3_B
      port map (PADDO=>VCC, FAULTCODE3=>FAULT_CODE(3));
    FAULT_CODE_2_I: FAULT_CODE_2_B
      port map (PADDO=>VCC, FAULTCODE2=>FAULT_CODE(2));
    FAULT_CODE_1_I: FAULT_CODE_1_B
      port map (PADDO=>VCC, FAULTCODE1=>FAULT_CODE(1));
    FAULT_CODE_0_I: FAULT_CODE_0_B
      port map (PADDO=>VCC, FAULTCODE0=>FAULT_CODE(0));
    K_ENI: K_ENB
      port map (PADDO=>VCC, KEN=>K_EN);
    K_SELI: K_SELB
      port map (PADDO=>VCC, KSEL=>K_SEL);
    COMPUTE_ENI: COMPUTE_ENB
      port map (PADDO=>VCC, COMPUTEEN=>COMPUTE_EN);
    MOTOR_ENI: MOTOR_ENB
      port map (PADDO=>VCC, MOTOREN=>MOTOR_EN);
    PRECHARGE_LATCHI: PRECHARGE_LATCHB
      port map (PADDO=>VCC, PRECHARGELATCH=>PRECHARGE_LATCH);
    SW_REMOTE_DISARMI: SW_REMOTE_DISARMB
      port map (PADDI=>SW_REMOTE_DISARM_c, SWREMOTEDISARM=>SW_REMOTE_DISARM);
    FB_CLOSEDI: FB_CLOSEDB
      port map (PADDI=>FB_CLOSED_c, FBCLOSED=>FB_CLOSED);
    PRECHARGE_OKI: PRECHARGE_OKB
      port map (PADDI=>PRECHARGE_OK_c, PRECHARGEOK=>PRECHARGE_OK);
    MCU_CMD_FAULTI: MCU_CMD_FAULTB
      port map (PADDI=>MCU_CMD_FAULT_c, MCUCMDFAULT=>MCU_CMD_FAULT);
    MCU_CMD_ARMI: MCU_CMD_ARMB
      port map (PADDI=>MCU_CMD_ARM_c, MCUCMDARM=>MCU_CMD_ARM);
    MCU_ALIVEI: MCU_ALIVEB
      port map (PADDI=>MCU_ALIVE_c, MCUALIVE=>MCU_ALIVE);
    VBUS_OVI: VBUS_OVB
      port map (PADDI=>VBUS_OV_c, VBUSOV=>VBUS_OV);
    UVI: UVB
      port map (PADDI=>UV_c, UVS=>UV);
    OVI: OVB
      port map (PADDI=>OV_c, OVS=>OV);
    ESTOP_OKI: ESTOP_OKB
      port map (PADDI=>ESTOP_OK_c, ESTOPOK=>ESTOP_OK);
    SW_REMOTE_ARMI: SW_REMOTE_ARMB
      port map (PADDI=>SW_REMOTE_ARM_c, SWREMOTEARM=>SW_REMOTE_ARM);
    SW_RST_FAULTI: SW_RST_FAULTB
      port map (PADDI=>SW_RST_FAULT_c, SWRSTFAULT=>SW_RST_FAULT);
    SW_ARMI: SW_ARMB
      port map (PADDI=>SW_ARM_c, SWARM=>SW_ARM);
    VHI_INST: VHI
      port map (Z=>VCCI);
    PUR_INST: PUR
      port map (PUR=>VCCI);
    GSR_INST: GSR
      port map (GSR=>VCCI);
  end Structure;



  library IEEE, vital2000, MACHXO2;
  configuration Structure_CON of pdu_selftest_mxo2 is
    for Structure
    end for;
  end Structure_CON;


