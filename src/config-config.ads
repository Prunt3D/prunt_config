with Motion_Planner;
with Physical_Types; use Physical_Types;
with TOML;
with Ada.Strings.Bounded;

generic
   type Stepper_Name is (<>);
   type Heater_Name is (<>);
   type Thermistor_Name is (<>);
   type Fan_Name is (<>);
   type Input_Switch_Name is (<>);
   Config_Path : String;
package Config.Config is

   package Path_Strings is new Ada.Strings.Bounded.Generic_Bounded_Length (Max => 200);

   type Attached_Steppers is array (Stepper_Name) of Boolean;

   IO_Error                 : exception;
   Config_File_Format_Error : exception;

   type Prunt_Parameters is record
      Enabled : Boolean := False;
   end record;

   type Stepper_Parameters is record
      Enabled              : Boolean := False;
      Invert_Direction     : Boolean := False;
      Enabled_On_High      : Boolean := False;
      Fault_On_High        : Boolean := False;
      Mm_Per_Step          : Length  := Length'Last / 2.0;
      Direction_Setup_Time : Time    := 0.0 * s;
      Step_Time            : Time    := 0.0 * s;
   end record;

   type Kinematics_Kind is (Cartesian_Kind, Core_XY_Kind);

   type Kinematics_Parameters (Kind : Kinematics_Kind := Cartesian_Kind) is record
      Lower_Pos_Limit                 : Position := [E_Axis => Length'First / 2.0, others => 0.0 * mm];
      Upper_Pos_Limit                 : Position := [E_Axis => Length'Last / 2.0, others => 0.0 * mm];
      Max_Limits                      : Motion_Planner.Kinematic_Limits :=
        (Acceleration_Max => 0.0 * mm / s**2,
         Jerk_Max         => 0.0 * mm / s**3,
         Snap_Max         => 0.0 * mm / s**4,
         Crackle_Max      => 0.0 * mm / s**5,
         Chord_Error_Max  => 0.0 * mm);
      Max_Feedrate                    : Velocity                        := 0.0 * mm / s;
      Max_Axial_Velocities            : Axial_Velocities                := [others => 0.0 * mm / s];
      Ignore_E_Feedrate_In_XYZE_Moves : Boolean                         := True;
      Planning_Scaler                 : Position_Scale                  := [others => 1.0];
      Minimum_Cruise_Ratio            : Cruise_Ratio                    := Cruise_Ratio'First;
      Z_Steppers                      : Attached_Steppers               := [others => False];
      E_Steppers                      : Attached_Steppers               := [others => False];
      case Kind is
         when Cartesian_Kind =>
            X_Steppers : Attached_Steppers := [others => False];
            Y_Steppers : Attached_Steppers := [others => False];
         when Core_XY_Kind =>
            A_Steppers : Attached_Steppers := [others => False];
            B_Steppers : Attached_Steppers := [others => False];
      end case;
   end record;

   type Input_Switch_Parameters is record
      Enabled     : Boolean := False;
      Hit_On_High : Boolean := False;
   end record;

   type Homing_Kind is (Double_Tap_Kind, Set_To_Value_Kind);

   type Homing_Parameters (Kind : Homing_Kind := Double_Tap_Kind) is record
      case Kind is
         when Double_Tap_Kind =>
            Switch               : Input_Switch_Name := Input_Switch_Name'First;
            First_Move_Distance  : Length            := 0.0 * mm;
            Second_Move_Distance : Length            := 0.0 * mm;
            Switch_Position      : Length            := 0.0 * mm;
         when Set_To_Value_Kind =>
            Value : Length := 0.0 * mm;
      end case;
   end record;

   type Extruder_Parameters is record
      Nozzle_Diameter                : Length := 0.4 * mm;
      Filament_Diameter              : Length := 1.75 * mm;
      Starting_Pressure_Advance_Time : Time   := 0.0 * s;
   end record;

   type Thermistor_Parameters is record
      Enabled             : Boolean     := False;
      Minimum_Temperature : Temperature := 0.0 * celcius;
      Maximum_Temperature : Temperature := 0.0 * celcius;
   end record;

   type Heater_Kind is (Disabled_Kind, PID_Kind, Bang_Bang_Kind);

   type Heater_Parameters (Kind : Heater_Kind := Disabled_Kind) is record
      Thermistor : Thermistor_Name := Thermistor_Name'First;
      case Kind is
         when Disabled_Kind =>
            null;
         when PID_Kind =>
            Proportional_Scale : Dimensionless := 0.0;
            Integral_Scale     : Dimensionless := 0.0;
            Derivative_Scale   : Dimensionless := 0.0;
         when Bang_Bang_Kind =>
            Max_Delta : Temperature := 2.0 * celcius;
      end case;
   end record;

   type Bed_Mesh_Kind is (No_Mesh_Kind, Beacon_Kind);

   type Bed_Mesh_Parameters (Kind : Bed_Mesh_Kind := No_Mesh_Kind) is record
      case Kind is
         when No_Mesh_Kind =>
            null;
         when Beacon_Kind =>
            Serial_Port_Path     : Path_Strings.Bounded_String := Path_Strings.To_Bounded_String ("");
            X_Offset             : Length                      := 0.0 * mm;
            Y_Offset             : Length                      := 0.0 * mm;
            Calibration_Floor    : Length                      := 0.2 * mm;
            Calibration_Ceiling  : Length                      := 5.0 * mm;
            Calibration_Feedrate : Velocity                    := 1.0 * mm / s;
      end case;
   end record;

   type Fan_Kind is (Disabled_Kind, Dynamic_PWM_Kind, Dynamic_Voltage_Kind, Always_On_Kind);

   type Fan_Parameters (Kind : Fan_Kind := Disabled_Kind) is record
      case Kind is
         when Disabled_Kind =>
            null;
         when Dynamic_PWM_Kind =>
            Disable_Below_PWM : PWM_Scale := 0.5;
            Max_PWM           : PWM_Scale := 1.0;
            Fixed_Voltage     : Voltage   := 6.0 * volt;
         when Dynamic_Voltage_Kind =>
            Disable_Below_Voltage : Voltage := 5.0 * volt;
            Max_Voltage           : Voltage := 6.0 * volt;
         when Always_On_Kind =>
            Always_On_PWM     : PWM_Scale := 1.0;
            Always_On_Voltage : Voltage   := 6.0 * volt;
      end case;
   end record;

   type G_Code_Assignment_Parameters is record
      Bed_Heater    : Heater_Name := Heater_Name'First;
      --  Chamber_Heater : Heater_Name := Heater_Name'First;
      Hotend_Heater : Heater_Name := Heater_Name'First;
   end record;

   protected Config_File is
      procedure Read (Data : out Prunt_Parameters);
      procedure Write (Data : Prunt_Parameters; Append_Only : Boolean := False);
      procedure Read (Data : out Stepper_Parameters; Stepper : Stepper_Name);
      procedure Write (Data : Stepper_Parameters; Stepper : Stepper_Name; Append_Only : Boolean := False);
      procedure Read (Data : out Kinematics_Parameters);
      procedure Write (Data : Kinematics_Parameters; Append_Only : Boolean := False);
      procedure Read (Data : out Input_Switch_Parameters; Input_Switch : Input_Switch_Name);
      procedure Write
        (Data : Input_Switch_Parameters; Input_Switch : Input_Switch_Name; Append_Only : Boolean := False);
      procedure Read (Data : out Homing_Parameters; Axis : Axis_Name);
      procedure Write (Data : Homing_Parameters; Axis : Axis_Name; Append_Only : Boolean := False);
      procedure Read (Data : out Extruder_Parameters);
      procedure Write (Data : Extruder_Parameters; Append_Only : Boolean := False);
      procedure Read (Data : out Thermistor_Parameters; Thermistor : Thermistor_Name);
      procedure Write (Data : Thermistor_Parameters; Thermistor : Thermistor_Name; Append_Only : Boolean := False);
      procedure Read (Data : out Heater_Parameters; Heater : Heater_Name);
      procedure Write (Data : Heater_Parameters; Heater : Heater_Name; Append_Only : Boolean := False);
      procedure Read (Data : out Bed_Mesh_Parameters);
      procedure Write (Data : Bed_Mesh_Parameters; Append_Only : Boolean := False);
      procedure Read (Data : out Fan_Parameters; Fan : Fan_Name);
      procedure Write (Data : Fan_Parameters; Fan : Fan_Name; Append_Only : Boolean := False);
      procedure Read (Data : out G_Code_Assignment_Parameters);
      procedure Write (Data : G_Code_Assignment_Parameters; Append_Only : Boolean := False);
   private
      procedure Maybe_Read_File;
      procedure Write_File;
      File_Read : Boolean         := False;
      TOML_Data : TOML.TOML_Value := TOML.No_TOML_Value;
   end Config_File;

end Config.Config;
