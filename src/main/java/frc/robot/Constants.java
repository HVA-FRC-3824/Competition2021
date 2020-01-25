package frc.robot;

public class Constants 
{
  /**
   * PID Constants
   */
  public static final int K_SLOT_IDX                              = 0;
  public static final int K_PID_LOOP_IDX                          = 0;
  public static final int K_TIMEOUT_MS                            = 30;

  /* Chamber */
  public static final double CHAMBER_CHAMBER_START_F              = 0.0;
  public static final double CHAMBER_CHAMBER_START_P              = 0.0;
  public static final double CHAMBER_CHAMBER_START_I              = 0.0;
  public static final double CHAMBER_CHAMBER_START_D              = 0.0;
  public static final int    CHAMBER_CHAMBER_START_CRUISEVELOCITY = 0;
  public static final int    CHAMBER_CHAMBER_START_ACCELERATION   = 0;

  public static final double CHAMBER_CHAMBER_END_F                = 0.0;
  public static final double CHAMBER_CHAMBER_END_P                = 0.0;
  public static final double CHAMBER_CHAMBER_END_I                = 0.0;
  public static final double CHAMBER_CHAMBER_END_D                = 0.0;
  public static final int    CHAMBER_CHAMBER_END_CRUISEVELOCITY   = 0;
  public static final int    CHAMBER_CHAMBER_END_ACCELERATION     = 0;

  /* Chassis */
  public static final double CHASSIS_LEFT_MASTER_F                = 0.0;
  public static final double CHASSIS_LEFT_MASTER_P                = 0.0;
  public static final double CHASSIS_LEFT_MASTER_I                = 0.0;
  public static final double CHASSIS_LEFT_MASTER_D                = 0.0;

  public static final double CHASSIS_LEFT_SLAVE_F                 = 0.0;
  public static final double CHASSIS_LEFT_SLAVE_P                 = 0.0;
  public static final double CHASSIS_LEFT_SLAVE_I                 = 0.0;
  public static final double CHASSIS_LEFT_SLAVE_D                 = 0.0;

  public static final double CHASSIS_RIGHT_MASTER_F               = 0.0;
  public static final double CHASSIS_RIGHT_MASTER_P               = 0.0;
  public static final double CHASSIS_RIGHT_MASTER_I               = 0.0;
  public static final double CHASSIS_RIGHT_MASTER_D               = 0.0;

  public static final double CHASSIS_RIGHT_SLAVE_F                = 0.0;
  public static final double CHASSIS_RIGHT_SLAVE_P                = 0.0;
  public static final double CHASSIS_RIGHT_SLAVE_I                = 0.0;
  public static final double CHASSIS_RIGHT_SLAVE_D                = 0.0;

  /* Climber */
  public static final double CLIMBER_STRING_PULLER_F              = 0.0;
  public static final double CLIMBER_STRING_PULLER_P              = 0.0;
  public static final double CLIMBER_STRING_PULLER_I              = 0.0;
  public static final double CLIMBER_STRING_PULLER_D              = 0.0;
  public static final int CLIMBER_STRING_PULLER_CRUISEVELOCITY    = 0;
  public static final int CLIMBER_STRING_PULLER_ACCELERATION      = 0;

  /* Control Panel */
  public static final double CONTROLPANEL_WHEEL_SPINNER_F         = 0.0;
  public static final double CONTROLPANEL_WHEEL_SPINNER_P         = 0.0;
  public static final double CONTROLPANEL_WHEEL_SPINNER_I         = 0.0;
  public static final double CONTROLPANEL_WHEEL_SPINNER_D         = 0.0;

  /* Launcher */
  public static final double LAUNCHER_TOP_WHEEL_F                 = 0.0;
  public static final double LAUNCHER_TOP_WHEEL_P                 = 0.0;
  public static final double LAUNCHER_TOP_WHEEL_I                 = 0.0;
  public static final double LAUNCHER_TOP_WHEEL_D                 = 0.0;

  public static final double LAUNCHER_BOTTOM_WHEEL_F              = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_P              = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_I              = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_D              = 0.0;

  public static final double LAUNCHER_FEEDER_F                    = 0.0;
  public static final double LAUNCHER_FEEDER_P                    = 0.0;
  public static final double LAUNCHER_FEEDER_I                    = 0.0;
  public static final double LAUNCHER_FEEDER_D                    = 0.0;
  public static final int    LAUNCHER_FEEDER_CRUISEVELOCITY       = 0;
  public static final int    LAUNCHER_FEEDER_ACCELERATION         = 0;

  public static final double LAUNCHER_PIVOT_F                     = 0.0;
  public static final double LAUNCHER_PIVOT_P                     = 0.0;
  public static final double LAUNCHER_PIVOT_I                     = 0.0;
  public static final double LAUNCHER_PIVOT_D                     = 0.0;
  public static final int    LAUNCHER_PIVOT_CRUISEVELOCITY        = 0;
  public static final int    LAUNCHER_PIVOT_ACCELERATION          = 0;

  /* Intake */
  public static final double INTAKE_WHEEL_INTAKE_F                = 0.0;
  public static final double INTAKE_WHEEL_INTAKE_P                = 0.0;
  public static final double INTAKE_WHEEL_INTAKE_I                = 0.0;
  public static final double INTAKE_WHEEL_INTAKE_D                = 0.0;

  /**
   * Robot Map
   * A collection of ports and IDs for various objects representing hardware.
   */

  /* CAN IDs */
  public static final int CHASSIS_LEFT_MASTER_ID                  = 1;
  public static final int CHASSIS_LEFT_SLAVE_ID                   = 2;
  public static final int CHASSIS_RIGHT_MASTER_ID                 = 3;
  public static final int CHASSIS_RIGHT_SLAVE_ID                  = 4;
  public static final int INTAKE_WHEEL_INTAKE_ID                  = 5;
  public static final int CHAMBER_START_ID                        = 6;
  public static final int CHAMBER_END_ID                          = 7;
  public static final int LAUNCHER_WHEEL_TOP_ID                   = 8;
  public static final int LAUNCHER_WHEEL_BOTTOM_ID                = 9;
  public static final int LAUNCHER_FEEDER_ID                      = 10;
  public static final int LAUNCHER_PIVOT_ID                       = 11;
  public static final int CLIMBER_STRING_PULLER_ID                = 12;
  public static final int CONTROLPANEL_WHEEL_SPINNER_ID           = 13;

  /* Pneumatic Ports */
  public static final int CHASSIS_GEARSHIFT_PORT_A                = 0;
  public static final int CHASSIS_GEARSHIFT_PORT_B                = 1;
  public static final int INTAKE_EXTENDER_PORT_A                  = 2;
  public static final int INTAKE_EXTENDER_PORT_B                  = 3;
  public static final int CLIMBER_PTO_PORT_A                      = 4;
  public static final int CLIMBER_PTO_PORT_B                      = 5;

  /* Digital IO Ports */
  public static final int CHAMBER_BALL_POS_1_PORT                 = 0;
  public static final int CHAMBER_BALL_POS_2_PORT                 = 1;
  public static final int CHAMBER_BALL_POS_3_PORT                 = 2;
  public static final int CHAMBER_BALL_POS_4_PORT                 = 3;
  public static final int LAUNCHER_BALL_SWITCH_PORT               = 4;
  public static final int CONTROLPANEL_COLOR_SENSOR_PORT          = 5;

  /* PWM Ports */
  public static final int CLIMBER_RATCHET_LEFT_PORT               = 0;
  public static final int CLIMBER_RATCHET_RIGHT_PORT              = 1;

  /**
   * Driver Joystick Map
   */
  public static final int DRIVER_JOYSTICK_PORT                    = 0;
  public static final int DRIVER_GEAR_SHIFT_BTN_ID                = 2; 

  /**
   * Operator Joystick Map
   */
  public static final int OPERATOR_JOYSTICK_PORT                  = 1;
  public static final int OPERATOR_EXTEND_INTAKE_BTN_ID           = 2;
  public static final int OPERATOR_SET_INTAKE_WHEEL_POWER_BTN_ID  = 3;
  public static final int OPERATOR_SET_INTAKE_WHEEL_RPM_BTN_ID    = 4;
  
}
