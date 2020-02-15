package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class Constants 
{
  /**
   * Robot Map
   * A collection of ports and IDs for various objects representing hardware.
   */

  /* CAN IDs */
  public static final int CHAMBER_ELEVATOR_ID                               = 6;

  public static final int CHASSIS_LEFT_MASTER_ID                            = 1;
  public static final int CHASSIS_LEFT_SLAVE_ID                             = 2;
  public static final int CHASSIS_RIGHT_MASTER_ID                           = 3;
  public static final int CHASSIS_RIGHT_SLAVE_ID                            = 4;

  public static final int CLIMBER_STRING_PULLER_ID                          = 12;
  public static final int CLIMBER_REEL_ID                                   = 11;
  public static final int CLIMBER_ZIPLINE_ID                                = 14;

  public static final int CONTROL_PANEL_SPINNER_ID                          = 13;

  public static final int INTAKE_WHEEL_INTAKE_ID                            = 5;

  public static final int LAUNCHER_TOP_WHEEL_ID                             = 8;
  public static final int LAUNCHER_BOTTOM_WHEEL_ID                          = 9;
  public static final int LAUNCHER_FEEDER_ID                                = 10;
  public static final int LAUNCHER_PIVOT_ID                                 = 11;
 
  /* Pneumatic Ports */
  public static final int CHASSIS_GEARSHIFT_PORT_A                          = 0;
  public static final int CHASSIS_GEARSHIFT_PORT_B                          = 1;

  public static final int INTAKE_EXTENDER_PORT_A                            = 2;
  public static final int INTAKE_EXTENDER_PORT_B                            = 3;

  public static final int CLIMBER_PTO_PORT_A                                = 4;
  public static final int CLIMBER_PTO_PORT_B                                = 5;

  /* Digital IO Ports */
  public static final int CHAMBER_BALL_POS_1_PORT_A                         = 0;
  public static final int CHAMBER_BALL_POS_1_PORT_B                         = 1;
  public static final int CHAMBER_BALL_POS_2_PORT_A                         = 2;
  public static final int CHAMBER_BALL_POS_2_PORT_B                         = 3;
  public static final int CHAMBER_BALL_POS_3_PORT_A                         = 4;
  public static final int CHAMBER_BALL_POS_3_PORT_B                         = 5;
  public static final int CHAMBER_BALL_POS_4_PORT_A                         = 6;
  public static final int CHAMBER_BALL_POS_4_PORT_B                         = 7;
  public static final int CHAMBER_BALL_POS_5_PORT_A                         = 8;
  public static final int CHAMBER_BALL_POS_5_PORT_B                         = 9;
  
  public static final int CONTROLPANEL_COLOR_SENSOR_PORT                    = 10;

  public static final int LAUNCHER_BALL_SWITCH_PORT                         = 11;

  /* PWM Ports */
  public static final int CLIMBER_RATCHET_LEFT_PORT                         = 1;
  public static final int CLIMBER_RATCHET_RIGHT_PORT                        = 2;
  public static final int CLIMBER_LOCK_RATCHET_LEFT_PORT                    = 3;
  public static final int CLIMBER_LOCK_RATCHET_RIGHT_PORT                   = 4;

  /**
   * PID Constants
   */
  public static final int    K_PID_LOOP_IDX                                 = 0;
  public static final int    K_SLOT_IDX                                     = 0;
  public static final int    K_TIMEOUT_MS                                   = 30;

  /* Chassis */
  public static final double K_CHASSIS_TURN_P                               = 0;
  public static final double K_CHASSIS_TURN_I                               = 0;
  public static final double K_CHASSIS_TURN_D                               = 0;

  public static final double K_TURN_TOLERANCE_DEG                           = 0;
  public static final double K_TURN_RATE_TOLERANCE_DEG_PER_SEC              = 0;

  public static final double K_CHASSIS_TURN_VISION_P                        = 0.04;
  public static final double K_CHASSIS_TURN_VISION_MIN                      = 0.03;

  /* Chamber */
  public static final int    CHAMBER_ENTERING_BALLPOS                       = 0;
  public static final int    CHAMBER_EXITING_BALLPOS                        = 1;

  public static final double CHAMBER_BALL_PRESENT_DIST                      = 10.0;

  public static final double CHAMBER_BALL_STEP_DIST                         = 2000;
  
  public static final double CHAMBER_ELEVATOR_F                             = 0.0;
  public static final double CHAMBER_ELEVATOR_P                             = 0.0;
  public static final double CHAMBER_ELEVATOR_I                             = 0.0;
  public static final double CHAMBER_ELEVATOR_D                             = 0.0;
  public static final int    CHAMBER_ELEVATOR_CRUISEVELOCITY                = 0;
  public static final int    CHAMBER_ELEVATOR_ACCELERATION                  = 0;

  /* Climber */
  public static final double CLIMBER_STRING_PULLER_F                        = 0.0;
  public static final double CLIMBER_STRING_PULLER_P                        = 0.0;
  public static final double CLIMBER_STRING_PULLER_I                        = 0.0;
  public static final double CLIMBER_STRING_PULLER_D                        = 0.0;
  public static final int    CLIMBER_STRING_PULLER_CRUISEVELOCITY           = 0;
  public static final int    CLIMBER_STRING_PULLER_ACCELERATION             = 0;

  public static final double CLIMBER_REEL_F                                 = 0.0;
  public static final double CLIMBER_REEL_P                                 = 0.0;
  public static final double CLIMBER_REEL_I                                 = 0.0;
  public static final double CLIMBER_REEL_D                                 = 0.0;
  public static final int    CLIMBER_REEL_CRUISEVELOCITY                    = 0;
  public static final int    CLIMBER_REEL_ACCELERATION                      = 0;
  
  public static final double CLIMBER_ZIPLINE_F                              = 0;
  public static final double CLIMBER_ZIPLINE_P                              = 0;
  public static final double CLIMBER_ZIPLINE_I                              = 0;
  public static final double CLIMBER_ZIPLINE_D                              = 0;
  public static final int    CLIMBER_ZIPLINE_CRUISEVELOCITY                 = 0;
  public static final int    CLIMBER_ZIPLINE_ACCELERATION                   = 0;

  /* Control Panel */   
  public static final double CONTROL_PANEL_SPINNER_F                        = 0.0;
  public static final double CONTROL_PANEL_SPINNER_P                        = 0.0;
  public static final double CONTROL_PANEL_SPINNER_I                        = 0.0;
  public static final double CONTROL_PANEL_SPINNER_D                        = 0.0;

  /* Intake */
  public static final double INTAKE_WHEEL_INTAKE_F                          = 0.0;
  public static final double INTAKE_WHEEL_INTAKE_P                          = 0.0;
  public static final double INTAKE_WHEEL_INTAKE_I                          = 0.0;
  public static final double INTAKE_WHEEL_INTAKE_D                          = 0.0;

  /* Launcher */
  public static final double LAUNCHER_TOP_WHEEL_F                           = 0.0;
  public static final double LAUNCHER_TOP_WHEEL_P                           = 0.0;
  public static final double LAUNCHER_TOP_WHEEL_I                           = 0.0;
  public static final double LAUNCHER_TOP_WHEEL_D                           = 0.0;

  public static final double LAUNCHER_BOTTOM_WHEEL_F                        = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_P                        = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_I                        = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_D                        = 0.0;

  public static final double LAUNCHER_FEEDER_F                              = 0.0;
  public static final double LAUNCHER_FEEDER_P                              = 0.01;
  public static final double LAUNCHER_FEEDER_I                              = 0.0001;
  public static final double LAUNCHER_FEEDER_D                              = 0.0;
  public static final int    LAUNCHER_FEEDER_CRUISEVELOCITY                 = 0;
  public static final int    LAUNCHER_FEEDER_ACCELERATION                   = 0;

  public static final double LAUNCHER_PIVOT_F                               = 0.0;
  public static final double LAUNCHER_PIVOT_P                               = 0.0;
  public static final double LAUNCHER_PIVOT_I                               = 0.0;
  public static final double LAUNCHER_PIVOT_D                               = 0.0;
  public static final int    LAUNCHER_PIVOT_CRUISEVELOCITY                  = 0;
  public static final int    LAUNCHER_PIVOT_ACCELERATION                    = 0;

  /**
   * Subsystem-Specific Values For Commands
   */

  /* Chassis */
  public static final int    CHASSIS_INITIATION_LINE_ANGLE                  = 0;
  public static final int    CHASSIS_CLOSE_TRENCH_ANGLE                     = 0;
  public static final int    CHASSIS_FAR_TRENCH_ANGLE                       = 0;

  /* Chamber */
  public static final int    CHAMBER_ELEVATOR_POWER                         = 0;
  public static final int    CHAMBER_ELEVATOR_RPM                           = 0;

  /* Climber */
  public static final int    CLIMBER_REEL_MIN_POSITION                      = 0;
  public static final int    CLIMBER_REEL_MAX_POSITION                      = 90;
  public static final int    CLIMBER_REEL_JOG_MAGNITUDE                     = 5;

  public static final int    CLIMBER_ZIPLINE_MIN_POSITION                   = 0;
  public static final int    CLIMBER_ZIPLINE_MAX_POSITION                   = 90;
  public static final int    CLIMBER_ZIPLINE_JOG_MAGNITUDE                  = 0;

  public static final double CLIMBER_ZIPLINE_POWER                          = 0;

  /* Control Panel */
  public static final int    CONTROL_PANEL_SPINNER_POWER                    = 0;
  public static final int    CONTROL_PANEL_SPINNER_RPM                      = 0;

  /* Intake */
  public static final double INTAKE_WHEEL_POWER                             = -0.7;
  public static final int    INTAKE_WHEEL_RPM                               = 120;

  /* Launcher */
  public static final int    LAUNCHER_CLOSE_TRENCH_TOP_RPM                  = 0;
  public static final int    LAUNCHER_CLOSE_TRENCH_BOTTOM_RPM               = 0;
  public static final int    LAUNCHER_CLOSE_TRENCH_ANGLE                    = 0;

  public static final int    LAUNCHER_FAR_TRENCH_TOP_RPM                    = 0;
  public static final int    LAUNCHER_FAR_TRENCH_BOTTOM_RPM                 = 0;
  public static final int    LAUNCHER_FAR_TRENCH_ANGLE                      = 0;

  public static final int    LAUNCHER_INITIATION_LINE_TOP_RPM               = 0;
  public static final int    LAUNCHER_INITIATION_LINE_BOTTOM_RPM            = 0;
  public static final int    LAUNCHER_INITIATION_LINE_ANGLE                 = 0;

  public static final double LAUNCHER_FEEDER_POWER                          = 0;
  public static final int    LAUNCHER_FEEDER_RPM                            = 120;

  public static final int    LAUNCHER_PIVOT_MIN_ANGLE                       = 0;
  public static final int    LAUNCHER_PIVOT_MAX_ANGLE                       = 90;
  public static final int    LAUNCHER_PIVOT_JOG_MAGNITUDE                   = 5;

  public static final int    LAUNCHER_WHEEL_MAX_RPM                         = 6380; // Max RPM from TalonFX tech specs on CTRE.

  /**
   * Driver Joystick Map
   */

  /* Joystick */
  public static final int    DRIVER_JOYSTICK_PORT                           = 0;

  /* Buttons */
  public static final int    DRIVER_GEAR_SHIFT_BTN_ID                       = 2;

  public static final int    DRIVER_CHASSIS_TURN_TO_TARGET_BTN_ID           = 1;

  public static final int    DRIVER_TURN_CHASSIS_FOR_INITIATION_LINE_BTN_ID = 3;
  public static final int    DRIVER_TURN_CHASSIS_FOR_CLOSE_TRENCH_BTN_ID    = 4;
  public static final int    DRIVER_TURN_CHASSIS_FOR_FAR_TRENCH_BTN_ID      = 5;

  /**
   * Operator Joystick Map
   */

  /* Joystick */
  public static final int    OPERATOR_JOYSTICK_PORT                         = 1;
  

  /**
   * Buttons
   */

  /* Chamber */
  public static final int    OPERATOR_CHAMBER_ELEVATOR_POWER_BTN_ID         = 15;
  public static final int    OPERATOR_CHAMBER_ELEVATOR_RPM_BTN_ID           = 16;

  /* Climber */
  public static final int    OPERATOR_CLIMBER_JOG_REEL_POSITION_UP_BTN_ID   = 19;
  public static final int    OPERATOR_CLIMBER_JOG_REEL_POSITION_DOWN_BTN_ID = 20;

  public static final int    OPERATOR_CLIMBER_JOG_ZIPLINE_LEFT_BTN_ID       = 21;
  public static final int    OPERATOR_CLIMBER_JOG_ZIPLINE_RIGHT_BTN_ID      = 22;

  public static final int    OPERATOR_CLIMBER_POWER_BTN_ID                  = 23;

  /* Control Panel */
  public static final int    OPERATOR_CONTROL_PANEL_SPINNER_POWER_BTN_ID    = 17;
  public static final int    OPERATOR_CONTROL_PANEL_SPINNER_RPM_BTN_ID      = 18;
  
  /* Intake */
  public static final int    OPERATOR_TOGGLE_INTAKE_BTN_ID                  = 1;
  public static final int    OPERATOR_INTAKE_WHEEL_POWER_BTN_ID             = 4;
  public static final int    OPERATOR_INTAKE_WHEEL_RPM_BTN_ID               = 11;

  /* Launcher */
  public static final int    OPERATOR_LAUNCHER_WHEELS_POWER_BTN_ID          = 5;
  public static final int    OPERATOR_LAUNCHER_WHEELS_RPM_BTN_ID            = 9;
  public static final int    OPERATOR_LAUNCHER_WHEELS_STOP_BTN_ID           = 6;

  public static final int    OPERATOR_LAUNCHER_JOG_ANGLE_UP_BTN_ID          = 8;
  public static final int    OPERATOR_LAUNCHER_JOG_ANGLE_DOWN_BTN_ID        = 10;

  public static final int    OPERATOR_LAUNCHER_INIT_LINE_BTN_ID             = 7;
  public static final int    OPERATOR_LAUNCHER_CLOSE_TRENCH_BTN_ID          = 2;
  public static final int    OPERATOR_LAUNCHER_FAR_TRENCH_BTN_ID            = 3;

  public static final int    OPERATOR_LAUNCHER_FEEDER_POWER_BTN_ID          = 13;
  public static final int    OPERATOR_LAUNCHER_FEEDER_RPM_BTN_ID            = 14;
  
  /* Axes */
  public static final int    OPERATOR_LAUNCHER_WHEELS_SLIDER_ID             = 5;

  /**
   * Autonomous Constants
   */
  public static final int    K_ENCODER_TICKS_PER_REVOLUTION                 = 28300;
  public static final double K_WHEEL_DIAMETER_METERS                        = 0.1524;
  public static final double K_ENCODER_DISTANCE_PER_PULSE                   = (K_WHEEL_DIAMETER_METERS * Math.PI) / 
                                                                              (double) K_ENCODER_TICKS_PER_REVOLUTION;

  public static final boolean K_GYRO_REVERSED                               = true;

  /* Use robot characterization tool for these values. */
  public static final double K_S_VOLTS                                      = 0.372;
  public static final double K_V_VOLT_SECONDS_PER_METER                     = 3.09;
  public static final double K_A_VOLT_SECONDS_SQUARED_PER_METER             = 0.154;
  
  public static final double K_P_DRIVE_VEL                                  = 0.00425;

  public static final double K_TRACK_WIDTH_METERS                           = 0.774;
  public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS        = new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);

  /* Maximum voltage is 10V rather than nominal battery voltage of 12V for 
    "headroom" in dealing with "voltage sag." */
  public static final int    K_MAX_VOLTAGE                                  = 10;
  
  /* If these values are changed, must also edit max velocity/acceleration in PathWeaver. */
  public static final double K_MAX_SPEED_METERS_PER_SECOND                  = 2.5;
  public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED   = 3.0;

  public static final double K_RAMSETE_B                                    = 2;
  public static final double K_RAMSETE_ZETA                                 = 0.7;

}
