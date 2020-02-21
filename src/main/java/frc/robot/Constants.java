package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class Constants 
{
  /**
   * Robot Map
   * A collection of ports and IDs for various objects representing hardware.
   */

  /* CAN IDs */
  public static final int CHASSIS_LEFT_MASTER_ID                            = 1;
  public static final int CHASSIS_LEFT_SLAVE_ID                             = 2;
  public static final int CHASSIS_RIGHT_MASTER_ID                           = 3;
  public static final int CHASSIS_RIGHT_SLAVE_ID                            = 4;
  
  public static final int INTAKE_WHEEL_INTAKE_ID                            = 5;

  public static final int CHAMBER_BASE_ID                                   = 6;
  public static final int CHAMBER_ELEVATOR_FRONT_ID                         = 7;
  public static final int CHAMBER_ELEVATOR_BACK_ID                          = 8;

  public static final int LAUNCHER_TOP_WHEEL_ID                             = 9;
  public static final int LAUNCHER_BOTTOM_WHEEL_ID                          = 10;
  public static final int LAUNCHER_PIVOT_ID                                 = 11;

  public static final int CLIMBER_REEL_LEFT_ID                              = 12;
  public static final int CLIMBER_REEL_RIGHT_ID                             = 13;

  public static final int CONTROL_PANEL_SPINNER_ID                          = 14;
 
  /* Pneumatic Ports */
  public static final int CHASSIS_GEARSHIFT_PORT_A                          = 1;
  public static final int CHASSIS_GEARSHIFT_PORT_B                          = 0;

  public static final int INTAKE_EXTENDER_PORT_A                            = 2;
  public static final int INTAKE_EXTENDER_PORT_B                            = 3;

  public static final int CLIMBER_PTO_PORT_A                                = 4;
  public static final int CLIMBER_PTO_PORT_B                                = 5;

  /* Digital IO Ports */
  public static final int CHAMBER_BALL_POS_ENTER_PORT_A                     = 1;
  public static final int CHAMBER_BALL_POS_ENTER_PORT_B                     = 0;
  public static final int CHAMBER_BALL_POS_EXIT_PORT_A                      = 3;
  public static final int CHAMBER_BALL_POS_EXIT_PORT_B                      = 2;
  
  public static final int CONTROLPANEL_COLOR_SENSOR_PORT                    = 10;

  /* PWM Ports */
  public static final int CHAMBER_LEDS_PORT                                 = 0;

  public static final int CLIMBER_LOCK_RATCHET_LEFT_PORT                    = 1;
  public static final int CLIMBER_LOCK_RATCHET_RIGHT_PORT                   = 2;

  /* Analog Ports */
  public static final int LAUNCHER_PIVOT_FEEDBACK_PORT                      = 0;

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

  public static final double K_CHASSIS_TURN_VISION_P                        = 0.02;
  public static final double K_CHASSIS_TURN_VISION_MIN                      = 0.06;

  /* Chamber */
  public static final double CHAMBER_ELEVATOR_FRONT_F                       = 0.0;
  public static final double CHAMBER_ELEVATOR_FRONT_P                       = 0.0;
  public static final double CHAMBER_ELEVATOR_FRONT_I                       = 0.0;
  public static final double CHAMBER_ELEVATOR_FRONT_D                       = 0.0;
  public static final int    CHAMBER_ELEVATOR_FRONT_CRUISEVELOCITY          = 0;
  public static final int    CHAMBER_ELEVATOR_FRONT_ACCELERATION            = 0;

  public static final double CHAMBER_ELEVATOR_BACK_F                        = 0.0;
  public static final double CHAMBER_ELEVATOR_BACK_P                        = 0.0;
  public static final double CHAMBER_ELEVATOR_BACK_I                        = 0.0;
  public static final double CHAMBER_ELEVATOR_BACK_D                        = 0.0;
  public static final int    CHAMBER_ELEVATOR_BACK_CRUISEVELOCITY           = 0;
  public static final int    CHAMBER_ELEVATOR_BACK_ACCELERATION             = 0;

  public static final double CHAMBER_BASE_F                                 = 0.0;
  public static final double CHAMBER_BASE_P                                 = 0.0;
  public static final double CHAMBER_BASE_I                                 = 0.0;
  public static final double CHAMBER_BASE_D                                 = 0.0;

  /* Climber */
  public static final double CLIMBER_REEL_LEFT_F                            = 0.0;
  public static final double CLIMBER_REEL_LEFT_P                            = 0.0;
  public static final double CLIMBER_REEL_LEFT_I                            = 0.0;
  public static final double CLIMBER_REEL_LEFT_D                            = 0.0;
  public static final int    CLIMBER_REEL_LEFT_CRUISEVELOCITY               = 0;
  public static final int    CLIMBER_REEL_LEFT_ACCELERATION                 = 0;

  public static final double CLIMBER_REEL_RIGHT_F                           = 0.0;
  public static final double CLIMBER_REEL_RIGHT_P                           = 0.0;
  public static final double CLIMBER_REEL_RIGHT_I                           = 0.0;
  public static final double CLIMBER_REEL_RIGHT_D                           = 0.0;
  public static final int    CLIMBER_REEL_RIGHT_CRUISEVELOCITY              = 0;
  public static final int    CLIMBER_REEL_RIGHT_ACCELERATION                = 0;

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
  public static final double LAUNCHER_TOP_WHEEL_P                           = 0.01;
  public static final double LAUNCHER_TOP_WHEEL_I                           = 0.0001;
  public static final double LAUNCHER_TOP_WHEEL_D                           = 0.0;

  public static final double LAUNCHER_BOTTOM_WHEEL_F                        = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_P                        = 0.01;
  public static final double LAUNCHER_BOTTOM_WHEEL_I                        = 0.0001;
  public static final double LAUNCHER_BOTTOM_WHEEL_D                        = 0.0;

  public static final double LAUNCHER_PIVOT_F                               = 0.0;
  public static final double LAUNCHER_PIVOT_P                               = 10.0;
  public static final double LAUNCHER_PIVOT_I                               = 0.0;
  public static final double LAUNCHER_PIVOT_D                               = 0.0;

  public static final int    LAUNCHER_PIVOT_MIN_ADC                         = 250;
  public static final int    LAUNCHER_PIVOT_MAX_ADC                         = 3700;

  public static final double LAUNCHER_AIM_VISION_P                          = 0.07;
  public static final double LAUNCHER_AIM_VISION_MIN                        = 0.03;

  /**
   * Subsystem-Specific Values For Commands
   */

  /* Chamber */
  public static final double CHAMBER_ELEVATOR_POWER                         = 0.25;

  public static final double CHAMBER_BASE_POWER                             = 0.75;
  public static final int    CHAMBER_BASE_RPM                               = 500;

  public static final int    CHAMBER_ENTERING_BALLPOS                       = 0;
  public static final int    CHAMBER_EXITING_BALLPOS                        = 1;

  public static final double CHAMBER_BALL_FAR_DIST                          = 10.0;
  public static final double CHAMBER_BALL_NEAR_DIST                         = 5.0;

  public static final double CHAMBER_BALL_STEP_DIST                         = 2000;

  public static final int    CHAMBER_NUMBER_OF_LEDS                         = 60;

  /* Climber */
  public static final double CLIMBER_REEL_POWER                             = 0.25;
  
  public static final int    CLIMBER_REEL_MIN_POSITION                      = 0;
  public static final int    CLIMBER_REEL_MAX_POSITION                      = 4096;

  public static final double CLIMBER_LOCK_RATCHET_RELEASED_POSITION         = 0.0;
  public static final double CLIMBER_LOCK_RATCHET_LOCKED_POSITION           = 1.0;

  /* Control Panel */
  public static final int    CONTROL_PANEL_SPINNER_POWER                    = 0;
  public static final int    CONTROL_PANEL_SPINNER_RPM                      = 0;

  /* Intake */
  public static final double INTAKE_WHEEL_POWER                             = -0.5;
  public static final int    INTAKE_WHEEL_RPM                               = 500;

  /* Launcher */
  public static final int    LAUNCHER_WHEEL_MAX_RPM                         = 2500; // Max RPM from TalonFX at full power

  public static final int    LAUNCHER_PIVOT_MIN_ANGLE                       = 0;
  public static final int    LAUNCHER_PIVOT_MAX_ANGLE                       = 90;
  public static final int    LAUNCHER_PIVOT_JOG_MAGNITUDE                   = 5; // in degrees.

  public static final int    LAUNCHER_CLOSE_TRENCH_TOP_RPM                  = 0;
  public static final int    LAUNCHER_CLOSE_TRENCH_BOTTOM_RPM               = 0;
  public static final int    LAUNCHER_CLOSE_TRENCH_ANGLE                    = 0;

  public static final int    LAUNCHER_FAR_TRENCH_TOP_RPM                    = 0;
  public static final int    LAUNCHER_FAR_TRENCH_BOTTOM_RPM                 = 0;
  public static final int    LAUNCHER_FAR_TRENCH_ANGLE                      = 0;

  public static final int    LAUNCHER_INITIATION_LINE_TOP_RPM               = 0;
  public static final int    LAUNCHER_INITIATION_LINE_BOTTOM_RPM            = 0;
  public static final int    LAUNCHER_INITIATION_LINE_ANGLE                 = 0;

  /**
   * Driver Joystick Map
   */

  /* Joystick */
  public static final int    DRIVER_JOYSTICK_PORT                           = 0;

  /* Buttons */
  public static final int    DRIVER_GEAR_SHIFT_BTN_ID                       = 2;

  public static final int    DRIVER_TOGGLE_LL_BTN_ID                        = 7;

  public static final int    DRIVER_CHASSIS_TURN_TO_TARGET_BTN_ID           = 1;

  /**
   * Operator Joystick Map
   */

  /* Joystick */
  public static final int    OPERATOR_JOYSTICK_PORT                         = 1;
  
  /**
   * Buttons
   */

  /* Chamber */
  public static final int    OPERATOR_CHAMBER_BASE_POWER_BTN_ID             = 3;
  public static final int    OPERATOR_CHAMBER_BASE_RPM_BTN_ID               = 26;
  
  public static final int    OPERATOR_CHAMBER_ELEVATOR_POWER_BTN_ID         = 2;

  /* Climber */
  public static final int    OPERATOR_CLIMBER_EXTEND_POWER_BTN_ID           = 30;
  public static final int    OPERATOR_CLIMBER_RETRACT_POWER_BTN_ID          = 31;
  
  public static final int    OPERATOR_CLIMBER_EXTEND_POSITION_BTN_ID        = 32;
  public static final int    OPERATOR_CLIMBER_RETRACT_POSITION_BTN_ID       = 33;

  public static final int    OPERATOR_CLIMBER_TOGGLE_PTO_BTN_ID             = 34;

  public static final int    OPERATOR_CLIMBER_TOGGLE_LOCK_RATCHETS_BTN_ID   = 35;

  /* Control Panel */
  public static final int    OPERATOR_CONTROL_PANEL_SPINNER_POWER_BTN_ID    = 17;
  public static final int    OPERATOR_CONTROL_PANEL_SPINNER_RPM_BTN_ID      = 18;
  
  /* Intake */
  public static final int    OPERATOR_TOGGLE_INTAKE_BTN_ID                  = 1;
  public static final int    OPERATOR_INTAKE_WHEEL_POWER_BTN_ID             = 4;
  public static final int    OPERATOR_INTAKE_WHEEL_RPM_BTN_ID               = 11;

  /* Launcher */
  public static final int    OPERATOR_LAUNCHER_WHEELS_POWER_BTN_ID          = 5;
  public static final int    OPERATOR_LAUNCHER_WHEELS_RPM_BTN_ID            = 15;
  public static final int    OPERATOR_LAUNCHER_WHEELS_STOP_BTN_ID           = 6;

  public static final int    OPERATOR_LAUNCHER_JOG_ANGLE_UP_BTN_ID          = 9;
  public static final int    OPERATOR_LAUNCHER_JOG_ANGLE_DOWN_BTN_ID        = 10;

  public static final int    OPERATOR_LAUNCHER_INIT_LINE_BTN_ID             = 7;
  public static final int    OPERATOR_LAUNCHER_CLOSE_TRENCH_BTN_ID          = 8;
  public static final int    OPERATOR_LAUNCHER_FAR_TRENCH_BTN_ID            = 25;
  
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
  public static final double K_MAX_SPEED_METERS_PER_SECOND                  = 1.0;//2.5;
  public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED   = 3.0;

  public static final double K_RAMSETE_B                                    = 2;
  public static final double K_RAMSETE_ZETA                                 = 0.7;

}
