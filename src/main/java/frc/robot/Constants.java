package frc.robot;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

public class Constants 
{
  /**
   * Robot Map
   * A collection of ports and IDs for various objects representing hardware.
   */

  /* CAN IDs */

  public static final int CHASSIS_LEFT_MASTER_ID                            = 140;  //14
  public static final int CHASSIS_LEFT_SLAVE_ID                             = 150;  //15
  public static final int CHASSIS_RIGHT_MASTER_ID                           = 90;  //0
  public static final int CHASSIS_RIGHT_SLAVE_ID                            = 100;  //1
  
  public static final int INTAKE_WHEEL_INTAKE_ID                            = 0;

  public static final int CHAMBER_BASE_ID                                   = 60;
  public static final int CHAMBER_ELEVATOR_SLAVE_ID                         = 70;
  public static final int CHAMBER_ELEVATOR_MASTER_ID                        = 80; //8
  public static final int CHAMBER_WHEEL_ID                                  = 18;

  public static final int LAUNCHER_TOP_WHEEL_ID                             = 900; //9
  public static final int LAUNCHER_TOP_LEFT_MOTOR_ID                        = 7;
  public static final int LAUNCHER_TOP_RIGHT_MOTOR_ID                       = 5; 
  public static final int LAUNCHER_BOTTOM_WHEEL_ID                          = 6;  //10
  public static final int LAUNCHER_PIVOT_ID                                 = 110; //11

  public static final int CLIMBER_REEL_LEFT_ID                              = 18;  //12
  public static final int CLIMBER_REEL_RIGHT_ID                             = 19;  //13

  public static final int CLIMBER_LIFT_LEFT_ID                              = 140;  //14
  public static final int CLIMBER_LIFT_RIGHT_ID                             = 150;  //15

  public static final int CONTROL_PANEL_SPINNER_ID                          = 16;

  public static final int FRONT_RIGHT_ANGLE_MOTOR_ID                          = 13; //0
  public static final int FRONT_RIGHT_SPEED_MOTOR_ID                          = 12; //1

  public static final int FRONT_LEFT_ANGLE_MOTOR_ID                          = 15; //2
  public static final int FRONT_LEFT_SPEED_MOTOR_ID                          = 14; //3

  public static final int BACK_LEFT_ANGLE_MOTOR_ID                        = 8; //4
  public static final int BACK_LEFT_SPEED_MOTOR_ID                        = 9; //5

  public static final int BACK_RIGHT_ANGLE_MOTOR_ID                         = 10; //6
  public static final int BACK_RIGHT_SPEED_MOTOR_ID                         = 11; //7

  /* Pneumatic Ports */
  public static final int CHASSIS_GEARSHIFT_PORT_A                          = 1;
  public static final int CHASSIS_GEARSHIFT_PORT_B                          = 0;

  public static final int INTAKE_EXTENDER_PORT_A                            = 3;
  public static final int INTAKE_EXTENDER_PORT_B                            = 2;

  /* Digital IO Ports */
  public static final int CHAMBER_BALL_POS_ENTER_PORT_A                     = 1;
  public static final int CHAMBER_BALL_POS_ENTER_PORT_B                     = 0;
  public static final int CHAMBER_BALL_POS_EXIT_PORT_A                      = 3;
  public static final int CHAMBER_BALL_POS_EXIT_PORT_B                      = 2;
  
  public static final int CONTROLPANEL_COLOR_SENSOR_PORT                    = 10;

  /* PWM Ports */
  public static final int CHAMBER_LEDS_PORT                                 = 0;

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
  public static final double K_CHASSIS_TURN_VISION_MIN                      = 0.1;
  public static final double CHASSIS_TURN_ERROR_THRESHOLD                   = 0.5;

  public static final double K_CHASSIS_ANGLE_P                              = 0.19;
  public static final double K_CHASSIS_ANGLE_I                              = 0.0000175;   //previous: 0.0000175
  public static final double K_CHASSIS_ANGLE_D                              = 0;

  /* Chamber */
  public static final double CHAMBER_ELEVATOR_F                             = 0.0;
  public static final double CHAMBER_ELEVATOR_P                             = 1.0;
  public static final double CHAMBER_ELEVATOR_I                             = 0.0;
  public static final double CHAMBER_ELEVATOR_D                             = 0.0;
  public static final int    CHAMBER_ELEVATOR_CRUISEVELOCITY                = 2500;
  public static final int    CHAMBER_ELEVATOR_ACCELERATION                  = 5000;

  public static final double CHAMBER_BASE_F                                 = 0.0;
  public static final double CHAMBER_BASE_P                                 = 0.1;
  public static final double CHAMBER_BASE_I                                 = 0.001;
  public static final double CHAMBER_BASE_D                                 = 0.0;

  public static final double CHAMBER_F                                     = 0.0;
  public static final double CHAMBER_P                                     = 1.0;
  public static final double CHAMBER_I                                     = 0.0;
  public static final double CHAMBER_D                                     = 0.0;

  /* Climber */

  /* Control Panel */   
  public static final double CONTROL_PANEL_SPINNER_F                        = 0.0;
  public static final double CONTROL_PANEL_SPINNER_P                        = 0.0;
  public static final double CONTROL_PANEL_SPINNER_I                        = 0.0;
  public static final double CONTROL_PANEL_SPINNER_D                        = 0.0;

  /* Intake */
  public static final double INTAKE_WHEEL_INTAKE_F                          = 0.0;
  public static final double INTAKE_WHEEL_INTAKE_P                          = 0.03;
  public static final double INTAKE_WHEEL_INTAKE_I                          = 0.0002;
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

  public static final int    LAUNCHER_PIVOT_ADC_THRESHOLD                   = 50;
  public static final double LAUNCHER_PIVOT_ANGLE_P                         = 0.005;
  public static final double LAUNCHER_PIVOT_ANGLE_MIN                       = 0.02;

  public static final double LAUNCHER_AIM_VISION_P                          = 0.07;
  public static final double LAUNCHER_AIM_VISION_MIN                        = 0.03;
  
  /**
   * Subsystem-Specific Values For Commands
   */

  /* Chassis */
  public static final String[] CHASSIS_TRAJECTORIES_NAMES                   = {"sixBall",
                                                                               "sixBall2",
                                                                               "straightForward",
                                                                               "straightForward2",
                                                                               "straightBackward",
                                                                               "test",
                                                                               "test2"};

  public static final double CHASSIS_MAX_POWER                              = 0.7;

  //public static final double kSensorUnitsPerRotation                        = 4096;
  //public static final double kGearRatio                                     = 3 / 20;

  public static final double WHEEL_MOTOR_TICKS_PER_REVOLUTION               = 2048 * 12; //kSensorUnitsPerRotation / kGearRatio;



  /* Chamber */
  public static final int    CHAMBER_BASE_TPR                               = 4096;
  public static final int    CHAMBER_BASE_RPM                               = 300;

  public static final double CHAMBER_BALL_THRESHOLD                         = 3.0;
  public static final double CHAMBER_BALL_STEP_DIST                         = 4000;

  public static final int    CHAMBER_TOTAL_NUM_OF_LEDS                      = 150;
  public static final int    CHAMBER_SIDE_NUM_OF_LEDS                       = 56;
  public static final int    CHAMBER_TOP_NUM_OF_LEDS                        = 38;

  /* Climber */
  public static final int    CLIMBER_REEL_MIN_POSITION                      = 0;
  public static final int    CLIMBER_REEL_MAX_POSITION                      = 4096;
  public static final int    CLIMBER_REEL_STEP_MAGNITUDE                    = 2000;

  public static final int    CLIMBER_LIFT_MIN_POSITION                      = 0;
  public static final int    CLIMBER_LIFT_MAX_POSITION                      = 4096;
  public static final int    CLIMBER_LIFT_STEP_MAGNITUDE                    = 2000;

  public static final double CLIMBER_REEL_EXTEND_POWER                      = 0.6;
  public static final double CLIMBER_REEL_RETRACT_POWER                     = -0.6;

  public static final double CLIMBER_LIFT_UNREEL_POWER                      = 1.0;

  public static final double CLIMBER_LIFT_EXTEND_POWER                      = 1.0;
  public static final double CLIMBER_LIFT_RETRACT_POWER                     = -1.0;

  /* Control Panel */
  public static final int    CONTROL_PANEL_SPINNER_TPR                      = 0;

  public static final int    CONTROL_PANEL_SPINNER_POWER                    = 0;
  public static final int    CONTROL_PANEL_SPINNER_RPM                      = 0;

  /* Intake */
  public static final int    INTAKE_WHEEL_TPR                               = 16000;
  public static final int    INTAKE_WHEEL_RPM                               = 750;
  public static final double INTAKE_WHEEL_POWER                             = 0.5;

  /* Launcher */
  public static final int    LAUNCHER_WHEEL_MAX_RPM                         = 6000;
  
  public static final int    LAUNCHER_TOP_WHEEL_TPR                         = 2048;
  public static final int    LAUNCHER_BOTTOM_WHEEL_TPR                      = 2048;
  public static final int    LAUNCHER_RPM_ERROR_TRESHOLD                    = 500;

  public static final int    LAUNCHER_PIVOT_MIN_ADC                         = 1700;
  public static final int    LAUNCHER_PIVOT_MAX_ADC                         = 3600;
  public static final double LAUNCHER_PIVOT_JOG_MAGNITUDE                   = 0.25;
  public static final double LAUNCHER_PIVOT_ERROR_TRESHOLD                  = 0.5;

  public static final int    LAUNCHER_CLOSE_TRENCH_TOP_RPM                  = 5500;
  public static final int    LAUNCHER_CLOSE_TRENCH_BOTTOM_RPM               = 5500;
  public static final int    LAUNCHER_CLOSE_TRENCH_ANGLE                    = 3000;

  public static final int    LAUNCHER_FAR_TRENCH_TOP_RPM                    = 3000;
  public static final int    LAUNCHER_FAR_TRENCH_BOTTOM_RPM                 = 3000;
  public static final int    LAUNCHER_FAR_TRENCH_ANGLE                      = 3000;

  public static final int    LAUNCHER_INITIATION_LINE_TOP_RPM               = 2600;
  public static final int    LAUNCHER_INITIATION_LINE_BOTTOM_RPM            = 2600;
  public static final int    LAUNCHER_INITIATION_LINE_ANGLE                 = 3600;

  public static final int    LAUNCHER_AUTO_INIT_TOP_RPM                     = 3000;
  public static final int    LAUNCHER_AUTO_INIT_BOTTOM_RPM                  = 3000;
  public static final int    LAUNCHER_AUTO_INIT_ANGLE                       = 3600;

  public static final int    LAUNCHER_AUTO_TRENCH_TOP_RPM                   = 4500;
  public static final int    LAUNCHER_AUTO_TRENCH_BOTTOM_RPM                = 4500;
  public static final int    LAUNCHER_AUTO_TRENCH_ANGLE                     = 3000;//3350;

  public static final double LAUNCHER_GREEN_ZONE_TOP_MOTOR_POWER            = 0.25;
  public static final double LAUNCHER_YELLOW_ZONE_TOP_MOTOR_POWER           = 0.5;
  public static final double LAUNCHER_BLUE_ZONE_TOP_MOTOR_POWER             = 0.75;
  public static final double LAUNCHER_RED_ZONE_TOP_MOTOR_POWER              = 1.0;

  public static final double LAUNCHER_GREEN_ZONE_BOTTOM_MOTOR_POWER         = 0.25;
  public static final double LAUNCHER_YELLOW_ZONE_BOTTOM_MOTOR_POWER        = 0.5;
  public static final double LAUNCHER_BLUE_ZONE_BOTTOM_MOTOR_POWER          = 0.75;
  public static final double LAUNCHER_RED_ZONE_BOTTOM_MOTOR_POWER           = 1.0;
 
  // Swerve Drive
  public static final double SWERVE_DRIVE_WHEEL_AXLE_LENGTH                 = 36;
  public static final double SWERVE_DRIVE_WHEEL_AXLE_WIDTH                  = 48;
  public static final double SWERVE_DRIVE_WHEEL_AXLE_DIAGONAL               = 60;
  
  /**
   * Driver Joystick Map
   */

  /* Joystick */
  public static final int    DRIVER_JOYSTICK_PORT                           = 0;

  /* Buttons */
  public static final int    DRIVER_GEAR_SHIFT_BTN_ID                       = 2;

  public static final int    DRIVER_TOGGLE_LL_BTN_ID                        = 7;

  public static final int    DRIVER_LAUNCHER_VISION_BTN_ID                  = 1;

  public static final int    DRIVER_SET_HEADING_BTN_ID                      = 4;

  // Defense Mode
  public static final int    DRIVER_TOGGLE_DEFENSE_MODE_BTN_ID              = 3;

  /**
   * Operator Joystick Map
   */

  /* Joystick */
  public static final int    OPERATOR_JOYSTICK_PORT                         = 1;
  
  /**
   * Buttons
   */

   /* Chamber */
  public static final int    OPERATOR_CHAMBER_BASE_RPM_BTN_ID               = 140; //14
  
  public static final int    OPERATOR_CHAMBER_ELEVATOR_LAUNCH_BTN_ID        = 8; //11
  public static final int    OPERATOR_CHAMBER_ELEVATOR_DOWN_BTN_ID          = 9; //10

  /* Climber */
  public static final int    OPERATOR_CLIMBER_EXTEND_LEFT_BTN_ID            = 300; //3
  public static final int    OPERATOR_CLIMBER_RETRACT_LEFT_BTN_ID           = 600; //6

  public static final int    OPERATOR_CLIMBER_EXTEND_RIGHT_BTN_ID           = 4; //4
  public static final int    OPERATOR_CLIMBER_RETRACT_RIGHT_BTN_ID          = 5; //5

  public static final int    OPERATOR_CLIMBER_REEL_BTN_ID                   = 70; //7
  public static final int    OPERATOR_CLIMBER_LIFT_BTN_ID                   = 19; //19

  /* Control Panel */
  // public static final int    OPERATOR_CONTROL_PANEL_SPINNER_POWER_BTN_ID    = 19;
  // public static final int    OPERATOR_CONTROL_PANEL_SPINNER_RPM_BTN_ID      = 18;

  /* Intake */
  public static final int    OPERATOR_TOGGLE_INTAKE_BTN_ID                  = 15; //15
  public static final int    OPERATOR_INTAKE_WHEEL_RPM_BTN_ID               = 11;

  /* Launcher */
  public static final int    OPERATOR_LAUNCHER_JOG_ANGLE_UP_BTN_ID          = 120; //12
  public static final int    OPERATOR_LAUNCHER_JOG_ANGLE_DOWN_BTN_ID        = 130; //13

  public static final int    OPERATOR_LAUNCHER_PRESET_BTN_ID                = 60; //6
  public static final int    OPERATOR_LAUNCHER_PRESET_INIT_BTN_ID           = 80; //8
  public static final int    OPERATOR_LAUNCHER_PRESET_CLOSE_BTN_ID          = 90; //9

  public static final int    OP_LAUNCHER_PRESET_GREEN_BTN_ID                = 3;
  public static final int    OP_LAUNCHER_PRESET_YELLOW_BTN_ID               = 6; 
  public static final int    OP_LAUNCHER_PRESET_BLUE_BTN_ID                 = 4;
  public static final int    OP_LAUNCHER_PRESET_RED_BTN_ID                  = 5;

  /* LEDs */
  public static final int    OPERATOR_LEDS_CHASE_INWARD_BTN_ID              = 1000; //1
  public static final int    OPERATOR_LEDS_CHASE_OUTWARD_BTN_ID             = 200; //2
  public static final int    OPERATOR_LEDS_RAINBOW_BTN_ID                   = 30; //3

  
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
    "headroom" in dealing with voltage sag." */
  public static final int    K_MAX_VOLTAGE                                  = 10;
  
  public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED   = 3.0;

  public static final double K_RAMSETE_B                                    = 2;
  public static final double K_RAMSETE_ZETA                                 = 0.7;

  /* Ask build captains what type of swerve module we have. */
  public static final double SWERVE_DRIVE_MAX_VOLTAGE                      = 4.95;

  public static final Translation2d BACK_LEFT_WHEEL_LOCATION   = new Translation2d(-12.5 , 10.75); //TODO forward is +X and left is +Y
  public static final Translation2d BACK_RIGHT_WHEEL_LOCATION  = new Translation2d(-12.5 , -10.75); //TODO forward is +X and left is +Y
  public static final Translation2d FRONT_LEFT_WHEEL_LOCATION  = new Translation2d(12.5 , 10.75); //TODO forward is +X and left is +Y        
  public static final Translation2d FRONT_RIGHT_WHEEL_LOCATION = new Translation2d(12.5 , -10.75); //TODO forward is +X and left is +Y

  public static final Constraints ANGLE_CONTROLLER_CONSTRAINTS = new Constraints(0.0, 0.0);


}
