package frc.robot;

public class Constants 
{
  /**
   * PID Constants
   */
  public static final int K_SLOT_IDX      = 0;
  public static final int K_PID_LOOP_IDX  = 0;
  public static final int K_TIMEOUT_MS    = 30;

  /* Launcher */
  public static final double LAUNCHER_TOP_WHEEL_F = 0.0;
  public static final double LAUNCHER_TOP_WHEEL_P = 0.0;
  public static final double LAUNCHER_TOP_WHEEL_I = 0.0;
  public static final double LAUNCHER_TOP_WHEEL_D = 0.0;

  public static final double LAUNCHER_BOTTOM_WHEEL_F = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_P = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_I = 0.0;
  public static final double LAUNCHER_BOTTOM_WHEEL_D = 0.0;

  public static final double LAUNCHER_FEEDER_F              = 0.0;
  public static final double LAUNCHER_FEEDER_P              = 0.0;
  public static final double LAUNCHER_FEEDER_I              = 0.0;
  public static final double LAUNCHER_FEEDER_D              = 0.0;
  public static final int    LAUNCHER_FEEDER_CRUISECONTROL = 0;
  public static final int    LAUNCHER_FEEDER_ACCELERATION  = 0;

  public static final double LAUNCHER_PIVOT_F              = 0.0;
  public static final double LAUNCHER_PIVOT_P              = 0.0;
  public static final double LAUNCHER_PIVOT_I              = 0.0;
  public static final double LAUNCHER_PIVOT_D              = 0.0;
  public static final int    LAUNCHER_PIVOT_CRUISECONTROL = 0;
  public static final int    LAUNCHER_PIVOT_ACCELERATION  = 0;

  

  /**
   * Robot Map
   * A collection of ports and IDs for various objects representing hardware.
   */

  /* CAN IDs */
  public static final int CHASSIS_LEFT_MASTER_ID        = 1;
  public static final int CHASSIS_LEFT_SLAVE_ID         = 2;
  public static final int CHASSIS_RIGHT_MASTER_ID       = 3;
  public static final int CHASSIS_RIGHT_SLAVE_ID        = 4;
  public static final int INTAKE_WHEEL_INTAKE_ID        = 5;
  public static final int CHAMBER_START_ID              = 6;
  public static final int CHAMBER_END_ID                = 7;
  public static final int LAUNCHER_WHEEL_TOP_ID         = 8;
  public static final int LAUNCHER_WHEEL_BOTTOM_ID      = 9;
  public static final int LAUNCHER_FEEDER_ID            = 10;
  public static final int LAUNCHER_PIVOT_ID             = 11;
  public static final int CLIMBER_STRING_PULLER_ID      = 12;
  public static final int CONTROLPANEL_WHEEL_SPINNER_ID = 13;

  /* Pneumatic Ports */
  public static final int CHASSIS_GEARSHIFT_PORT_A = 0;
  public static final int CHASSIS_GEARSHIFT_PORT_B = 1;
  public static final int INTAKE_EXTENDER_PORT_A   = 2;
  public static final int INTAKE_EXTENDER_PORT_B   = 3;
  public static final int CLIMBER_PTO_PORT_A       = 4;
  public static final int CLIMBER_PTO_PORT_B       = 5;

  /* Digital IO Ports */
  public static final int CHAMBER_BALL_POS_1_PORT        = 0;
  public static final int CHAMBER_BALL_POS_2_PORT        = 1;
  public static final int CHAMBER_BALL_POS_3_PORT        = 2;
  public static final int CHAMBER_BALL_POS_4_PORT        = 3;
  public static final int LAUNCHER_BALL_SWITCH_PORT      = 4;
  public static final int CONTROLPANEL_COLOR_SENSOR_PORT = 5;

  /* PWM Ports */
  public static final int CLIMBER_RATCHET_LEFT_PORT  = 0;
  public static final int CLIMBER_RATCHET_RIGHT_PORT = 1;

  /**
   * Operator Joystick Buttons Map
   */
  public static final int OPERATOR_BTN_LAUNCHER_SPINWHEELS_ID = 0;

  /**
   * Driver Joystick Buttons Map
   */
  
}
