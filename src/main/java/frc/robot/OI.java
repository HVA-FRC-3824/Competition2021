package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefenseMode;

public class OI 
{
  /**
   * Declare all joysticks and buttons here.
   */

  /* 
   * Driver Joystick 
   */
  private static Joystick       m_driverJoystick;

  private static JoystickButton m_gearShiftBtn;

  private static JoystickButton m_toggleLimelightBtn;

  private static JoystickButton m_setLauncherVisionBtn;

  private static JoystickButton m_setHeadingBtn;

  /* 
   * Operator Joystick
   */
  private static Joystick       m_operatorJoystick;
  
  /* Chamber */
  private static JoystickButton m_setChamberBaseRPMBtn;

  private static JoystickButton m_setChamberElevatorToLaunchBtn;
  private static JoystickButton m_setChamberElevatorDownBtn;

  /* Climber */
  // private static JoystickButton m_extendClimberReelPositionBtn;
  // private static JoystickButton m_retractClimberReelPositionBtn;

  // private static JoystickButton m_extendClimberLiftPositionBtn;
  // private static JoystickButton m_retractClimberLiftPositionBtn;
  private static JoystickButton m_extendClimberLeftBtn;
  private static JoystickButton m_retractClimberLeftBtn;

  private static JoystickButton m_extendClimberRightBtn;
  private static JoystickButton m_retractClimberRightBtn;

  /* Control Panel */
  // private static JoystickButton m_setControlPanelSpinnerPowerBtn;
  // private static JoystickButton m_setControlPanelSpinnerRPMBtn;
  
  /* Intake */
  private static JoystickButton m_toggleIntakePistonsBtn;
  private static JoystickButton m_setIntakeWheelRPMBtn;

  /* Launcher */
  private static JoystickButton m_jogLauncherAngleUpBtn;
  private static JoystickButton m_jogLauncherAngleDownBtn;

  private static JoystickButton m_setLauncherPresetBtn;

  private static JoystickButton m_setLauncherPresetGreenBtn;
  private static JoystickButton m_setLauncherPresetYellowBtn;
  private static JoystickButton m_setLauncherPresetBlueBtn;
  private static JoystickButton m_setLauncherPresetRedBtn;


  /* LEDs */
  private static JoystickButton m_LEDsChaseInwardBtn;
  private static JoystickButton m_LEDsChaseOutwardBtn;
  private static JoystickButton m_LEDsRainbowBtn;


  /* Defense mode */
  private static JoystickButton m_toggleDefenseModeBtn;

  public OI() 
  {
    /**
     * Instantiate the declared joysticks and joystick buttons here.
     */

    /*
     * Driver Joystick 
     */
    m_driverJoystick                  = new Joystick(Constants.DRIVER_JOYSTICK_PORT);

    m_gearShiftBtn                    = new JoystickButton(m_driverJoystick, Constants.DRIVER_GEAR_SHIFT_BTN_ID);

    m_toggleLimelightBtn              = new JoystickButton(m_driverJoystick, Constants.DRIVER_TOGGLE_LL_BTN_ID);

    m_setLauncherVisionBtn            = new JoystickButton(m_driverJoystick, Constants.DRIVER_LAUNCHER_VISION_BTN_ID);

    m_toggleDefenseModeBtn            = new JoystickButton(m_driverJoystick, Constants.DRIVER_TOGGLE_DEFENSE_MODE_BTN_ID);

    m_setHeadingBtn                   = new JoystickButton(m_driverJoystick, Constants.DRIVER_SET_HEADING_BTN_ID);

    /* 
     * Operator Joystick
     */    
    m_operatorJoystick                = new Joystick(Constants.OPERATOR_JOYSTICK_PORT);

    /* Chamber */
    m_setChamberBaseRPMBtn            = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_BASE_RPM_BTN_ID);

    m_setChamberElevatorToLaunchBtn   = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_ELEVATOR_LAUNCH_BTN_ID);
    m_setChamberElevatorDownBtn       = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_ELEVATOR_DOWN_BTN_ID);

    /* Climber */
    m_extendClimberLeftBtn            = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_EXTEND_LEFT_BTN_ID);
    m_retractClimberLeftBtn           = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_RETRACT_LEFT_BTN_ID);

    m_extendClimberRightBtn           = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_EXTEND_RIGHT_BTN_ID);
    m_retractClimberRightBtn          = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_RETRACT_RIGHT_BTN_ID);

    /* Control Panel */
    // m_setControlPanelSpinnerPowerBtn  = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CONTROL_PANEL_SPINNER_POWER_BTN_ID);
    // m_setControlPanelSpinnerRPMBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CONTROL_PANEL_SPINNER_RPM_BTN_ID);

    /* Intake */
    m_toggleIntakePistonsBtn          = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_TOGGLE_INTAKE_BTN_ID);
    m_setIntakeWheelRPMBtn            = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_INTAKE_WHEEL_RPM_BTN_ID);

    /* Launcher */
    m_jogLauncherAngleUpBtn           = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_JOG_ANGLE_UP_BTN_ID);
    m_jogLauncherAngleDownBtn         = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_JOG_ANGLE_DOWN_BTN_ID);

    m_setLauncherPresetBtn            = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_PRESET_BTN_ID);

    m_setLauncherPresetGreenBtn       = new JoystickButton(m_operatorJoystick, Constants.OP_LAUNCHER_PRESET_GREEN_BTN_ID);
    m_setLauncherPresetYellowBtn      = new JoystickButton(m_operatorJoystick, Constants.OP_LAUNCHER_PRESET_YELLOW_BTN_ID);
    m_setLauncherPresetBlueBtn        = new JoystickButton(m_operatorJoystick, Constants.OP_LAUNCHER_PRESET_BLUE_BTN_ID);
    m_setLauncherPresetRedBtn         = new JoystickButton(m_operatorJoystick, Constants.OP_LAUNCHER_PRESET_RED_BTN_ID);

  
    /* LEDs */
    m_LEDsChaseInwardBtn              = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LEDS_CHASE_INWARD_BTN_ID);
    m_LEDsChaseOutwardBtn             = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LEDS_CHASE_OUTWARD_BTN_ID);
    m_LEDsRainbowBtn                  = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LEDS_RAINBOW_BTN_ID);

  }

  /**
   * Allows use of driverJoystick/operatorJoystick object outside of OI class.
   * @return access to driverJoystick/operatorJoystick values/attributes.
   */
  public Joystick getDriverJoystick() 
  {
    return m_driverJoystick;
  }

  public Joystick getOperatorController() 
  {
    return m_operatorJoystick;
  }

  /**
   * Bind commands to joystick buttons.
   * This method is run after the instantiation of the inline commands class because if run before,
   * the inline commands binded to the joystick buttons wouldn't have a reference point.
   */
  public void configureButtonBindings()
  {
    /* Chamber Buttons */
    // m_setChamberBaseRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberBaseRPM);
    // m_setChamberBaseRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChamberBase);

    m_setChamberElevatorToLaunchBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberElevatorToLaunch);
    m_setChamberElevatorToLaunchBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChamberElevator);

    m_setChamberElevatorDownBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberElevatorDown);
    m_setChamberElevatorDownBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChamberElevator);

    /* Chassis Buttons */
    //m_gearShiftBtn.whenPressed(RobotContainer.m_inlineCommands.m_shiftHighGear);
    //m_gearShiftBtn.whenReleased(RobotContainer.m_inlineCommands.m_shiftLowGear);

    m_setHeadingBtn.whenPressed(RobotContainer.m_inlineCommands.m_setHeading);

    m_toggleLimelightBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleLimelight);

    /* Climber Buttons */
    // m_extendClimberLeftBtn.whenPressed(RobotContainer.m_inlineCommands.m_extendClimberLeft);
    // m_extendClimberLeftBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopClimberLeft);

    // m_retractClimberLeftBtn.whenPressed(RobotContainer.m_inlineCommands.m_retractClimberLeft);
    // m_retractClimberLeftBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopClimberLeft);

    // m_extendClimberRightBtn.whenPressed(RobotContainer.m_inlineCommands.m_extendClimberRight);
    // m_extendClimberRightBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopClimberRight);

    // m_retractClimberRightBtn.whenPressed(RobotContainer.m_inlineCommands.m_retractClimberRight);
    // m_retractClimberRightBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopClimberRight);

    /* Control Panel Buttons */
    // m_setControlPanelSpinnerPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setControlPanelSpinnerPower);
    // m_setControlPanelSpinnerPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopControlPanelSpinner);
    // m_setControlPanelSpinnerRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setControlPanelSpinnerRPM);
    // m_setControlPanelSpinnerRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopControlPanelSpinner);

    
    /* Defense Mode */
    // m_toggleDefenseModeBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleDefenseMode);

    /* Intake Buttons */
    // m_toggleIntakePistonsBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleIntakePistons);

    m_setIntakeWheelRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setIntakeWheelsRPM);
    m_setIntakeWheelRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopIntakeWheels);

    /* Launcher Buttons */
    m_jogLauncherAngleUpBtn.whenPressed(RobotContainer.m_inlineCommands.m_jogLauncherAngleUp);
    m_jogLauncherAngleUpBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLauncherAngle);

    m_jogLauncherAngleDownBtn.whenPressed(RobotContainer.m_inlineCommands.m_jogLauncherAngleDown);
    m_jogLauncherAngleDownBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLauncherAngle);  

    m_setLauncherVisionBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherVision);
    m_setLauncherVisionBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);

    m_setLauncherPresetBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherPreset);
    m_setLauncherPresetBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);

    m_setLauncherPresetGreenBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherPreset);
    m_setLauncherPresetGreenBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);

    m_setLauncherPresetYellowBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherPreset);
    m_setLauncherPresetYellowBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);

    m_setLauncherPresetBlueBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherPreset);
    m_setLauncherPresetBlueBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);

    m_setLauncherPresetRedBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherPreset);
    m_setLauncherPresetRedBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);

    /*LED buttons */
    m_LEDsChaseInwardBtn.whenPressed(RobotContainer.m_inlineCommands.m_chaseInwards);
    m_LEDsChaseInwardBtn.whenReleased(RobotContainer.m_inlineCommands.m_neutral);

    m_LEDsChaseOutwardBtn.whenPressed(RobotContainer.m_inlineCommands.m_chaseOutwards);
    m_LEDsChaseOutwardBtn.whenReleased(RobotContainer.m_inlineCommands.m_neutral);

    m_LEDsRainbowBtn.whenPressed(RobotContainer.m_inlineCommands.m_rainbow);
    m_LEDsRainbowBtn.whenReleased(RobotContainer.m_inlineCommands.m_neutral);
  }
  
  //BRUH JOVI MOMENT <-- Excuse me, what is this? -Jovi, you know. -Joey, at least indent your comment properly. -Jovi, 
  //Stop me. -Joey
}