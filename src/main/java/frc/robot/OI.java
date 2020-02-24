package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  /* 
   * Operator Joystick
   */
  private static Joystick       m_operatorJoystick;
  
  /* Chamber */
  private static JoystickButton m_setChamberBaseRPMBtn;

  private static JoystickButton m_setChamberElevatorToLaunchBtn;
  private static JoystickButton m_setChamberElevatorDownBtn;

  /* Climber */
  private static JoystickButton m_extendClimberReelPositionBtn;
  private static JoystickButton m_retractClimberReelPositionBtn;

  private static JoystickButton m_extendClimberLiftPositionBtn;
  private static JoystickButton m_retractClimberLiftPositionBtn;
  
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

    /* 
     * Operator Joystick
     */    
    m_operatorJoystick                = new Joystick(Constants.OPERATOR_JOYSTICK_PORT);

    /* Chamber */
    m_setChamberBaseRPMBtn            = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_BASE_RPM_BTN_ID);

    m_setChamberElevatorToLaunchBtn   = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_ELEVATOR_LAUNCH_BTN_ID);
    m_setChamberElevatorDownBtn       = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_ELEVATOR_DOWN_BTN_ID);

    /* Climber */
    m_extendClimberReelPositionBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_EXTEND_REEL_POSITION_BTN_ID);
    m_retractClimberReelPositionBtn   = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_RETRACT_REEL_POSITION_BTN_ID);

    m_extendClimberLiftPositionBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_EXTEND_LIFT_POSITION_BTN_ID);
    m_retractClimberLiftPositionBtn   = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_RETRACT_LIFT_POSITION_BTN_ID);

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
    m_setChamberBaseRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberBaseRPM);
    m_setChamberBaseRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChamberBase);

    m_setChamberElevatorToLaunchBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberElevatorToLaunch);
    m_setChamberElevatorToLaunchBtn.whenReleased(RobotContainer.m_inlineCommands.m_setChamberElevatorAuto);

    m_setChamberElevatorDownBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberElevatorDown);
    m_setChamberElevatorDownBtn.whenReleased(RobotContainer.m_inlineCommands.m_setChamberElevatorAuto);

    /* Chassis Buttons */
    m_gearShiftBtn.whenPressed(RobotContainer.m_inlineCommands.m_shiftHighGear);
    m_gearShiftBtn.whenReleased(RobotContainer.m_inlineCommands.m_shiftLowGear);

    m_toggleLimelightBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleLimelight);

    /* Climber Buttons */
    m_extendClimberReelPositionBtn.whenPressed(RobotContainer.m_inlineCommands.m_extendClimberReelPosition);
    m_extendClimberReelPositionBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopExtendClimberReelPos);
    m_retractClimberReelPositionBtn.whenPressed(RobotContainer.m_inlineCommands.m_retractClimberReelPosition);
    m_retractClimberReelPositionBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopRetractClimberReelPos);

    m_extendClimberLiftPositionBtn.whenPressed(RobotContainer.m_inlineCommands.m_extendClimberLiftPosition);
    m_extendClimberLiftPositionBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopExtendClimberLiftPos);
    m_retractClimberLiftPositionBtn.whenPressed(RobotContainer.m_inlineCommands.m_retractClimberLiftPosition);
    m_retractClimberLiftPositionBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopRetractClimberLiftPos);

    /* Control Panel Buttons */
    // m_setControlPanelSpinnerPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setControlPanelSpinnerPower);
    // m_setControlPanelSpinnerPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopControlPanelSpinner);
    // m_setControlPanelSpinnerRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setControlPanelSpinnerRPM);
    // m_setControlPanelSpinnerRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopControlPanelSpinner);

    /* Intake Buttons */
    m_toggleIntakePistonsBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleIntakePistons);

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
  }
  
  //BRUH JOVI MOMENT <-- Excuse me, what is this? -Jovi, you know -Joey, at least indent your comment properly. -Jovi
}