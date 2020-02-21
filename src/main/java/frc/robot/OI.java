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

  private static JoystickButton m_chassisTurnToTargetBtn;

  /* 
   * Operator Joystick
   */
  private static Joystick       m_operatorJoystick;
  
  /* Chamber */
  private static JoystickButton m_setChamberBasePowerBtn;
  private static JoystickButton m_setChamberBaseRPMBtn;

  private static JoystickButton m_setChamberElevatorPowerBtn;

  /* Climber */
  private static JoystickButton m_extendClimberPowerBtn;
  private static JoystickButton m_retractClimberPowerBtn;

  private static JoystickButton m_extendClimberPositionBtn;
  private static JoystickButton m_retractClimberPositionBtn;

  private static JoystickButton m_toggleClimberPTOBtn;

  private static JoystickButton m_toggleClimberLockRatchetsBtn;
  
  /* Control Panel */
  private static JoystickButton m_setControlPanelSpinnerPowerBtn;
  private static JoystickButton m_setControlPanelSpinnerRPMBtn;
  
  /* Intake */
  private static JoystickButton m_toggleIntakePistonsBtn;
  private static JoystickButton m_setIntakeWheelPowerBtn;
  private static JoystickButton m_setIntakeWheelRPMBtn;

  /* Launcher */
  private static JoystickButton m_setLauncherWheelsPowerBtn;
  private static JoystickButton m_setLauncherWheelsRPMBtn;
  private static JoystickButton m_stopLauncherWheelsBtn;

  private static JoystickButton m_jogLauncherAngleUpBtn;
  private static JoystickButton m_jogLauncherAngleDownBtn;

  private static JoystickButton m_setLauncherForInitiationLineBtn;
  private static JoystickButton m_setLauncherForCloseTrenchBtn;
  private static JoystickButton m_setLauncherForFarTrenchBtn;

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

    m_chassisTurnToTargetBtn          = new JoystickButton(m_driverJoystick, Constants.DRIVER_CHASSIS_TURN_TO_TARGET_BTN_ID);

    /* 
     * Operator Joystick
     */    
    m_operatorJoystick                = new Joystick(Constants.OPERATOR_JOYSTICK_PORT);

    /* Chamber */
    m_setChamberBasePowerBtn          = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_BASE_POWER_BTN_ID);
    m_setChamberBaseRPMBtn            = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_BASE_RPM_BTN_ID);

    m_setChamberElevatorPowerBtn      = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_ELEVATOR_POWER_BTN_ID);

    /* Climber */
    m_extendClimberPowerBtn           = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_EXTEND_POWER_BTN_ID);
    m_retractClimberPowerBtn          = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_RETRACT_POWER_BTN_ID);

    m_extendClimberPositionBtn        = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_EXTEND_POSITION_BTN_ID);
    m_retractClimberPositionBtn       = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_RETRACT_POSITION_BTN_ID);

    m_toggleClimberPTOBtn             = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_TOGGLE_PTO_BTN_ID);

    m_toggleClimberLockRatchetsBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_TOGGLE_LOCK_RATCHETS_BTN_ID);

    /* Control Panel */
    m_setControlPanelSpinnerPowerBtn  = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CONTROL_PANEL_SPINNER_POWER_BTN_ID);
    m_setControlPanelSpinnerRPMBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CONTROL_PANEL_SPINNER_RPM_BTN_ID);

    /* Intake */
    m_toggleIntakePistonsBtn          = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_TOGGLE_INTAKE_BTN_ID);
    m_setIntakeWheelPowerBtn          = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_INTAKE_WHEEL_POWER_BTN_ID);
    m_setIntakeWheelRPMBtn            = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_INTAKE_WHEEL_RPM_BTN_ID);

    /* Launcher */
    m_setLauncherWheelsPowerBtn       = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_WHEELS_POWER_BTN_ID);
    m_setLauncherWheelsRPMBtn         = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_WHEELS_RPM_BTN_ID);
    m_stopLauncherWheelsBtn           = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_WHEELS_STOP_BTN_ID);

    m_jogLauncherAngleUpBtn           = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_JOG_ANGLE_UP_BTN_ID);
    m_jogLauncherAngleDownBtn         = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_JOG_ANGLE_DOWN_BTN_ID);

    m_setLauncherForInitiationLineBtn = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_INIT_LINE_BTN_ID);
    m_setLauncherForCloseTrenchBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_CLOSE_TRENCH_BTN_ID);
    m_setLauncherForFarTrenchBtn      = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_FAR_TRENCH_BTN_ID);
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
    m_setChamberBasePowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberBasePower);
    m_setChamberBasePowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChamberBase);
    m_setChamberBaseRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberBaseRPM);
    m_setChamberBaseRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChamberBase);

    m_setChamberElevatorPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberElevatorPower);
    m_setChamberElevatorPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChamberElevator);

    /* Chassis Buttons */
    m_gearShiftBtn.whenPressed(RobotContainer.m_inlineCommands.m_shiftHighGear);
    m_gearShiftBtn.whenReleased(RobotContainer.m_inlineCommands.m_shiftLowGear);

    m_toggleLimelightBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleLimelight);

    m_chassisTurnToTargetBtn.whenPressed(RobotContainer.m_inlineCommands.m_chassisTurnToTarget);
    m_chassisTurnToTargetBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChassisTurnToTarget);

    /* Climber Buttons */
    m_extendClimberPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_extendClimberPower);
    m_extendClimberPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopClimber);

    m_retractClimberPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_retractClimberPower);
    m_retractClimberPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopClimber);

    m_extendClimberPositionBtn.whenPressed(RobotContainer.m_inlineCommands.m_extendClimberPosition);
    m_retractClimberPositionBtn.whenPressed(RobotContainer.m_inlineCommands.m_retractClimberPosition);

    m_toggleClimberPTOBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleClimberPTO);

    m_toggleClimberLockRatchetsBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleClimberLockRatchets);

    /* Control Panel Buttons */
    m_setControlPanelSpinnerPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setControlPanelSpinnerPower);
    m_setControlPanelSpinnerPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopControlPanelSpinner);
    m_setControlPanelSpinnerRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setControlPanelSpinnerRPM);
    m_setControlPanelSpinnerRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopControlPanelSpinner);

    /* Intake Buttons */
    m_toggleIntakePistonsBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleIntakePistons);

    m_setIntakeWheelPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setIntakeWheelsPower);
    m_setIntakeWheelPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopIntakeWheels);

    m_setIntakeWheelRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setIntakeWheelsRPM);
    m_setIntakeWheelRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopIntakeWheels);

    /* Launcher Buttons */
    m_setLauncherWheelsPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherWheelsPower);
    m_setLauncherWheelsRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherWheelsRPM);
    m_stopLauncherWheelsBtn.whenPressed(RobotContainer.m_inlineCommands.m_stopLauncherWheels);

    m_jogLauncherAngleUpBtn.whenPressed(RobotContainer.m_inlineCommands.m_jogLauncherAngleUp);
    m_jogLauncherAngleDownBtn.whenPressed(RobotContainer.m_inlineCommands.m_jogLauncherAngleDown);

    m_setLauncherForInitiationLineBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherForInitiationLine);
    m_setLauncherForCloseTrenchBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherForCloseTrench);
    m_setLauncherForFarTrenchBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherForFarTrench);

    
  }
  
  //BRUH JOVI MOMENT <-- Excuse me, what is this? -Jovi, you know -Joey, at least indent your comment properly. -Jovi
}