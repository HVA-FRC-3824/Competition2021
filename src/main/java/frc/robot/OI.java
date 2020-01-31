package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI 
{
  /**
   * Declare all joysticks and buttons here.
   */

  /* Driver Joystick */
  private static Joystick       m_driverJoystick;

  private static JoystickButton m_gearShiftBtn;

  /* Operator Joystick */
  private static Joystick       m_operatorJoystick;
  
  private static JoystickButton m_setChamberElevatorPowerBtn;
  private static JoystickButton m_setChamberElevatorRPMBtn;

  private static JoystickButton m_jogClimberReelPositionUpBtn;
  private static JoystickButton m_jogClimberReelPositionDownBtn;

  // private static JoystickButton m_toggleIntakePistonsBtn;
  private static JoystickButton m_setIntakeWheelPowerBtn;
  private static JoystickButton m_setIntakeWheelRPMBtn;

  private static JoystickButton m_jogLauncherAngleUpBtn;
  private static JoystickButton m_jogLauncherAngleDownBtn;

  private static JoystickButton m_setLauncherWheelsPowerBtn;
  private static JoystickButton m_setLauncherWheelsRPMBtn;
  private static JoystickButton m_stopLauncherWheelsBtn;

  private static JoystickButton m_setLauncherForInitiationLineBtn;
  private static JoystickButton m_setLauncherForCloseTrenchBtn;
  private static JoystickButton m_setLauncherForFarTrenchBtn;

  private static JoystickButton m_setLauncherFeederPowerBtn;
  private static JoystickButton m_setLauncherFeederRPMBtn;

  private static JoystickButton m_setPIDSetpointBtn;

  private static JoystickButton m_setControlPanelSpinnerPowerBtn;
  private static JoystickButton m_setControlPanelSpinnerRPMBtn;
  
  private static JoystickButton m_jogClimberZiplineLeftBtn;
  private static JoystickButton m_jogClimberZiplineRightBtn;

  private static JoystickButton m_setClimberZiplinePowerBtn;
  

  public OI() 
  {
    /**
     * Instantiate the declared joysticks and joystick buttons here.
     */

    /* Driver Joystick */
    m_driverJoystick                  = new Joystick(Constants.DRIVER_JOYSTICK_PORT);

    m_gearShiftBtn                    = new JoystickButton(m_driverJoystick, Constants.DRIVER_GEAR_SHIFT_BTN_ID);

    /* Operator Joystick */
    m_operatorJoystick                = new Joystick(Constants.OPERATOR_JOYSTICK_PORT);

    // m_toggleIntakePistonsBtn          = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_TOGGLE_INTAKE_BTN_ID);
    m_setIntakeWheelPowerBtn          = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_INTAKE_WHEEL_POWER_BTN_ID);
    m_setIntakeWheelRPMBtn            = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_INTAKE_WHEEL_RPM_BTN_ID);

    m_setLauncherWheelsPowerBtn       = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_WHEELS_POWER_BTN_ID);
    m_setLauncherWheelsRPMBtn         = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_WHEELS_RPM_BTN_ID);
    m_stopLauncherWheelsBtn           = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_WHEELS_STOP_BTN_ID);

    m_jogLauncherAngleUpBtn           = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_JOG_ANGLE_UP_BTN_ID);
    m_jogLauncherAngleDownBtn         = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_JOG_ANGLE_DOWN_BTN_ID);

    m_setLauncherForInitiationLineBtn = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_INIT_LINE_BTN_ID);
    m_setLauncherForCloseTrenchBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_CLOSE_TRENCH_BTN_ID);
    m_setLauncherForFarTrenchBtn      = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_FAR_TRENCH_BTN_ID);

    m_setLauncherFeederPowerBtn       = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_FEEDER_POWER_BTN_ID);
    m_setLauncherFeederRPMBtn         = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_FEEDER_RPM_BTN_ID);
    
    m_setPIDSetpointBtn               = new JoystickButton(m_operatorJoystick, 2);

    m_setChamberElevatorPowerBtn      = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_ELEVATOR_RPM_BTN_ID);
    m_setChamberElevatorRPMBtn        = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_ELEVATOR_POWER_BTN_ID);

    m_jogClimberReelPositionUpBtn     = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_JOG_REEL_POSITION_UP_BTN_ID);
    m_jogClimberReelPositionDownBtn   = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_JOG_REEL_POSITION_DOWN_BTN_ID);

    m_jogClimberZiplineLeftBtn        = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_JOG_ZIPLINE_LEFT_BTN_ID);
    m_jogClimberZiplineRightBtn       = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_JOG_ZIPLINE_RIGHT_BTN_ID);

    m_setClimberZiplinePowerBtn       = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_POWER_BTN_ID);
    

    m_setControlPanelSpinnerPowerBtn  = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CONTROL_PANEL_SPINNER_POWER_BTN_ID);
    m_setControlPanelSpinnerRPMBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CONTROL_PANEL_SPINNER_RPM_BTN_ID);

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
    /* Chassis Buttons */
    m_gearShiftBtn.whenPressed(RobotContainer.m_inlineCommands.m_shiftHighGear);
    m_gearShiftBtn.whenReleased(RobotContainer.m_inlineCommands.m_shiftLowGear);

    /* Intake Buttons */
    // m_toggleIntakePistonsBtn.whenPressed(RobotContainer.m_inlineCommands.m_toggleIntakePistons);

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

    m_setLauncherFeederPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherFeederPower);
    m_setLauncherFeederPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLauncherFeeder);
    m_setLauncherFeederRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherFeederRPM);
    m_setLauncherFeederRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLauncherFeeder);

    m_setPIDSetpointBtn.whenPressed(RobotContainer.m_inlineCommands.m_setPIDPractice);
    m_setPIDSetpointBtn.whenReleased(RobotContainer.m_inlineCommands.m_setPIDPracticeZero);

    /* Chamber Buttons */
    m_setChamberElevatorPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberElevatorPower);
    m_setChamberElevatorPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChamberElevator);
    m_setChamberElevatorRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberElevatorRPM);
    m_setChamberElevatorRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopChamberElevator);

    /* Climber Buttons */
    m_jogClimberReelPositionUpBtn.whenPressed(RobotContainer.m_inlineCommands.m_jogLauncherAngleUp);
    m_jogClimberReelPositionDownBtn.whenPressed(RobotContainer.m_inlineCommands.m_jogLauncherAngleDown);
    m_jogClimberZiplineLeftBtn.whenPressed(RobotContainer.m_inlineCommands.m_jogClimberZiplinePositionLeft);
    m_jogClimberZiplineRightBtn.whenPressed(RobotContainer.m_inlineCommands.m_jogClimberZiplinePositionRight);

    m_setClimberZiplinePowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setClimberZiplinePower);
    m_setClimberZiplinePowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopClimberZipline);


    /* Control Panel Buttons */
    m_setControlPanelSpinnerPowerBtn.whenPressed(RobotContainer.m_inlineCommands.m_setControlPanelSpinnerPower);
    m_setControlPanelSpinnerPowerBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopControlPanelSpinner);
    m_setControlPanelSpinnerRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setControlPanelSpinnerRPM);
    m_setControlPanelSpinnerRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopControlPanelSpinner);
  }
  
  //BRUH JOVI MOMENT <-- Excuse me, what is this? -Jovi, you know -Joey
}