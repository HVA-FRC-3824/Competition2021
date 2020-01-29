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

  private static JoystickButton m_toggleIntakePistonsBtn;
  private static JoystickButton m_setIntakeWheelPowerBtn;
  private static JoystickButton m_setIntakeWheelRPMBtn;

  private static JoystickButton m_setLauncherWheelsPowerBtn;
  private static JoystickButton m_setLauncherWheelsRPMBtn;
  private static JoystickButton m_stopLauncherWheelsBtn;

  private static JoystickButton m_jogLauncherAngleUpBtn;
  private static JoystickButton m_jogLauncherAngleDownBtn;

  private static JoystickButton m_setLauncherForInitiationLineBtn;
  private static JoystickButton m_setLauncherForCloseTrenchBtn;
  private static JoystickButton m_setLauncherForFarTrenchBtn;

  private static JoystickButton m_setLauncherFeederPowerBtn;
  private static JoystickButton m_setLauncherFeederRPMBtn;

  private static JoystickButton m_setChamberElevatorPowerBtn;
  private static JoystickButton m_setChamberElevatorRPMBtn;

  private static JoystickButton m_jogClimberReelPositionUpBtn;
  private static JoystickButton m_jogClimberReelPositionDownBtn;

  private static JoystickButton m_setControlPanelSpinnerPowerBtn;
  private static JoystickButton m_setControlPanelSpinnerRPMBtn;

  public OI() 
  {
    /**
     * Instantiate declared joysticks and buttons here.
     */

    /* Driver Joystick */
    m_driverJoystick                  = new Joystick(Constants.DRIVER_JOYSTICK_PORT);

    m_gearShiftBtn                    = new JoystickButton(m_driverJoystick, Constants.DRIVER_GEAR_SHIFT_BTN_ID);

    /* Operator Joystick */
    m_operatorJoystick                = new Joystick(Constants.OPERATOR_JOYSTICK_PORT);

    m_toggleIntakePistonsBtn          = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_TOGGLE_INTAKE_BTN_ID);
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

    m_setChamberElevatorPowerBtn      = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_ELEVATOR_RPM_BTN_ID);
    m_setChamberElevatorRPMBtn        = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_ELEVATOR_POWER_BTN_ID);

    m_jogClimberReelPositionUpBtn     = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_JOG_REEL_POSITION_UP_BTN_ID);
    m_jogClimberReelPositionDownBtn   = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CLIMBER_JOG_REEL_POSITION_DOWN_BTN_ID);

    m_setControlPanelSpinnerPowerBtn  = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CONTROL_PANEL_SPINNER_POWER_BTN_ID);
    m_setControlPanelSpinnerRPMBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CONTROL_PANEL_SPINNER_RPM_BTN_ID);

    /**
     * Bind commands to buttons here.
     */

    /* Chassis Buttons */
    m_gearShiftBtn.whenPressed(Robot.m_inlineCommands.m_shiftHighGear);
    m_gearShiftBtn.whenReleased(Robot.m_inlineCommands.m_shiftLowGear);

    /* Intake Buttons */
    m_toggleIntakePistonsBtn.whenPressed(Robot.m_inlineCommands.m_toggleIntakePistons);

    m_setIntakeWheelPowerBtn.whenPressed(Robot.m_inlineCommands.m_setIntakeWheelsPower);
    m_setIntakeWheelPowerBtn.whenReleased(Robot.m_inlineCommands.m_stopIntakeWheels);

    m_setIntakeWheelRPMBtn.whenPressed(Robot.m_inlineCommands.m_setIntakeWheelsRPM);
    m_setIntakeWheelRPMBtn.whenReleased(Robot.m_inlineCommands.m_stopIntakeWheels);

    /* Launcher Buttons */
    m_setLauncherWheelsPowerBtn.whenPressed(Robot.m_inlineCommands.m_setLauncherWheelsPower);
    m_setLauncherWheelsRPMBtn.whenPressed(Robot.m_inlineCommands.m_setLauncherWheelsRPM);
    m_stopLauncherWheelsBtn.whenPressed(Robot.m_inlineCommands.m_stopLauncherWheels);

    m_jogLauncherAngleUpBtn.whenPressed(Robot.m_inlineCommands.m_jogLauncherAngleUp);
    m_jogLauncherAngleDownBtn.whenPressed(Robot.m_inlineCommands.m_jogLauncherAngleDown);

    m_setLauncherForInitiationLineBtn.whenPressed(Robot.m_inlineCommands.m_setLauncherForInitiationLine);
    m_setLauncherForCloseTrenchBtn.whenPressed(Robot.m_inlineCommands.m_setLauncherForCloseTrench);
    m_setLauncherForFarTrenchBtn.whenPressed(Robot.m_inlineCommands.m_setLauncherForFarTrench);

    m_setLauncherFeederPowerBtn.whenPressed(Robot.m_inlineCommands.m_setLauncherFeederPower);
    m_setLauncherFeederPowerBtn.whenReleased(Robot.m_inlineCommands.m_stopLauncherFeeder);
    m_setLauncherFeederRPMBtn.whenPressed(Robot.m_inlineCommands.m_setLauncherFeederRPM);
    m_setLauncherFeederRPMBtn.whenReleased(Robot.m_inlineCommands.m_stopLauncherFeeder);

    /* Chamber Buttons */
    m_setChamberElevatorPowerBtn.whenPressed(Robot.m_inlineCommands.m_setChamberElevatorPower);
    m_setChamberElevatorPowerBtn.whenReleased(Robot.m_inlineCommands.m_stopChamberElevator);
    m_setChamberElevatorRPMBtn.whenPressed(Robot.m_inlineCommands.m_setChamberElevatorRPM);
    m_setChamberElevatorRPMBtn.whenReleased(Robot.m_inlineCommands.m_stopChamberElevator);

    /* Climber Buttons */
    m_jogClimberReelPositionUpBtn.whenPressed(Robot.m_inlineCommands.m_jogLauncherAngleUp);
    m_jogClimberReelPositionDownBtn.whenPressed(Robot.m_inlineCommands.m_jogLauncherAngleDown);

    /* Control Panel Buttons */
    m_setControlPanelSpinnerPowerBtn.whenPressed(Robot.m_inlineCommands.m_setControlPanelSpinnerPower);
    m_setControlPanelSpinnerPowerBtn.whenReleased(Robot.m_inlineCommands.m_stopControlPanelSpinner);
    m_setControlPanelSpinnerPowerBtn.whenPressed(Robot.m_inlineCommands.m_setControlPanelSpinnerRPM);
    m_setControlPanelSpinnerPowerBtn.whenReleased(Robot.m_inlineCommands.m_stopControlPanelSpinner);
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
  
  //BRUH JOVI MOMENT <-- Excuse me, what is this? -Jovi
}