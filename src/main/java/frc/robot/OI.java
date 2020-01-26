package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Describes all the operator interface controls for the robot
 * 
 * Launcher Controls
 * -------------------------------------------------------------------------
 * Button: set launcher angle and wheel speed at initiation line
 * Button: set launcher angle and wheel speed at control panel
 * Slider: manual set wheel speed
 * Button: increase launcher angle
 * Button: decrease launcher angle
 * Button: feeds ball to launcher
 * Button: auto aim and launch
 * 
 * Intake Controls
 * -------------------------------------------------------------------------
 * Button: extend/retract intake and start/stop intake wheels
 * 
 * Chamber Controls
 * -------------------------------------------------------------------------
 * Button: enable conveyor while held
 * Button: auto feed 1 cell
 * 
 * Chassis Controls
 * -------------------------------------------------------------------------
 * Joystick: drive robot with speed and direction
 * Button: shift gears
 * Button: auto align with vision
 * 
 * Climber Controls
 * -------------------------------------------------------------------------
 * Button: extend hooks
 * Button: pull robot up
 * Button: auto pull up and balance
 * Button: manual balance left
 * Button: manual balance right
 * Button: lock ratchets
 * 
 * Control Panel Controls
 * -------------------------------------------------------------------------
 * Button: rotate wheel 4 times
 * Button: rotate to color
 * Button: override rotation when held
 */

public class OI 
{
  /**
   * Declare all joysticks and buttons here.
   */

  /* Driver Joystick */
  private static Joystick m_driverJoystick;

  private static JoystickButton m_gearShiftBtn;

  /* Operator Joystick */
  private static Joystick m_operatorJoystick;

  private static JoystickButton m_toggleIntakePistonsBtn;
  private static JoystickButton m_setIntakeWheelPowerBtn;
  private static JoystickButton m_setIntakeWheelRPMBtn;

  private static JoystickButton m_setLauncherWheelsPowerBtn;
  private static JoystickButton m_setLauncherWheelsRPMBtn;
  private static JoystickButton m_stopLauncherWheelsBtn;

  public OI() 
  {
    /**
     * Instantiate declared joysticks and buttons here.
     */

    /* Driver Joystick */
    m_driverJoystick = new Joystick(Constants.DRIVER_JOYSTICK_PORT);

    m_gearShiftBtn = new JoystickButton(m_driverJoystick, Constants.DRIVER_GEAR_SHIFT_BTN_ID);

    /* Operator Joystick */
    m_operatorJoystick = new Joystick(Constants.OPERATOR_JOYSTICK_PORT);

    m_toggleIntakePistonsBtn = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_TOGGLE_INTAKE_BTN_ID);
    m_setIntakeWheelPowerBtn = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_INTAKE_WHEEL_POWER_BTN_ID);
    m_setIntakeWheelRPMBtn = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_INTAKE_WHEEL_RPM_BTN_ID);

    m_setLauncherWheelsPowerBtn = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_WHEELS_POWER_BTN_ID);
    m_setLauncherWheelsRPMBtn = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_WHEELS_RPM_BTN_ID);
    m_stopLauncherWheelsBtn = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_WHEELS_STOP_BTN_ID);

    /**
     * Bind commands to buttons here.
     */

    /* Chassis Buttons */
    m_gearShiftBtn.whenPressed(Robot.m_inlineCommands.m_shiftHighGear);
    m_gearShiftBtn.whenReleased(Robot.m_inlineCommands.m_shiftLowGear);

    /* Intake Buttons */
    m_toggleIntakePistonsBtn.whenPressed(Robot.m_inlineCommands.m_toggleIntake);

    m_setIntakeWheelPowerBtn.whenPressed(Robot.m_inlineCommands.m_setIntakeWheelsPower);
    m_setIntakeWheelPowerBtn.whenReleased(Robot.m_inlineCommands.m_stopIntakeWheels);

    m_setIntakeWheelRPMBtn.whenPressed(Robot.m_inlineCommands.m_setIntakeWheelsRPM);
    m_setIntakeWheelRPMBtn.whenReleased(Robot.m_inlineCommands.m_stopIntakeWheels);

    /* Launcher Buttons */
    m_setLauncherWheelsPowerBtn.whenPressed(Robot.m_inlineCommands.m_setLauncherWheelsPower);
    m_setLauncherWheelsRPMBtn.whenPressed(Robot.m_inlineCommands.m_setLauncherWheelsRPM);
    m_stopLauncherWheelsBtn.whenPressed(Robot.m_inlineCommands.m_stopLauncherWheels);
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