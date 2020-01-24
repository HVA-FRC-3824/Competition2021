package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

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
  private static Joystick m_driverJoystick;
  private static Joystick m_operatorJoystick;

  public OI() 
  {
    /**
     * Instantiate declared joysticks and buttons here.
     */
    m_driverJoystick = new Joystick(0);
    m_operatorJoystick = new Joystick(1);

    /**
     * Bind commands to buttons here.
     */
  }

  /**
   * Allows use of driverJoystick object outside of OI class.
   * @return access to driverJoystick values/attributes.
   */
  public static Joystick getDriverJoystick() 
  {
    return m_driverJoystick;
  }

  public static Joystick getOperatorController() 
  {
    return m_operatorJoystick;
  }
  //BRUH JOVI MOMENT <-- Excuse me, what is this? -Jovi
}