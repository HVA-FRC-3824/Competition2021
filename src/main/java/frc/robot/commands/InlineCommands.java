package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * Inline commands are a convenient way of creating commands without the need
 * for an entirely new CommandBase file. Inline commands will be used for
 * "simple" actions (Example: extending a piston). For "complex" actions
 * (Example: ten-ball autonomous command sequence), create a separate
 * CommandBase/CommandGroup file. Inline commands can also be utilized in other
 * files (other commands or OI.java for binding commands to buttons).
 */
public class InlineCommands {
  /**
   * Declare all inline commands here.
   */

  /* Chassis Inline Command Declarations */
  public final Command m_driveWithJoystick;
  public final Command m_shiftHighGear;
  public final Command m_shiftLowGear;

  /* Launcher Inline Command Declarations */
  public final Command m_setLauncherTopWheelPower;
  public final Command m_setLauncherBottomWheelPower;
  public final Command m_setLauncherWheelsPower;
    
  public InlineCommands()
  {
    /**
     * Instantiate inline commands here.
     */

    /* Chassis Inline Command Instantiations */
    m_driveWithJoystick =
      new RunCommand(() -> Robot.m_chassis.drive(Robot.m_OI.getDriverJoystick().getY(), Robot.m_OI.getDriverJoystick().getTwist()), Robot.m_chassis);
    m_shiftHighGear = 
      new InstantCommand(() -> Robot.m_chassis.shiftGear(true));
    m_shiftLowGear = 
      new InstantCommand(() -> Robot.m_chassis.shiftGear(false));    

    /* Launcher Inline Command Instantiations */
    m_setLauncherTopWheelPower =
      new InstantCommand(() -> Robot.m_launcher.setTopWheelPower(Robot.m_OI.getOperatorController().getRawAxis(5)));
    m_setLauncherBottomWheelPower =
      new InstantCommand(() -> Robot.m_launcher.setBottomWheelPower(Robot.m_OI.getOperatorController().getRawAxis(5)));
    m_setLauncherWheelsPower =
      new RunCommand(() -> m_setLauncherTopWheelPower.alongWith(m_setLauncherBottomWheelPower), Robot.m_launcher);

    
  }
}