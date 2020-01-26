package frc.robot.commands;

import frc.robot.Constants;
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
  public final Command m_setLauncherTopWheelRPM;
  public final Command m_setLauncherBottomWheelRPM;
  public final Command m_setLauncherWheelsRPM;
  public final Command m_stopLauncherWheels;

  /* Intake Inline Command Declarations */
  public final Command m_toggleIntake;
  public final Command m_setIntakeWheelsPower;
  public final Command m_setIntakeWheelsRPM;
  public final Command m_stopIntakeWheels;

  public InlineCommands()
  {
    /**
     * Instantiate inline commands here.
     */

    /* Chassis Inline Command Instantiations */
    m_driveWithJoystick =
      new RunCommand(() -> Robot.m_chassis.drive(Robot.m_OI.getDriverJoystick().getY(), Robot.m_OI.getDriverJoystick().getTwist()), Robot.m_chassis);
    
    m_shiftHighGear = 
      new InstantCommand(() -> Robot.m_chassis.shiftHighGear());
    m_shiftLowGear =
      new InstantCommand(() -> Robot.m_chassis.shiftLowGear());

    /* Launcher Inline Command Instantiations */
    m_setLauncherTopWheelPower =
      new InstantCommand(() -> Robot.m_launcher.setTopWheelPower(Robot.m_OI.getOperatorController().getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID)));
    m_setLauncherBottomWheelPower =
      new InstantCommand(() -> Robot.m_launcher.setBottomWheelPower(Robot.m_OI.getOperatorController().getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID)));
    m_setLauncherWheelsPower =
      new RunCommand(() -> m_setLauncherTopWheelPower.alongWith(m_setLauncherBottomWheelPower), Robot.m_launcher);

    m_setLauncherTopWheelRPM =
      new InstantCommand(() -> Robot.m_launcher.setTopWheelRPM((int)(Robot.m_OI.getOperatorController().getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherBottomWheelRPM =
      new InstantCommand(() -> Robot.m_launcher.setBottomWheelRPM((int)(Robot.m_OI.getOperatorController().getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherWheelsRPM =
      new RunCommand(() -> m_setLauncherTopWheelRPM.alongWith(m_setLauncherBottomWheelRPM), Robot.m_launcher);

    m_stopLauncherWheels =
      new InstantCommand(() -> Robot.m_launcher.stopWheels(), Robot.m_launcher);

    /* Intake Inline Command Instantiations */ 
    m_toggleIntake =
      new InstantCommand(() -> Robot.m_intake.toggleExtender());

    m_setIntakeWheelsPower = 
      new InstantCommand(() -> Robot.m_intake.setWheelPower(Constants.INTAKE_WHEEL_POWER));
    m_setIntakeWheelsRPM = 
      new InstantCommand(() -> Robot.m_intake.setWheelRPM(Constants.INTAKE_WHEEL_RPM));
    m_stopIntakeWheels = 
      new InstantCommand(() -> Robot.m_intake.setWheelPower(0.0));
  }
}