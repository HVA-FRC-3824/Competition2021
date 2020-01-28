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

  public final Command m_jogLauncherAngleUp;
  public final Command m_jogLauncherAngleDown;

  public final Command m_setLauncherForInitiationLine;
  public final Command m_setLauncherForCloseTrench;
  public final Command m_setLauncherForFarTrench;

  public final Command m_setLauncherFeederPower;
  public final Command m_setLauncherFeederRPM;
  public final Command m_stopLauncherFeeder;

  /* Intake Inline Command Declarations */
  public final Command m_toggleIntakePistons;

  public final Command m_setIntakeWheelsPower;
  public final Command m_setIntakeWheelsRPM;
  public final Command m_stopIntakeWheels;

  /* Chamber Inline Command Declarations */
  public final Command m_setChamberElevatorPower;
  public final Command m_setChamberElevatorRPM;
  public final Command m_stopChamberElevator;

  public InlineCommands()
  {
    /**
     * Instantiate inline commands here.
     */

    /* Chassis Inline Command Instantiations */
    m_driveWithJoystick =
      new RunCommand(() -> Robot.m_chassis.drive(Robot.m_OI.getDriverJoystick().getY(), Robot.m_OI.getDriverJoystick().getTwist()),
                     Robot.m_chassis);
    
    m_shiftHighGear =
      new InstantCommand(() -> Robot.m_chassis.shiftHighGear());
    m_shiftLowGear =
      new InstantCommand(() -> Robot.m_chassis.shiftLowGear());

    /* Launcher Inline Command Instantiations */
    m_setLauncherTopWheelPower =
      new InstantCommand(() -> Robot.m_launcher.setTopWheelPower(Robot.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID)));
    m_setLauncherBottomWheelPower =
      new InstantCommand(() -> Robot.m_launcher.setBottomWheelPower(Robot.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID)));
    m_setLauncherWheelsPower =
      new RunCommand(() -> m_setLauncherTopWheelPower.alongWith(m_setLauncherBottomWheelPower), 
                     Robot.m_launcher);

    m_setLauncherTopWheelRPM =
      new InstantCommand(() -> Robot.m_launcher.setTopWheelRPM((int)(Robot.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherBottomWheelRPM =
      new InstantCommand(() -> Robot.m_launcher.setBottomWheelRPM((int)(Robot.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherWheelsRPM =
      new RunCommand(() -> m_setLauncherTopWheelRPM.alongWith(m_setLauncherBottomWheelRPM), 
                     Robot.m_launcher);

    m_stopLauncherWheels =
      new InstantCommand(() -> Robot.m_launcher.stopWheels(), 
                         Robot.m_launcher);

    m_jogLauncherAngleUp =
      new InstantCommand(() -> Robot.m_launcher.setAngle(Robot.m_launcher.getCurrentAngle() + Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    m_jogLauncherAngleDown =
      new InstantCommand(() -> Robot.m_launcher.setAngle(Robot.m_launcher.getCurrentAngle() - Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    
    m_setLauncherForInitiationLine =
      new InstantCommand(() -> Robot.m_launcher.setPreset(Constants.LAUNCHER_INITIATION_LINE_TOP_RPM, 
                        Constants.LAUNCHER_INITIATION_LINE_BOTTOM_RPM, Constants.LAUNCHER_INITIATION_LINE_ANGLE),Robot.m_launcher);
    m_setLauncherForCloseTrench =
      new InstantCommand(() -> Robot.m_launcher.setPreset(Constants.LAUNCHER_CLOSE_TRENCH_TOP_RPM, 
                        Constants.LAUNCHER_CLOSE_TRENCH_BOTTOM_RPM, Constants.LAUNCHER_CLOSE_TRENCH_ANGLE),Robot.m_launcher);
    m_setLauncherForFarTrench =
      new InstantCommand(() -> Robot.m_launcher.setPreset(Constants.LAUNCHER_FAR_TRENCH_TOP_RPM, 
                        Constants.LAUNCHER_FAR_TRENCH_BOTTOM_RPM, Constants.LAUNCHER_FAR_TRENCH_ANGLE), Robot.m_launcher);

    m_setLauncherFeederPower =
      new InstantCommand(() -> Robot.m_launcher.setFeederPower(Constants.LAUNCHER_FEEDER_POWER)).
                        alongWith(Robot.m_inlineCommands.m_setChamberElevatorPower);
    m_setLauncherFeederRPM =
      new InstantCommand(() -> Robot.m_launcher.setFeederRPM(Constants.LAUNCHER_FEEDER_RPM)).
                        alongWith(Robot.m_inlineCommands.m_setChamberElevatorRPM);
    m_stopLauncherFeeder =
      new InstantCommand(() -> Robot.m_launcher.setFeederPower(0.0));

    /* Intake Inline Command Instantiations */ 
    m_toggleIntakePistons =
      new InstantCommand(() -> Robot.m_intake.toggleExtender());

    m_setIntakeWheelsPower =
      new InstantCommand(() -> Robot.m_intake.setWheelPower(Constants.INTAKE_WHEEL_POWER));
    m_setIntakeWheelsRPM = 
      new InstantCommand(() -> Robot.m_intake.setWheelRPM(Constants.INTAKE_WHEEL_RPM));
    m_stopIntakeWheels = 
      new InstantCommand(() -> Robot.m_intake.setWheelPower(0.0));

    /* Chamber Inline Command Instantiations */
    m_setChamberElevatorPower =
      new InstantCommand(() -> Robot.m_chamber.setChamberElevatorPower(Constants.CHAMBER_ELEVATOR_POWER));
    m_setChamberElevatorRPM =
      new InstantCommand(() -> Robot.m_chamber.setChamberElevatorRMP(Constants.CHAMBER_ELEVATOR_RPM));
    m_stopChamberElevator =
      new InstantCommand(() -> Robot.m_chamber.setChamberElevatorPower(0.0));
  }
}