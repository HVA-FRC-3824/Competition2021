package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;

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

  /* Control Panel Command Declarations */
  public final Command m_setControlPanelSpinnerPower;
  public final Command m_setControlPanelSpinnerRPM;
  public final Command m_stopControlPanelSpinner;

  public InlineCommands()
  {
    /**
     * Instantiate inline commands here.
     */

    /* Chassis Inline Command Instantiations */
    m_driveWithJoystick =
      new RunCommand(() -> RobotContainer.m_chassis.drive(RobotContainer.m_OI.getDriverJoystick().getY(), RobotContainer.m_OI.getDriverJoystick().getTwist()),
                     RobotContainer.m_chassis);
    
    m_shiftHighGear =
      new InstantCommand(() -> RobotContainer.m_chassis.shiftHighGear());
    m_shiftLowGear =
      new InstantCommand(() -> RobotContainer.m_chassis.shiftLowGear());

    /* Launcher Inline Command Instantiations */
    m_setLauncherTopWheelPower =
      new InstantCommand(() -> RobotContainer.m_launcher.setTopWheelPower(RobotContainer.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID)));
    m_setLauncherBottomWheelPower =
      new InstantCommand(() -> RobotContainer.m_launcher.setBottomWheelPower(RobotContainer.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID)));
    m_setLauncherWheelsPower =
      new RunCommand(() -> m_setLauncherTopWheelPower.alongWith(m_setLauncherBottomWheelPower), 
                     RobotContainer.m_launcher);

    m_setLauncherTopWheelRPM =
      new InstantCommand(() -> RobotContainer.m_launcher.setTopWheelRPM((int)(RobotContainer.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherBottomWheelRPM =
      new InstantCommand(() -> RobotContainer.m_launcher.setBottomWheelRPM((int)(RobotContainer.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherWheelsRPM =
      new RunCommand(() -> m_setLauncherTopWheelRPM.alongWith(m_setLauncherBottomWheelRPM), 
                     RobotContainer.m_launcher);

    m_stopLauncherWheels =
      new InstantCommand(() -> RobotContainer.m_launcher.stopWheels(), 
                         RobotContainer.m_launcher);

    m_jogLauncherAngleUp =
      new InstantCommand(() -> RobotContainer.m_launcher.setAngle(RobotContainer.m_launcher.getCurrentAngle() + Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    m_jogLauncherAngleDown =
      new InstantCommand(() -> RobotContainer.m_launcher.setAngle(RobotContainer.m_launcher.getCurrentAngle() - Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    
    m_setLauncherForInitiationLine =
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_INITIATION_LINE_TOP_RPM, 
                        Constants.LAUNCHER_INITIATION_LINE_BOTTOM_RPM, Constants.LAUNCHER_INITIATION_LINE_ANGLE), RobotContainer.m_launcher);
    m_setLauncherForCloseTrench =
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_CLOSE_TRENCH_TOP_RPM, 
                        Constants.LAUNCHER_CLOSE_TRENCH_BOTTOM_RPM, Constants.LAUNCHER_CLOSE_TRENCH_ANGLE), RobotContainer.m_launcher);
    m_setLauncherForFarTrench =
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_FAR_TRENCH_TOP_RPM, 
                        Constants.LAUNCHER_FAR_TRENCH_BOTTOM_RPM, Constants.LAUNCHER_FAR_TRENCH_ANGLE), RobotContainer.m_launcher);

    m_setLauncherFeederPower =
      new InstantCommand(() -> RobotContainer.m_launcher.setFeederPower(Constants.LAUNCHER_FEEDER_POWER)).
                        alongWith(RobotContainer.m_inlineCommands.m_setChamberElevatorPower);
    m_setLauncherFeederRPM =
      new InstantCommand(() -> RobotContainer.m_launcher.setFeederRPM(Constants.LAUNCHER_FEEDER_RPM)).
                        alongWith(RobotContainer.m_inlineCommands.m_setChamberElevatorRPM);
    m_stopLauncherFeeder =
      new InstantCommand(() -> RobotContainer.m_launcher.setFeederPower(0.0));

    /* Intake Inline Command Instantiations */ 
    m_toggleIntakePistons =
      new InstantCommand(() -> RobotContainer.m_intake.toggleExtender());

    m_setIntakeWheelsPower =
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(Constants.INTAKE_WHEEL_POWER));
    m_setIntakeWheelsRPM = 
      new InstantCommand(() -> RobotContainer.m_intake.setWheelRPM(Constants.INTAKE_WHEEL_RPM));
    m_stopIntakeWheels = 
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.0));

    /* Chamber Inline Command Instantiations */
    m_setChamberElevatorPower =
      new InstantCommand(() -> RobotContainer.m_chamber.setChamberElevatorPower(Constants.CHAMBER_ELEVATOR_POWER));
    m_setChamberElevatorRPM =
      new InstantCommand(() -> RobotContainer.m_chamber.setChamberElevatorRMP(Constants.CHAMBER_ELEVATOR_RPM));
    m_stopChamberElevator =
      new InstantCommand(() -> RobotContainer.m_chamber.setChamberElevatorPower(0.0));

    /* Control Panel Command Instantiations */
    m_setControlPanelSpinnerPower =
      new InstantCommand(() -> RobotContainer.m_controlPanel.setWheelSpinnerPower(Constants.CONTROL_PANEL_SPINNER_POWER));
    m_setControlPanelSpinnerRPM = 
      new InstantCommand(() -> RobotContainer.m_controlPanel.setWheelSpinnerRPM(Constants.CONTROL_PANEL_SPINNER_RPM));
    m_stopControlPanelSpinner =
      new InstantCommand(() -> RobotContainer.m_controlPanel.setWheelSpinnerPower(0.0));
  }
}