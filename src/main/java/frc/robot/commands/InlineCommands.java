package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  /* Chamber Inline Command Declarations */
  public final Command m_setChamberElevatorPower;
  public final Command m_stopChamberElevator;

  public final Command m_setChamberBasePower;
  public final Command m_setChamberBaseRPM;
  public final Command m_stopChamberBase;

  /* Chassis Inline Command Declarations */
  public final Command m_driveWithJoystick;

  public final Command m_shiftHighGear;
  public final Command m_shiftLowGear;

  public final Command m_toggleLimelight;
  
  public final Command m_chassisTurnToTarget;
  public final Command m_chassisAutoTurnToTarget;
  public final Command m_stopChassisTurnToTarget;
  
  /* Climber Inline Command Declarations */
  public final Command m_extendClimberPower;
  public final Command m_retractClimberPower;

  public final Command m_extendClimberPosition;
  public final Command m_retractClimberPosition;

  public final Command m_stopClimber;

  public final Command m_toggleClimberPTO;

  public final Command m_toggleClimberLockRatchets;

  /* Control Panel Command Declarations */
  public final Command m_setControlPanelSpinnerPower;
  public final Command m_setControlPanelSpinnerRPM;
  public final Command m_stopControlPanelSpinner;
  
  /* Intake Inline Command Declarations */
  public final Command m_toggleIntakePistons;

  public final Command m_setIntakeWheelsPower;
  public final Command m_setIntakeWheelsRPM;
  public final Command m_stopIntakeWheels;

  /* Launcher Inline Command Declarations */
  public final Command m_setLauncherTopWheelPower;
  public final Command m_setLauncherBottomWheelPower;
  public final ParallelCommandGroup m_setLauncherWheelsPower;

  public final Command m_setLauncherTopWheelRPM;
  public final Command m_setLauncherBottomWheelRPM;
  public final ParallelCommandGroup m_setLauncherWheelsRPM;

  public final Command m_stopLauncherWheels;

  public final Command m_jogLauncherAngleUp;
  public final Command m_jogLauncherAngleDown;

  public final Command m_setLauncherForInitiationLine;
  public final Command m_setLauncherForCloseTrench;
  public final Command m_setLauncherForFarTrench;
  
  public InlineCommands()
  {
    /**
     * Instantiate inline commands here.
     */

    /* Chamber Inline Command Instantiations */
    m_setChamberElevatorPower =
      new InstantCommand(() -> RobotContainer.m_chamber.setElevatorPower(Constants.CHAMBER_ELEVATOR_POWER), RobotContainer.m_chamber);
    m_stopChamberElevator =
      new InstantCommand(() -> RobotContainer.m_chamber.setElevatorPower(0.0), RobotContainer.m_chamber);

    m_setChamberBasePower =
      new InstantCommand(() -> RobotContainer.m_chamber.setBasePower(Constants.CHAMBER_BASE_POWER));
    m_setChamberBaseRPM =
      new InstantCommand(() -> RobotContainer.m_chamber.setBaseRPM(Constants.CHAMBER_BASE_RPM));
    m_stopChamberBase =
      new InstantCommand(() -> RobotContainer.m_chamber.setBasePower(0.0));

    /* Chassis Inline Command Instantiations */
    m_driveWithJoystick =
      new RunCommand(() -> RobotContainer.m_chassis.teleopDrive(RobotContainer.m_OI.getDriverJoystick().getY(), 
                    RobotContainer.m_OI.getDriverJoystick().getTwist()), RobotContainer.m_chassis);
    m_shiftHighGear =
      new InstantCommand(() -> RobotContainer.m_chassis.shiftHighGear());
    m_shiftLowGear =
      new InstantCommand(() -> RobotContainer.m_chassis.shiftLowGear());

    m_toggleLimelight =
      new InstantCommand(() -> RobotContainer.m_limelight.toggleMode());

    // TODO: Change this command name to something better and relocate since controlling both the chassis turn and launcher aim.
    m_chassisTurnToTarget =
      new ChassisTurnToTarget().alongWith(new LauncherAimForTarget()).andThen(new InstantCommand(() -> RobotContainer.m_limelight.setModeDriver()), new InstantCommand(() -> RobotContainer.m_launcher.setPivotPower(0.0)));
    m_chassisAutoTurnToTarget =
      new ChassisTurnToTarget();
    m_stopChassisTurnToTarget =
      new InstantCommand(() -> this.m_chassisTurnToTarget.cancel());

    /* Climber Inline Command Instantiations */
    m_extendClimberPower =
      new InstantCommand(() -> RobotContainer.m_climber.setReelsPower(Constants.CLIMBER_REEL_POWER));
    m_retractClimberPower =
      new InstantCommand(() -> RobotContainer.m_climber.setReelsPower(-Constants.CLIMBER_REEL_POWER));

    m_extendClimberPosition =
      new InstantCommand(() -> RobotContainer.m_climber.setReelsPosition(Constants.CLIMBER_REEL_MAX_POSITION));
    m_retractClimberPosition =
      new InstantCommand(() -> RobotContainer.m_climber.setReelsPosition(Constants.CLIMBER_REEL_MIN_POSITION));

    m_stopClimber =
      new InstantCommand(() -> RobotContainer.m_climber.setReelsPower(0.0));

    m_toggleClimberPTO =
      new InstantCommand(() -> RobotContainer.m_climber.togglePTO());

    m_toggleClimberLockRatchets =
      new InstantCommand(() -> RobotContainer.m_climber.toggleLockRatchets());

    /* Control Panel Command Instantiations */
    m_setControlPanelSpinnerPower =
      new InstantCommand(() -> RobotContainer.m_controlPanel.setPanelSpinnerPower(Constants.CONTROL_PANEL_SPINNER_POWER));
    m_setControlPanelSpinnerRPM = 
      new InstantCommand(() -> RobotContainer.m_controlPanel.setPanelSpinnerRPM(Constants.CONTROL_PANEL_SPINNER_RPM));
    m_stopControlPanelSpinner =
      new InstantCommand(() -> RobotContainer.m_controlPanel.setPanelSpinnerPower(0.0));

    /* Intake Inline Command Instantiations */ 
    m_toggleIntakePistons =
      new InstantCommand(() -> RobotContainer.m_intake.toggleExtender());

    m_setIntakeWheelsPower =
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(Constants.INTAKE_WHEEL_POWER), RobotContainer.m_intake);
    m_setIntakeWheelsRPM = 
      new InstantCommand(() -> RobotContainer.m_intake.setWheelRPM(Constants.INTAKE_WHEEL_RPM));
    m_stopIntakeWheels = 
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.0));
      
    /* Launcher Inline Command Instantiations */
    m_setLauncherTopWheelPower =
      new RunCommand(() -> RobotContainer.m_launcher.setTopWheelPower(RobotContainer.m_OI.getOperatorController().
                    getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID)));
    m_setLauncherBottomWheelPower =
      new RunCommand(() -> RobotContainer.m_launcher.setBottomWheelPower(RobotContainer.m_OI.getOperatorController().
                    getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID)));
    m_setLauncherWheelsPower = new ParallelCommandGroup(m_setLauncherTopWheelPower, m_setLauncherBottomWheelPower);
    m_setLauncherWheelsPower.addRequirements(RobotContainer.m_launcher);

    m_setLauncherTopWheelRPM =
      new RunCommand(() -> RobotContainer.m_launcher.setTopWheelRPM((int)(RobotContainer.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherBottomWheelRPM =
      new RunCommand(() -> RobotContainer.m_launcher.setBottomWheelRPM((int)(RobotContainer.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherWheelsRPM = new ParallelCommandGroup(m_setLauncherTopWheelRPM, m_setLauncherBottomWheelRPM);
    m_setLauncherWheelsRPM.addRequirements(RobotContainer.m_launcher);

    m_stopLauncherWheels =
      new InstantCommand(() -> RobotContainer.m_launcher.stopWheels(), 
                         RobotContainer.m_launcher);

    m_jogLauncherAngleUp =
      new InstantCommand(() -> RobotContainer.m_launcher.setAngle(RobotContainer.m_launcher.getCurrentDesiredAngle() 
                        + Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    m_jogLauncherAngleDown =
      new InstantCommand(() -> RobotContainer.m_launcher.setAngle(RobotContainer.m_launcher.getCurrentDesiredAngle() 
                        - Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    
    m_setLauncherForInitiationLine =
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_INITIATION_LINE_TOP_RPM, 
                        Constants.LAUNCHER_INITIATION_LINE_BOTTOM_RPM, Constants.LAUNCHER_INITIATION_LINE_ANGLE), 
                        RobotContainer.m_launcher);
    m_setLauncherForCloseTrench =
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_CLOSE_TRENCH_TOP_RPM, 
                        Constants.LAUNCHER_CLOSE_TRENCH_BOTTOM_RPM, Constants.LAUNCHER_CLOSE_TRENCH_ANGLE), 
                        RobotContainer.m_launcher);
    m_setLauncherForFarTrench =
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_FAR_TRENCH_TOP_RPM, 
                        Constants.LAUNCHER_FAR_TRENCH_BOTTOM_RPM, Constants.LAUNCHER_FAR_TRENCH_ANGLE),
                        RobotContainer.m_launcher);
  }
}