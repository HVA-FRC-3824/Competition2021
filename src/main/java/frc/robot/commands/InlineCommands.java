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

  /* Chamber Inline Command Declarations */
  public final Command m_setChamberBaseRPM;
  public final Command m_stopChamberBase;

  public final Command m_setChamberElevatorToLaunch;
  public final Command m_setChamberElevatorDown;
  public final Command m_setChamberElevatorAuto;

  /* Chassis Inline Command Declarations */
  public final Command m_driveWithJoystick;

  public final Command m_shiftHighGear;
  public final Command m_shiftLowGear;

  public final Command m_toggleLimelight;
  
  public final Command m_chassisTurnToTarget;
  public final Command m_chassisAutoTurnToTarget;
  public final Command m_stopChassisTurnToTarget;
  
  /* Climber Inline Command Declarations */
  public final Command m_extendClimberReelPosition;
  public final Command m_stopExtendClimberReelPos;
  public final Command m_retractClimberReelPosition;
  public final Command m_stopRetractClimberReelPos;

  public final Command m_extendClimberLiftPosition;
  public final Command m_stopExtendClimberLiftPos;
  public final Command m_retractClimberLiftPosition;
  public final Command m_stopRetractClimberLiftPos;

  /* Control Panel Command Declarations */
  // public final Command m_setControlPanelSpinnerPower;
  // public final Command m_setControlPanelSpinnerRPM;
  // public final Command m_stopControlPanelSpinner;
  
  /* Intake Inline Command Declarations */
  public final Command m_toggleIntakePistons;

  public final Command m_setIntakeWheelsRPM;
  public final Command m_stopIntakeWheels;

  /* Launcher Inline Command Declarations */
  public final Command m_jogLauncherAngleUp;
  public final Command m_jogLauncherAngleDown;
  public final Command m_stopLauncherAngle;

  public final Command m_setLauncherPreset;
  
  public InlineCommands()
  {
    /**
     * Instantiate inline commands here.
     */

    /* Chamber Inline Command Instantiations */
    m_setChamberBaseRPM =
      new InstantCommand(() -> RobotContainer.m_chamber.setBaseRPM(Constants.CHAMBER_BASE_RPM));
    m_stopChamberBase =
      new InstantCommand(() -> RobotContainer.m_chamber.setBasePower(0.0));

    m_setChamberElevatorToLaunch =
      new RunCommand(() -> RobotContainer.m_chamber.stepChamberDistance(Constants.CHAMBER_BALL_STEP_DIST), RobotContainer.m_chamber);
    m_setChamberElevatorDown =
      new RunCommand(() -> RobotContainer.m_chamber.stepChamberDistance(-Constants.CHAMBER_BALL_STEP_DIST), RobotContainer.m_chamber);
    m_setChamberElevatorAuto =
      new ChamberIndexBalls();

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
    // m_extendClimberReelPosition =
    //   new RunCommand(() -> RobotContainer.m_climber.stepReels(Constants.CLIMBER_REEL_STEP_MAGNITUDE), RobotContainer.m_climber);
    // m_stopExtendClimberReelPos =
    //   new InstantCommand(() -> this.m_extendClimberReelPosition.cancel());
    // m_retractClimberReelPosition =
    //   new RunCommand(() -> RobotContainer.m_climber.stepReels(-Constants.CLIMBER_REEL_STEP_MAGNITUDE), RobotContainer.m_climber);
    // m_stopRetractClimberReelPos =
    //   new InstantCommand(() -> this.m_retractClimberReelPosition.cancel());

    // m_extendClimberLiftPosition =
    //   new RunCommand(() -> RobotContainer.m_climber.stepLifts(Constants.CLIMBER_LIFT_STEP_MAGNITUDE), RobotContainer.m_climber);
    // m_stopExtendClimberLiftPos =
    //   new InstantCommand(() -> this.m_extendClimberLiftPosition.cancel());
    // m_retractClimberLiftPosition =
    //   new RunCommand(() -> RobotContainer.m_climber.stepLifts(-Constants.CLIMBER_LIFT_STEP_MAGNITUDE), RobotContainer.m_climber);
    // m_stopRetractClimberLiftPos =
    //   new InstantCommand(() -> this.m_retractClimberLiftPosition.cancel());

    m_extendClimberReelPosition =
      new InstantCommand(() -> RobotContainer.m_climber.setReelsPower(0.5), RobotContainer.m_climber);
    m_stopExtendClimberReelPos =
      new InstantCommand(() -> RobotContainer.m_climber.setReelsPower(0.0), RobotContainer.m_climber);
    m_retractClimberReelPosition =
      new InstantCommand(() -> RobotContainer.m_climber.setReelsPower(-0.5), RobotContainer.m_climber);
    m_stopRetractClimberReelPos =
      new InstantCommand(() -> RobotContainer.m_climber.setReelsPower(0.0), RobotContainer.m_climber);

    m_extendClimberLiftPosition =
      new InstantCommand(() -> RobotContainer.m_climber.setLiftsPower(0.6), RobotContainer.m_climber);
    m_stopExtendClimberLiftPos =
      new InstantCommand(() -> RobotContainer.m_climber.setLiftsPower(0.0), RobotContainer.m_climber);
    m_retractClimberLiftPosition =
      new InstantCommand(() -> RobotContainer.m_climber.setLiftsPower(-0.6), RobotContainer.m_climber);
    m_stopRetractClimberLiftPos =
      new InstantCommand(() -> RobotContainer.m_climber.setLiftsPower(0.0), RobotContainer.m_climber);

    /* Control Panel Command Instantiations */
    // m_setControlPanelSpinnerPower =
    //   new InstantCommand(() -> RobotContainer.m_controlPanel.setPanelSpinnerPower(Constants.CONTROL_PANEL_SPINNER_POWER));
    // m_setControlPanelSpinnerRPM = 
    //   new InstantCommand(() -> RobotContainer.m_controlPanel.setPanelSpinnerRPM(Constants.CONTROL_PANEL_SPINNER_RPM));
    // m_stopControlPanelSpinner =
    //   new InstantCommand(() -> RobotContainer.m_controlPanel.setPanelSpinnerPower(0.0));

    /* Intake Inline Command Instantiations */ 
    m_toggleIntakePistons =
      new InstantCommand(() -> RobotContainer.m_intake.toggleExtender());

    m_setIntakeWheelsRPM = 
      new InstantCommand(() -> RobotContainer.m_intake.setWheelRPM(Constants.INTAKE_WHEEL_RPM), RobotContainer.m_intake);
    m_stopIntakeWheels = 
      new InstantCommand(() -> RobotContainer.m_intake.setWheelRPM(0), RobotContainer.m_intake);
      
    /* Launcher Inline Command Instantiations */
    m_jogLauncherAngleUp =
      new InstantCommand(() -> RobotContainer.m_launcher.setPivotPower(-Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    m_jogLauncherAngleDown =
      new InstantCommand(() -> RobotContainer.m_launcher.setPivotPower(Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    m_stopLauncherAngle =
      new InstantCommand(() -> RobotContainer.m_launcher.setPivotPower(0.0));
    
    m_setLauncherPreset =
      new LauncherSetPreset();
  }
}