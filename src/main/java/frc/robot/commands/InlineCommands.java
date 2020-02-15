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
  public final Command m_setChamberElevatorRPM;
  public final Command m_stopChamberElevator;

  /* Chassis Inline Command Declarations */
  public final Command m_driveWithJoystick;

  public final Command m_shiftHighGear;
  public final Command m_shiftLowGear;
  
  public final Command m_chassisTurnToTarget;
  public final Command m_chassisAutoTurnToTarget;
  public final Command m_stopChassisTurnToTarget;

  public final Command m_turnChassisForInitiationLine;
  public final Command m_turnChassisForCloseTrench;
  public final Command m_turnChassisForFarTrench;
  
  public final Command m_stopTurningChassisAtInitiationLine;
  public final Command m_stopTurningChassisAtCloseTrench;
  public final Command m_stopTurningChassisAtFarTrench;
  
  /* Climber Inline Command Declarations */
  public final Command m_jogClimberReelPositionUp;
  public final Command m_jogClimberReelPositionDown;

  public final Command m_jogClimberZiplinePositionLeft;
  public final Command m_jogClimberZiplinePositionRight;

  public final Command m_setClimberZiplinePower;
  public final Command m_stopClimberZipline;

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

  public final Command m_setLauncherFeederPower;
  public final Command m_setLauncherFeederRPM;
  public final Command m_stopLauncherFeeder;
  
  public InlineCommands()
  {
    /**
     * Instantiate inline commands here.
     */

    /* Chassis Inline Command Instantiations */
    m_driveWithJoystick =
      new RunCommand(() -> RobotContainer.m_chassis.teleopDrive(RobotContainer.m_OI.getDriverJoystick().getY(), 
                    RobotContainer.m_OI.getDriverJoystick().getTwist()), RobotContainer.m_chassis);
    m_shiftHighGear =
      new InstantCommand(() -> RobotContainer.m_chassis.shiftHighGear());
    m_shiftLowGear =
      new InstantCommand(() -> RobotContainer.m_chassis.shiftLowGear());

    m_chassisTurnToTarget =
      new ChassisTurnToTarget().andThen(new InstantCommand(() -> RobotContainer.m_limelight.setModeDriver()));
    m_chassisAutoTurnToTarget =
      new ChassisTurnToTarget();
    m_stopChassisTurnToTarget =
      new InstantCommand(() -> this.m_chassisTurnToTarget.cancel());

    m_turnChassisForInitiationLine =
      new ChassisTurnToAngle(Constants.CHASSIS_INITIATION_LINE_ANGLE);
    m_turnChassisForCloseTrench =    
      new ChassisTurnToAngle(Constants.CHASSIS_CLOSE_TRENCH_ANGLE);
    m_turnChassisForFarTrench =
      new ChassisTurnToAngle(Constants.CHASSIS_FAR_TRENCH_ANGLE);
      
    m_stopTurningChassisAtInitiationLine =
      new InstantCommand(() -> this.m_turnChassisForInitiationLine.cancel());
    m_stopTurningChassisAtCloseTrench =
      new InstantCommand(() -> this.m_turnChassisForCloseTrench.cancel());
    m_stopTurningChassisAtFarTrench =
      new InstantCommand(() -> this.m_turnChassisForFarTrench.cancel());
  
    /* Chamber Inline Command Instantiations */
    m_setChamberElevatorPower =
      new InstantCommand(() -> RobotContainer.m_chamber.setChamberElevatorPower(Constants.CHAMBER_ELEVATOR_POWER));
    m_setChamberElevatorRPM =
      new InstantCommand(() -> RobotContainer.m_chamber.setChamberElevatorRMP(Constants.CHAMBER_ELEVATOR_RPM));
    m_stopChamberElevator =
      new InstantCommand(() -> RobotContainer.m_chamber.setChamberElevatorPower(0.0));

    /* Climber Inline Command Instantiations */
    m_jogClimberReelPositionUp =
      new InstantCommand(() -> RobotContainer.m_climber.setReelPosition(RobotContainer.m_climber.getReelCurrentPosition() 
                        + Constants.CLIMBER_REEL_JOG_MAGNITUDE));
    m_jogClimberReelPositionDown = 
      new InstantCommand(() -> RobotContainer.m_climber.setReelPosition(RobotContainer.m_climber.getReelCurrentPosition() 
                        - Constants.CLIMBER_REEL_JOG_MAGNITUDE));
    
    m_jogClimberZiplinePositionLeft =
      new InstantCommand(() -> RobotContainer.m_climber.setZiplinePosition(RobotContainer.m_climber.getZiplineCurrentPosition() 
                        + Constants.CLIMBER_ZIPLINE_JOG_MAGNITUDE));
    m_jogClimberZiplinePositionRight =
      new InstantCommand(() -> RobotContainer.m_climber.setZiplinePosition(RobotContainer.m_climber.getZiplineCurrentPosition() 
                        - Constants.CLIMBER_ZIPLINE_JOG_MAGNITUDE));

    m_setClimberZiplinePower =
      new InstantCommand(() -> RobotContainer.m_climber.setZiplinePower(Constants.CLIMBER_ZIPLINE_POWER));
    m_stopClimberZipline = 
      new InstantCommand(() -> RobotContainer.m_climber.setZiplinePower(0.0));

    // bruh jovi moment 2.0 <- lmao >:(

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
      new InstantCommand(() -> RobotContainer.m_launcher.setTopWheelRPM((int)(RobotContainer.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherBottomWheelRPM =
      new InstantCommand(() -> RobotContainer.m_launcher.setBottomWheelRPM((int)(RobotContainer.m_OI.getOperatorController().
                        getRawAxis(Constants.OPERATOR_LAUNCHER_WHEELS_SLIDER_ID) * Constants.LAUNCHER_WHEEL_MAX_RPM)));
    m_setLauncherWheelsRPM = new ParallelCommandGroup(m_setLauncherTopWheelRPM, m_setLauncherBottomWheelRPM);
    m_setLauncherWheelsRPM.addRequirements(RobotContainer.m_launcher);

    m_stopLauncherWheels =
      new InstantCommand(() -> RobotContainer.m_launcher.stopWheels(), 
                         RobotContainer.m_launcher);

    m_jogLauncherAngleUp =
      new InstantCommand(() -> RobotContainer.m_launcher.setAngle(RobotContainer.m_launcher.getCurrentAngle() 
                        + Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE, 0.0));
    m_jogLauncherAngleDown =
      new InstantCommand(() -> RobotContainer.m_launcher.setAngle(RobotContainer.m_launcher.getCurrentAngle() 
                        - Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE, 0.0));
    
    m_setLauncherForInitiationLine =
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_INITIATION_LINE_TOP_RPM, 
                        Constants.LAUNCHER_INITIATION_LINE_BOTTOM_RPM, Constants.LAUNCHER_INITIATION_LINE_ANGLE, -0.50), 
                        RobotContainer.m_launcher);
    m_setLauncherForCloseTrench =
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_CLOSE_TRENCH_TOP_RPM, 
                        Constants.LAUNCHER_CLOSE_TRENCH_BOTTOM_RPM, Constants.LAUNCHER_CLOSE_TRENCH_ANGLE, 0.15), RobotContainer.m_launcher);
    m_setLauncherForFarTrench =
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_FAR_TRENCH_TOP_RPM, 
                        Constants.LAUNCHER_FAR_TRENCH_BOTTOM_RPM, Constants.LAUNCHER_FAR_TRENCH_ANGLE, 0.35), RobotContainer.m_launcher);

    m_setLauncherFeederPower =
      new InstantCommand(() -> RobotContainer.m_launcher.setFeederPower(Constants.LAUNCHER_FEEDER_POWER));
                        // alongWith(RobotContainer.m_inlineCommands.m_setChamberElevatorPower);
    m_setLauncherFeederRPM =
      new InstantCommand(() -> RobotContainer.m_launcher.setFeederRPM(Constants.LAUNCHER_FEEDER_RPM));
                        // alongWith(RobotContainer.m_inlineCommands.m_setChamberElevatorRPM);
    m_stopLauncherFeeder =
      new InstantCommand(() -> RobotContainer.m_launcher.setFeederPower(0.0));
  }
}