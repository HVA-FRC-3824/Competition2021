package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
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
  // public final Command m_setChamberBaseRPM;
  // public final Command m_stopChamberBase;

  public final Command m_setChamberElevatorToLaunch;
  public final Command m_setChamberElevatorDown;
  public final Command m_stopChamberElevator;

  /* Chassis Inline Command Declarations */
  public final Command m_driveWithJoystick;

  public final Command m_shiftHighGear;
  public final Command m_shiftLowGear;

  public final Command m_toggleLimelight;

  public final Command m_setHeading;

  /* Climber Inline Command Declarations */
  public final Command m_extendClimberLeft;
  public final Command m_retractClimberLeft;
  public final Command m_stopClimberLeft;

  public final Command m_extendClimberRight;
  public final Command m_retractClimberRight;
  public final Command m_stopClimberRight;

  /* Control Panel Command Declarations */
  // public final Command m_setControlPanelSpinnerPower;
  // public final Command m_setControlPanelSpinnerRPM;
  // public final Command m_stopControlPanelSpinner;
  
  /* Defense Mode Commands */
  public final Command m_toggleDefenseMode;
  
  /* Intake Inline Command Declarations */
  public final Command m_toggleIntakePistons;

  public final Command m_setIntakeWheelsRPM;
  public final Command m_stopIntakeWheels;

  /* Launcher Inline Command Declarations */
  public final Command m_jogLauncherAngleUp;
  public final Command m_jogLauncherAngleDown;
  public final Command m_stopLauncherAngle;

  public final Command m_setLauncherVision; // Turns chassis and sets launcher angle & rpms.
  public final Command m_setLauncherPreset; // Set launcher angle & rpms to a specified setpoint.
  public final Command m_stopLaunchSequence; // Enables teleop driving and stops launcher angle & rpms.

  /* LED Inline Command Declarations */
  public final Command m_chaseInwards;
  public final Command m_chaseOutwards;
  public final Command m_rainbow;
  public final Command m_neutral;


  
  public InlineCommands()
  {
    /**
     * Instantiate inline commands here.
     */

    /* Chamber Inline Command Instantiations */
    // m_setChamberBaseRPM =
    //   new InstantCommand(() -> RobotContainer.m_chamber.setBaseRPM(Constants.CHAMBER_BASE_RPM));
    // m_stopChamberBase =
    //   new InstantCommand(() -> RobotContainer.m_chamber.setBasePower(0.0));

    m_setChamberElevatorToLaunch =
      new InstantCommand(() -> RobotContainer.m_chamber.setElevatorPower(-0.5));
    m_setChamberElevatorDown =
      new InstantCommand(() -> RobotContainer.m_chamber.setElevatorPower(0.5));
    m_stopChamberElevator =
      new InstantCommand(() -> RobotContainer.m_chamber.setElevatorPower(0.0));

    /* Chassis Inline Command Instantiations */
    // m_driveWithJoystick =
    //   new RunCommand(() -> RobotContainer.m_chassis.teleopDrive(RobotContainer.m_OI.getDriverJoystick().getY(), 
    //                 RobotContainer.m_OI.getDriverJoystick().getTwist()), RobotContainer.m_chassis);
                    
    m_driveWithJoystick =
    new RunCommand(() -> RobotContainer.m_chassis.convertSwerveValues(RobotContainer.m_OI.getDriverJoystick().getRawAxis(0), 
                  RobotContainer.m_OI.getDriverJoystick().getRawAxis(1), RobotContainer.m_OI.getDriverJoystick().getRawAxis(4)), 
                  RobotContainer.m_chassis);
  
    m_shiftHighGear =
      new InstantCommand(() -> RobotContainer.m_chassis.shiftHighGear());
    m_shiftLowGear =
      new InstantCommand(() -> RobotContainer.m_chassis.shiftLowGear());

    m_toggleLimelight =
      new InstantCommand(() -> RobotContainer.m_limelight.toggleMode());

    m_setHeading =
      new InstantCommand(() -> RobotContainer.m_chassis.zeroHeading());

    /* Climber Inline Command Instantiations */
    m_extendClimberLeft =
      new ClimberSetLeftPower(1);
    m_retractClimberLeft =
      new ClimberSetLeftPower(-1);
    m_stopClimberLeft =
      new InstantCommand(() -> this.m_extendClimberLeft.cancel()).alongWith(new InstantCommand(() -> this.m_retractClimberLeft.cancel()))
        .andThen(new InstantCommand(() -> RobotContainer.m_climber.setLeftLiftPower(0.0)).alongWith(new InstantCommand(() -> RobotContainer.m_climber.setLeftReelPower(0.0))));

    m_extendClimberRight =
      new ClimberSetRightPower(1);
    m_retractClimberRight =
      new ClimberSetRightPower(-1);
    m_stopClimberRight =
      new InstantCommand(() -> this.m_extendClimberRight.cancel()).alongWith(new InstantCommand(() -> this.m_retractClimberRight.cancel()))
        .andThen(new InstantCommand(() -> RobotContainer.m_climber.setRightLiftPower(0.0)).alongWith(new InstantCommand(() -> RobotContainer.m_climber.setRightReelPower(0.0))));

    /* Control Panel Command Instantiations */
    // m_setControlPanelSpinnerPower =
    //   new InstantCommand(() -> RobotContainer.m_controlPanel.setPanelSpinnerPower(Constants.CONTROL_PANEL_SPINNER_POWER));
    // m_setControlPanelSpinnerRPM = 
    //   new InstantCommand(() -> RobotContainer.m_controlPanel.setPanelSpinnerRPM(Constants.CONTROL_PANEL_SPINNER_RPM));
    // m_stopControlPanelSpinner =
    //   new InstantCommand(() -> RobotContainer.m_controlPanel.setPanelSpinnerPower(0.0));

    /* Defense Mode Command Instantiations */
    m_toggleDefenseMode =
      new InstantCommand(() -> RobotContainer.m_LEDs.toggleDefenseMode());

    /* Intake Inline Command Instantiations */ 
    m_toggleIntakePistons =
      new InstantCommand(() -> RobotContainer.m_intake.toggleExtender());

    m_setIntakeWheelsRPM = 
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.5), RobotContainer.m_intake);
    m_stopIntakeWheels = 
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.0), RobotContainer.m_intake);
      
    /* Launcher Inline Command Instantiations */
    m_jogLauncherAngleUp =
      new RunCommand(() -> RobotContainer.m_launcher.setPivotPower(-Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    m_jogLauncherAngleDown =
      new RunCommand(() -> RobotContainer.m_launcher.setPivotPower(Constants.LAUNCHER_PIVOT_JOG_MAGNITUDE));
    m_stopLauncherAngle =
      new InstantCommand(() -> RobotContainer.m_launcher.setPivotPower(0.0)).alongWith(new InstantCommand(() -> this.m_jogLauncherAngleUp.cancel()), new InstantCommand(() -> this.m_jogLauncherAngleDown.cancel()));

    m_setLauncherVision =
      new ChassisTurnToTarget().andThen(new InstantCommand(() -> RobotContainer.m_limelight.setModeDriver()));
      //.alongWith(new LauncherAimForTarget(), new InstantCommand(() -> RobotContainer.m_LEDs.setLaunchingStatus(true)))
    m_setLauncherPreset =
      new LauncherSetPreset().alongWith(new InstantCommand(() -> RobotContainer.m_LEDs.setLaunchingStatus(true)));
    m_stopLaunchSequence =
      new RunCommand(() -> RobotContainer.m_chassis.teleopDrive(RobotContainer.m_OI.getDriverJoystick().getY(), 
      RobotContainer.m_OI.getDriverJoystick().getTwist()), RobotContainer.m_chassis).alongWith(new InstantCommand(() -> RobotContainer.m_launcher.stopLauncher(), RobotContainer.m_launcher), 
                                    new InstantCommand(() -> RobotContainer.m_LEDs.setLaunchingStatus(false)));

    /* LEDs Inline Command Instantiations */
    m_chaseInwards =
      new RunCommand(() -> RobotContainer.m_LEDs.chaseInward()); 
    m_chaseOutwards =
      new RunCommand(() -> RobotContainer.m_LEDs.chaseOutward()); 
    m_rainbow =
      new RunCommand(() -> RobotContainer.m_LEDs.rainbow()); 
    m_neutral = 
      new RunCommand(() -> RobotContainer.m_LEDs.neutral());
  }
}