package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousSixBall extends SequentialCommandGroup
{
  public AutonomousSixBall()
  {
    addCommands(
      // extend intake
      // prepare for launch three balls loaded with initially
      new InstantCommand(() -> RobotContainer.m_intake.extendExtender())
        .alongWith(new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_AUTO_INIT_TOP_RPM, Constants.LAUNCHER_AUTO_INIT_BOTTOM_RPM, Constants.LAUNCHER_AUTO_INIT_ANGLE))),
      // turn on limelight
      // new InstantCommand(() -> RobotContainer.m_limelight.turnOnLED()),
      // wait for launcher to be ready
      new WaitCommand(1.0),
      // launch balls
      new InstantCommand(() -> RobotContainer.m_chamber.stepChamberDistance(50000)),
      // wait for balls to be done launching
      new WaitCommand(2.5),
      // stop launcher
      new InstantCommand(() -> RobotContainer.m_launcher.setTopWheelPower(0.0))
        .alongWith(new InstantCommand(() -> RobotContainer.m_launcher.setBottomWheelPower(0.0)), 
                   new InstantCommand(() -> RobotContainer.m_launcher.setPivotPower(0.0))),
      // spin intake & base
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.5))
        .alongWith(new InstantCommand(() -> RobotContainer.m_chamber.setBaseRPM(Constants.CHAMBER_BASE_RPM-200))),
      // blink LEDs
      // new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
      // prepare for launching newly picked up three balls.
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_AUTO_TRENCH_TOP_RPM, Constants.LAUNCHER_AUTO_TRENCH_BOTTOM_RPM, Constants.LAUNCHER_AUTO_TRENCH_ANGLE)),
      // follow path
      RobotContainer.m_chassis.generateRamsete("sixBall", true),
      // retract intake, stop intake, stop base
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.0))
        .alongWith(new InstantCommand(() -> RobotContainer.m_intake.retractExtender())),
      // turn on limelight
      // new InstantCommand(() -> RobotContainer.m_limelight.turnOnLED()),
      // align with vision with timeout OR pray straight voltage turns the right amount
      new InstantCommand(() -> RobotContainer.m_chassis.autoDrive(0.0, -0.25)),
      // turn for half a second
      new WaitCommand(0.25),
      // stop turning
      new InstantCommand(() -> RobotContainer.m_chassis.autoDrive(0.0, 0.0)),
      // launch balls
      new InstantCommand(() -> RobotContainer.m_chamber.stepChamberDistance(50000)),
      // wait for balls to be done launching
      new WaitCommand(2.5),
      // stop launcher
      new InstantCommand(() -> RobotContainer.m_launcher.setTopWheelPower(0.0))
        .alongWith(new InstantCommand(() -> RobotContainer.m_launcher.setBottomWheelPower(0.0)),
                   new InstantCommand(() -> RobotContainer.m_launcher.setPivotPower(0.0)),
                   new InstantCommand(() -> RobotContainer.m_chamber.setBasePower(0.0)))
      // turn off led
      // new InstantCommand(() -> RobotContainer.m_limelight.turnOffLED())
    );
  }
}