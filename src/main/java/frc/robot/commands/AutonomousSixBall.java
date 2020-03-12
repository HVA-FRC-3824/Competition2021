package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousSixBall extends SequentialCommandGroup
{
  Pose2d sixBallForward_startingPose = new Pose2d(0, 0, new Rotation2d(0));
  List<Translation2d> sixBallForward_waypoints = List.of(
                                            new Translation2d(1.15, 1.569)
                                          );
  Pose2d sixBallForward_endingPose = new Pose2d(4.95, 1.569, new Rotation2d(0));

  Pose2d sixBallBackward_startingPose = new Pose2d(4.95, 1.569, new Rotation2d(0));
  List<Translation2d> sixBallBackward_waypoints = List.of(
                                            new Translation2d(1.15, 0),
                                            new Translation2d(0.5, 0)
                                            );
  Pose2d sixBallBackward_endingPose = new Pose2d(0, 0, new Rotation2d(0));

  public AutonomousSixBall()
  {
    addCommands(
      // start launcher
      new InstantCommand(() -> RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_AUTO_INIT_TOP_RPM, Constants.LAUNCHER_AUTO_INIT_BOTTOM_RPM, 
                               Constants.LAUNCHER_AUTO_INIT_ANGLE)),
      //wait for launcher to speed up
      new WaitCommand(1),
      // run chamber
      new InstantCommand(() -> RobotContainer.m_chamber.setElevatorPower(0.5), RobotContainer.m_chamber),
      //wait for balls to launch
      new WaitCommand(2),
      //extend intake
      new InstantCommand(() -> RobotContainer.m_intake.extendExtender()),
      //stop chamber
      new InstantCommand(() -> RobotContainer.m_chamber.setElevatorPower(0)),
      // start automatic chamber indexing
      new InstantCommand(() -> RobotContainer.m_chamber.initAutoIndex()),
      //start intake
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(Constants.INTAKE_WHEEL_POWER)),
      //run chamber base
      new InstantCommand(() -> RobotContainer.m_chamber.setBaseRPM(Constants.CHAMBER_BASE_RPM)),
      //follow path to pick up balls
      RobotContainer.m_chassis.generateRamsete(sixBallForward_startingPose, sixBallForward_waypoints, sixBallForward_endingPose, 1.5, false),
      //follow path back to starting point
      RobotContainer.m_chassis.generateRamsete(sixBallBackward_startingPose, sixBallBackward_waypoints, sixBallBackward_endingPose, 2.5, true),
      // run chamber
      new InstantCommand(() -> RobotContainer.m_chamber.setElevatorPower(0.5), RobotContainer.m_chamber),
      //stop intake
      new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0)),
      //retract intake
      new InstantCommand(() -> RobotContainer.m_intake.retractExtender()),
      //wait for balls to launch
      new WaitCommand(2),
      //stop launcher
      new InstantCommand(() -> RobotContainer.m_launcher.stopLauncher()),
      //stop chamber elevator and base
      new InstantCommand(() -> RobotContainer.m_chamber.setElevatorPower(0)),
      new InstantCommand(() -> RobotContainer.m_chamber.setBasePower(0))
    );
  }
}