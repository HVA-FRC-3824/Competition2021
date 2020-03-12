package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandGroupTemplate extends SequentialCommandGroup
{
  public CommandGroupTemplate()
  {
    addCommands();

    // Pose2d sCurveForward_startingPose = new Pose2d(0, 0, new Rotation2d(0));
    // List<Translation2d> sCurveForward_waypoints = List.of(
    //                                           new Translation2d(1,1),
    //                                           new Translation2d(2, -1)
    //                                         );
    // Pose2d sCurveForward_endingPose = new Pose2d(3, 0, new Rotation2d(0));

    // Pose2d sCurveBackward_startingPose = new Pose2d(3, 0, new Rotation2d(0));
    // List<Translation2d> sCurveBackward_waypoints = List.of(
    //                                           new Translation2d(2, -1),
    //                                           new Translation2d(1, 1)
    //                                         );
    // Pose2d sCurveBackward_endingPose = new Pose2d(0, 0, new Rotation2d(0));

    // /**
    //  * S-curve forward and backward.
    //  */
    // addCommands(
    //   // go forward
    //   RobotContainer.m_chassis.generateRamsete(sCurveForward_startingPose, sCurveForward_waypoints, sCurveForward_endingPose, false),
    //   // wait for 3 seconds
    //   new WaitCommand(3.0),
    //   // go backward
    //   RobotContainer.m_chassis.generateRamsete(sCurveBackward_startingPose, sCurveBackward_waypoints, sCurveBackward_endingPose, true)
    // );
  }
}