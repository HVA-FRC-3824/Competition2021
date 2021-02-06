package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoNavBarrel extends SequentialCommandGroup
{
    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    List <Translation2d> wayPoints = List.of(
                                    new Translation2d(2.7, -0.25),
                                    new Translation2d(2.75, -1),
                                    new Translation2d(2, -2.5),
                                    new Translation2d(1.85, -0.5),
                                    new Translation2d(2.7, 0.15),
                                    new Translation2d(5.05, 0.6),
                                    new Translation2d(4.8, 1.5),
                                    new Translation2d(4.1, 0.8),
                                    new Translation2d(4.4, 0),
                                    new Translation2d(6.15, -1),
                                    new Translation2d(6.4, 0)
                                    );
    Pose2d endingPose = new Pose2d(0, 0.5, new Rotation2d(0));

    public AutoNavBarrel()
    {
        RobotContainer.m_chassis.generateRamsete(startingPose, wayPoints, endingPose, 1, false);
    }
}