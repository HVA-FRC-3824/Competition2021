package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoNavBarrel extends SequentialCommandGroup
{
    //Creates path from a start point to an end point
    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d endingPose = new Pose2d(1.275	,	-1.875, new Rotation2d(0));
    
    List<Translation2d> waypoints = List.of(
                                              new Translation2d(2.75    ,   -0.198),
                                              new Translation2d(3.025   ,   -1.114),
                                              new Translation2d(1.899   ,   -1.138),
                                              new Translation2d(1.911	,	-0.377),
                                              new Translation2d(2.74	,	-0.198),
                                              new Translation2d(5.381	, 	0.693),
                                              new Translation2d(4.831	,  	1.435),
                                              new Translation2d(3.971	,	0.977),
                                              new Translation2d(4.961	,	-0.625),
                                              new Translation2d(6.631	,	-1.144),
                                              new Translation2d(6.829	,	-0.52),
                                              new Translation2d(6.266	,	-0.037)
                                            );

    public AutoNavBarrel()
    {
        // Follow path
        // RobotContainer.m_chassis.generateSwerveCommand(startingPose, waypoints, endingPose, 2.5, false);
    }
}