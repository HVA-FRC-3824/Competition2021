package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrepareTrajectoryCommand extends CommandBase 
{
  private String m_pathName;
  private boolean m_isFirstPath;
  private SequentialCommandGroup m_followPath;
  private boolean m_testingHeading;

  public PrepareTrajectoryCommand(String pathName, boolean isFirstPath, boolean testHeading)
  {
    SmartDashboard.putString("DONE", "CONSTRUCT");
    m_pathName = pathName;
    m_isFirstPath = isFirstPath;
    m_testingHeading = testHeading;

    SmartDashboard.putString("DONE", "START");
      /* Get path/trajectory to follow from PathWeaver json file. */
    String trajectoryJSONFilePath = "paths/" + m_pathName + ".wpilib.json";
    Trajectory trajectory = null;
    try
    {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONFilePath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex)
    {
      System.out.println("\nUnable to open trajectory: " + trajectoryJSONFilePath + "\n" + ex.getStackTrace() + "\n");
    }

    System.out.println("\n\n\nFOLLOWED " + m_pathName + "\n\n\n");

    /**
     * Make trajectory relative to robot rather than relative to field. 
     * Transforms original trajectory to shift to robot's zeroed position.
     */
    if (m_isFirstPath)
    {
      RobotContainer.initializeTrajTransform(trajectory);
    }
    trajectory = trajectory.transformBy(RobotContainer.getTrajTransform());

    /* Create command that will follow the trajectory. */
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      RobotContainer.m_chassis::getPose,
      new RamseteController(Constants.K_RAMSETE_B, Constants.K_RAMSETE_ZETA),
      new SimpleMotorFeedforward(Constants.K_S_VOLTS,
                                 Constants.K_V_VOLT_SECONDS_PER_METER,
                                 Constants.K_A_VOLT_SECONDS_SQUARED_PER_METER),
      Constants.K_DRIVE_KINEMATICS,
      RobotContainer.m_chassis::getWheelSpeeds,
      new PIDController(Constants.K_P_DRIVE_VEL, 0, 0),
      new PIDController(Constants.K_P_DRIVE_VEL, 0, 0),
      RobotContainer.m_chassis::driveWithVoltage, // RamseteCommand passes volts to the callback.
      RobotContainer.m_chassis
    );

    /* Return command group that will run path following command, then stop the robot at the end. */
    RobotContainer.setTrajCommand(ramseteCommand.andThen(new InstantCommand(() -> RobotContainer.m_chassis.driveWithVoltage(0, 0))));
  }

  @Override
  public void initialize()
  {
    if (m_testingHeading)
    {
      RobotContainer.m_chassis.setAngle(180);
      RobotContainer.m_chassis.setReversedTest(true);
    }
    // SmartDashboard.putString("DONE", "START");
    //   /* Get path/trajectory to follow from PathWeaver json file. */
    // String trajectoryJSONFilePath = "paths/" + m_pathName + ".wpilib.json";
    // Trajectory trajectory = null;
    // try
    // {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONFilePath);
    //   trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex)
    // {
    //   System.out.println("\nUnable to open trajectory: " + trajectoryJSONFilePath + "\n" + ex.getStackTrace() + "\n");
    // }

    // System.out.println("\n\n\nFOLLOWED " + m_pathName + "\n\n\n");

    // /**
    //  * Make trajectory relative to robot rather than relative to field. 
    //  * Transforms original trajectory to shift to robot's zeroed position.
    //  */
    // if (m_isFirstPath)
    // {
    //   RobotContainer.initializeTrajTransform(trajectory);
    // }
    // trajectory = trajectory.transformBy(RobotContainer.getTrajTransform());

    // /* Create command that will follow the trajectory. */
    // RamseteCommand ramseteCommand = new RamseteCommand(
    //   trajectory,
    //   RobotContainer.m_chassis::getPose,
    //   new RamseteController(Constants.K_RAMSETE_B, Constants.K_RAMSETE_ZETA),
    //   new SimpleMotorFeedforward(Constants.K_S_VOLTS,
    //                              Constants.K_V_VOLT_SECONDS_PER_METER,
    //                              Constants.K_A_VOLT_SECONDS_SQUARED_PER_METER),
    //   Constants.K_DRIVE_KINEMATICS,
    //   RobotContainer.m_chassis::getWheelSpeeds,
    //   new PIDController(Constants.K_P_DRIVE_VEL, 0, 0),
    //   new PIDController(Constants.K_P_DRIVE_VEL, 0, 0),
    //   RobotContainer.m_chassis::driveWithVoltage, // RamseteCommand passes volts to the callback.
    //   RobotContainer.m_chassis
    // );

    // /* Return command group that will run path following command, then stop the robot at the end. */
    // RobotContainer.setTrajCommand(ramseteCommand.andThen(new InstantCommand(() -> RobotContainer.m_chassis.driveWithVoltage(0, 0))));
  }
  
  @Override
  public void execute()
  {
  }

  @Override
  public void end(boolean interrupted)
  {
    SmartDashboard.putString("DONE", "DONE");
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }
}