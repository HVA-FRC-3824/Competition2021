package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Swerve extends SubsystemBase
{
    private WPI_TalonFX m_frontLeft;
    private WPI_TalonFX m_frontRight;
    private WPI_TalonFX m_backLeft;
    private WPI_TalonFX m_backRight;
    private WPI_TalonFX m_frontLeftSteer;
    private WPI_TalonFX m_frontRightSteer;
    private WPI_TalonFX m_backLeftSteer;
    private WPI_TalonFX m_backRightSteer;

    private SwerveDriveKinematics m_kinematics;
    private Translation2d wheelsLocation;
 

    public Swerve()
    {
    m_frontLeft = new WPI_TalonFX(Constants.SWERVE_FRONTLEFT_ID);
    RobotContainer.configureTalonFX(m_frontLeft, false, false, 0.0, 0.0, 0.0, 0.0);

    m_frontRight = new WPI_TalonFX(Constants.SWERVE_FRONTRIGHT_ID);
    RobotContainer.configureTalonFX(m_frontRight, false, false, 0.0, 0.0, 0.0, 0.0);

    m_backLeft = new WPI_TalonFX(Constants.SWERVE_BACKLEFT_ID);
    RobotContainer.configureTalonFX(m_backLeft, false, false, 0.0, 0.0, 0.0, 0.0);

    m_backRight = new WPI_TalonFX(Constants.SWERVE_BACKRIGHT_ID);
    RobotContainer.configureTalonFX(m_backRight, false, false, 0.0, 0.0, 0.0, 0.0);

    m_frontLeftSteer = new WPI_TalonFX(Constants.SWERVE_FRONTLEFT_STEER_ID);
    RobotContainer.configureTalonFX(m_frontLeftSteer, false, false, 0.0, 0.0, 0.0, 0.0);

    m_frontRightSteer = new WPI_TalonFX(Constants.SWERVE_FRONTRIGHT_STEER_ID);
    RobotContainer.configureTalonFX(m_frontRightSteer, false, false, 0.0, 0.0, 0.0, 0.0);

    m_backLeftSteer = new WPI_TalonFX(Constants.SWERVE_BACKLEFT_STEER_ID);
    RobotContainer.configureTalonFX(m_backLeftSteer, false, false, 0.0, 0.0, 0.0, 0.0);

    m_backRightSteer = new WPI_TalonFX(Constants.SWERVE_BACKRIGHT_STEER_ID);
    RobotContainer.configureTalonFX(m_backRightSteer, false, false, 0.0, 0.0, 0.0, 0.0);

    wheelsLocation = new Translation2d(Constants.SWERVE_WHEEL_DISTANCE_X, Constants.SWERVE_WHEEL_DISTANCE_Y);

    m_kinematics = new SwerveDriveKinematics(wheelsLocation);

    }
}