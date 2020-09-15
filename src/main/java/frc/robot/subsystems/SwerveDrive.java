package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase
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
 
    /* x1 and y1 are inputs from the driving joystick while x2 is input from the rotation joystick */
    public void drive(double x1, double y1, double x2)
    {
        double r = Math.sqrt((Constants.SWERVE_WHEEL_DISTANCE_X * Constants.SWERVE_WHEEL_DISTANCE_X) + 
        (Constants.SWERVE_WHEEL_DISTANCE_Y * Constants.SWERVE_WHEEL_DISTANCE_Y));

        y1 *= -1;

        //Creates vaiables that will calculate the speeds of the motors
        double a = x1 - x2 * (Constants.SWERVE_WHEEL_DISTANCE_X/r);
        double b = x1 + x2 * (Constants.SWERVE_WHEEL_DISTANCE_X/r);
        double c = y1 - x2 * (Constants.SWERVE_WHEEL_DISTANCE_Y/r);
        double d = y1 + x2 * (Constants.SWERVE_WHEEL_DISTANCE_Y/r);

        //Calculates speed for each wheel between 0 and 1
        double m_backRightSpeed = Math.sqrt((a * a) + (d * d));
        double m_backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double m_frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double m_frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        //atan2: returns value between -pi & pi for the coordinate (x,y)
        //Calculates angles for each wheel between -1 and 1
        double m_backRightAngle = Math.atan2(a,d) / Math.PI;
        double m_backLeftAngle = Math.atan2(a, c) / Math.PI;
        double m_frontRightAngle = Math.atan2(b, d) / Math.PI;
        double m_frogntLeftAngle = Math.atan2(b, c) / Math.PI;
    }

    public SwerveDrive()
    {
        //0.0 values are placeholders
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