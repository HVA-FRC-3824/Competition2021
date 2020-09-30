package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WheelDrive;

public class SwerveDrive extends SubsystemBase
{
    private WPI_TalonFX m_frontLeftMotor;
    private WPI_TalonFX m_frontRightMotor;
    private WPI_TalonFX m_backLeftMotor;
    private WPI_TalonFX m_backRightMotor;
    private final WPI_TalonFX m_frontLeftSteer;
    private final WPI_TalonFX m_frontRightSteer;
    private final WPI_TalonFX m_backLeftSteer;
    private final WPI_TalonFX m_backRightSteer;

    private final SwerveDriveKinematics m_kinematics;
    private final Translation2d m_wheelsLocation;

    private WheelDrive m_backRight;
    private WheelDrive m_backLeft;
    private WheelDrive m_frontRight;
    private WheelDrive m_frontLeft;
 
    /* x1 and y1 are inputs from the driving joystick while x2 is input from the rotation joystick */
    public void drive(final double x1, double y1, final double x2)
    {
        final double r = Math.sqrt((Constants.SWERVE_WHEEL_DISTANCE_X * Constants.SWERVE_WHEEL_DISTANCE_X) + 
        (Constants.SWERVE_WHEEL_DISTANCE_Y * Constants.SWERVE_WHEEL_DISTANCE_Y));

        y1 *= -1;

        //Creates vaiables that will calculate the speeds of the motors
        final double a = x1 - x2 * (Constants.SWERVE_WHEEL_DISTANCE_X/r);
        final double b = x1 + x2 * (Constants.SWERVE_WHEEL_DISTANCE_X/r);
        final double c = y1 - x2 * (Constants.SWERVE_WHEEL_DISTANCE_Y/r);
        final double d = y1 + x2 * (Constants.SWERVE_WHEEL_DISTANCE_Y/r);

        //Calculates speed for each wheel between 0 and 1
        final double m_backRightSpeed = Math.sqrt((a * a) + (d * d));
        final double m_backLeftSpeed = Math.sqrt((a * a) + (c * c));
        final double m_frontRightSpeed = Math.sqrt((b * b) + (d * d));
        final double m_frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        //atan2: returns value between -pi & pi for the coordinate (x,y)
        //Calculates angles for each wheel between -1 and 1
        final double m_backRightAngle = Math.atan2(a,d) / Math.PI;
        final double m_backLeftAngle = Math.atan2(a, c) / Math.PI;
        final double m_frontRightAngle = Math.atan2(b, d) / Math.PI;
        final double m_frontLeftAngle = Math.atan2(b, c) / Math.PI;

        m_backRight.drive(m_backRightSpeed, m_backRightAngle);
        m_backLeft.drive(m_backLeftSpeed, m_backLeftAngle);
        m_frontRight.drive(m_frontRightSpeed, m_frontRightAngle);
        m_frontLeft.drive(m_frontLeftSpeed, m_frontLeftAngle);
    }

    public SwerveDrive(final WheelDrive backRight, final WheelDrive backLeft, final WheelDrive frontRight, final WheelDrive frontLeft)
    {
        //0.0 values are placeholders
        m_frontLeftMotor = new WPI_TalonFX(Constants.SWERVE_FRONTLEFT_ID);
        RobotContainer.configureTalonFX(m_frontLeftMotor, false, false, 0.0, 0.0, 0.0, 0.0);

        m_frontRightMotor = new WPI_TalonFX(Constants.SWERVE_FRONTRIGHT_ID);
        RobotContainer.configureTalonFX(m_frontRightMotor, false, false, 0.0, 0.0, 0.0, 0.0);

        m_backLeftMotor = new WPI_TalonFX(Constants.SWERVE_BACKLEFT_ID);
        RobotContainer.configureTalonFX(m_backLeftMotor, false, false, 0.0, 0.0, 0.0, 0.0);

        m_backRightMotor = new WPI_TalonFX(Constants.SWERVE_BACKRIGHT_ID);
        RobotContainer.configureTalonFX(m_backRightMotor, false, false, 0.0, 0.0, 0.0, 0.0);

        m_frontLeftSteer = new WPI_TalonFX(Constants.SWERVE_FRONTLEFT_STEER_ID);
        RobotContainer.configureTalonFX(m_frontLeftSteer, false, false, 0.0, 0.0, 0.0, 0.0);

        m_frontRightSteer = new WPI_TalonFX(Constants.SWERVE_FRONTRIGHT_STEER_ID);
        RobotContainer.configureTalonFX(m_frontRightSteer, false, false, 0.0, 0.0, 0.0, 0.0);

        m_backLeftSteer = new WPI_TalonFX(Constants.SWERVE_BACKLEFT_STEER_ID);
        RobotContainer.configureTalonFX(m_backLeftSteer, false, false, 0.0, 0.0, 0.0, 0.0);

        m_backRightSteer = new WPI_TalonFX(Constants.SWERVE_BACKRIGHT_STEER_ID);
        RobotContainer.configureTalonFX(m_backRightSteer, false, false, 0.0, 0.0, 0.0, 0.0);

        m_wheelsLocation = new Translation2d(Constants.SWERVE_WHEEL_DISTANCE_X, Constants.SWERVE_WHEEL_DISTANCE_Y);

        m_kinematics = new SwerveDriveKinematics(m_wheelsLocation);

        this.m_backRight = m_backRight;
        this.m_backLeft = m_backLeft;
        this.m_frontRight = m_frontRight;
        this.m_frontLeft = m_frontLeft;
    }
}