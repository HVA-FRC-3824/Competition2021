package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Chassis extends SubsystemBase 
{
  /**
   * Declaring objects for the drivetrain
   */
  private WPI_TalonFX m_leftMaster;
  private WPI_TalonFX m_leftSlave;

  private WPI_TalonFX m_rightMaster;
  private WPI_TalonFX m_rightSlave;

  private DifferentialDrive m_differentialDrive;

  private AHRS m_ahrs;

  private Compressor m_compressor;

  private DoubleSolenoid m_gearShift;

  private WPI_TalonFX m_angleMotorFrontRight;
  private WPI_TalonFX m_speedMotorFrontRight;
  
  private WPI_TalonFX m_angleMotorFrontLeft;
  private WPI_TalonFX m_speedMotorFrontLeft;

  private WPI_TalonFX m_angleMotorBackLeft;
  private WPI_TalonFX m_speedMotorBackLeft;

  private WPI_TalonFX m_angleMotorBackRight;
  private WPI_TalonFX m_speedMotorBackRight;

  //{VX, VY, Speed, Angle, Previous Angle, Offset}
  public double [] frontRight = {0, 0, 0, 0, 0, 0};
  public double [] frontLeft = {0, 0, 0, 0, 0, 0};
  public double [] backLeft = {0, 0, 0, 0, 0, 0};
  public double [] backRight = {0, 0, 0, 0, 0, 0};

  /**
   * Declaring objects for autonomous path following.
   */
  private final DifferentialDriveOdometry m_odometry;

  public Chassis() 
  {
    /**
     * Instantiating drivetrain objects
     */
    m_leftMaster = new WPI_TalonFX(Constants.CHASSIS_LEFT_MASTER_ID);
    RobotContainer.configureTalonFX(m_leftMaster, false, false, 0.0, 0.0, 0.0, 0.0);

    m_leftSlave = new WPI_TalonFX(Constants.CHASSIS_LEFT_SLAVE_ID);
    RobotContainer.configureTalonFX(m_leftSlave, false, false, 0.0, 0.0, 0.0, 0.0);

    m_leftSlave.follow(m_leftMaster);

    m_rightMaster = new WPI_TalonFX(Constants.CHASSIS_RIGHT_MASTER_ID);
    RobotContainer.configureTalonFX(m_rightMaster, false, false, 0.0, 0.0, 0.0, 0.0);

    m_rightSlave = new WPI_TalonFX(Constants.CHASSIS_RIGHT_SLAVE_ID);
    RobotContainer.configureTalonFX(m_rightSlave, false, false, 0.0, 0.0, 0.0, 0.0);

    m_rightSlave.follow(m_rightMaster);

    m_differentialDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
    m_differentialDrive.setSafetyEnabled(false);

    /**
     * Try to instantiate the navx gyro with exception catch
     */
    try 
    {
      m_ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) 
    {
      System.out.println("\nError instantiating navX-MXP:\n" + ex.getMessage() + "\n");
    }

    /**
     * Pneumatics objects
     */
    m_compressor = new Compressor();

    m_gearShift = new DoubleSolenoid(Constants.CHASSIS_GEARSHIFT_PORT_A, Constants.CHASSIS_GEARSHIFT_PORT_B);

    /**
     * Autonomous path following objects
     */

    /* Used for tracking robot pose. */
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeading()));

    /**
     * doubleious methods to call when chassis subsystem first starts up.
     */

    /* Reset encoders & gyro to ensure autonomous path following is correct. */
    this.resetEncoders();
    this.zeroHeading();

    m_angleMotorFrontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_ANGLE_MOTOR_ID);
    RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, Constants.K_CHASSIS_ANGLE_P, 
    Constants.K_CHASSIS_ANGLE_I, Constants.K_CHASSIS_ANGLE_D);

    m_speedMotorFrontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_SPEED_MOTOR_ID);
    RobotContainer.configureTalonFX(m_speedMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);

    m_angleMotorFrontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_ANGLE_MOTOR_ID);
    RobotContainer.configureTalonFX(m_angleMotorFrontLeft, false, false, 0.0, Constants.K_CHASSIS_ANGLE_P, 
    Constants.K_CHASSIS_ANGLE_I, Constants.K_CHASSIS_ANGLE_D);

    m_speedMotorFrontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_SPEED_MOTOR_ID);
    RobotContainer.configureTalonFX(m_speedMotorFrontLeft, false, false, 0.0, 0.0, 0.0, 0.0);
    
    m_angleMotorBackLeft = new WPI_TalonFX(Constants.BACK_LEFT_ANGLE_MOTOR_ID);
    RobotContainer.configureTalonFX(m_angleMotorBackLeft, false, false, 0.0, Constants.K_CHASSIS_ANGLE_P, 
    Constants.K_CHASSIS_ANGLE_I, Constants.K_CHASSIS_ANGLE_D);

    m_speedMotorBackLeft = new WPI_TalonFX(Constants.BACK_LEFT_SPEED_MOTOR_ID);
    RobotContainer.configureTalonFX(m_speedMotorBackLeft, false, false, 0.0, 0.0, 0.0, 0.0);

    m_angleMotorBackRight = new WPI_TalonFX(Constants.BACK_RIGHT_ANGLE_MOTOR_ID);
    RobotContainer.configureTalonFX(m_angleMotorBackRight, false, false, 0.0, Constants.K_CHASSIS_ANGLE_P, 
    Constants.K_CHASSIS_ANGLE_I, Constants.K_CHASSIS_ANGLE_D);

    m_speedMotorBackRight = new WPI_TalonFX(Constants.BACK_RIGHT_SPEED_MOTOR_ID);
    RobotContainer.configureTalonFX(m_speedMotorBackRight, false, false, 0.0, 0.0, 0.0, 0.0);
  }

  /**
   * This method will be called once per scheduler run.
   */
  @Override
  public void periodic()
  {
    /* Update odometry/position tracking of robot. */
    this.updateOdometry();

    /* Update drivetrain information on SmartDashboard for testing. */
    // this.displayDrivetrainInfo();
  }

  /**
   * Puts doubleious drivetrain parameters on the SmartDashboard for testing.
   */
  private void displayDrivetrainInfo()
  {
    SmartDashboard.putNumber("LEFT ENCODER", m_rightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("RIGHT ENCODER", -m_leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("LEFT VELOCITY", m_rightMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RIGHT VELOCITY", -m_leftMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("LEFT DISTANCE", this.getLeftEncoderDistance());
    SmartDashboard.putNumber("RIGHT DISTANCE", this.getRightEncoderDistance());
    SmartDashboard.putNumber("LEFT RATE", this.getLeftEncoderRate());
    SmartDashboard.putNumber("RIGHT RATE", this.getRightEncoderRate());
    SmartDashboard.putNumber("ANGLE", this.getAngle());
    SmartDashboard.putNumber("HEADING", this.getHeading());
    SmartDashboard.putNumber("LEFT VOLTAGE", m_leftMaster.getMotorOutputVoltage());
    SmartDashboard.putNumber("RIGHT VOLTAGE", m_rightMaster.getMotorOutputVoltage());
  }

  public WPI_TalonFX getMotor ()
  {
    return m_angleMotorFrontRight;
  }
  /**
   * Controls movement of robot drivetrain with passed in power and turn values
   * from driver input of joystick.
   * Allows external commands to control the private differentialDrive object.
   */
  public void teleopDrive(double power, double turn)
  {
    /* Reduces sensitivity of twist for turning. */
    turn = turn/1.5;
    if (power > Constants.CHASSIS_MAX_POWER)
    {
      power = Constants.CHASSIS_MAX_POWER;
    }
    else if (power < -Constants.CHASSIS_MAX_POWER)
    {
      power = -Constants.CHASSIS_MAX_POWER;
    }
    
    m_differentialDrive.arcadeDrive(power, turn, true);
  }

public void convertSwerveValues (double x1, double y1, double x2)
  {
      // Width and length of robot
      double w = 25;
      double l = 17;

      // Width and length relative ratios
      double wR;
      double lR;

      // Input velocities and turn
      double vX = 0;
      double vY = 0;
      double turn = 0;

      //implificaton for adding turn and strafe velocity for each wheel
      double a;
      double b;
      double c;
      double d;

      // Apply deadzone to turn analog stick
      if (Math.abs(x2) > 0.2)
          turn = x2;
      
      // Apply dead zone for velocities
      if (Math.abs(x1) > 0.2)
          vX = x1;
      if (Math.abs(y1) > 0.2)
          vY = -y1;

      // Find similar triangles to chassis for turn vectors (radius = 1)
      double turnAngle = Math.atan2(l, w);
      wR = Math.cos(turnAngle);
      lR = Math.sin(turnAngle);

      // Establishing swerve gyro difference
      double gyroCurrent = m_ahrs.getYaw();

      // Adjust strafe vector so that forward constant
      double r = Math.sqrt(vX * vX + vY * vY);
      double strafeAngle = Math.atan2(vY, vX);

      strafeAngle += gyroCurrent / 360 * 2 * Math.PI;
      vX = r * Math.cos(strafeAngle);
      vY = r * Math.sin(strafeAngle);

      // Shortening equations for adding strafe and turn for each wheel
      a = vX - turn * lR;
      b = vX + turn * lR;
      c = vY - turn * wR;
      d = vY + turn * wR;

      // Finding speed of each wheel based on x and y velocities
      frontRight[0] = Math.sqrt(Math.abs(b * b + c * c));
      frontLeft[0] = Math.sqrt(Math.abs(b * b + d * d));
      backLeft[0] = Math.sqrt(Math.abs(a * a + d * d));
      backRight[0] = Math.sqrt(Math.abs(a * a + c * c));

      // Adjust for exceeding max speed
      double highestSpeed = Math.max(Math.max(Math.max(frontRight[0], frontLeft[0]), backLeft[0]), backRight[0]);
      if (highestSpeed > 1) {
          frontRight[0] = frontRight[0] / highestSpeed;
          frontLeft[0] = frontLeft[0] / highestSpeed;
          backLeft[0] = backLeft[0] / highestSpeed;
          backRight[0] = backRight[0] / highestSpeed;
      }

      // Update last angle
      frontRight[2] = frontRight[1];
      frontLeft[2] = frontLeft[1];
      backLeft[2] = backLeft[1];
      backRight[2] = backLeft[1];

      // Set new angles
      if (!(vX == 0 && vY == 0 && turn == 0)) {
          // Find angle of each wheel based on velocities
          frontRight[1] = Math.atan2(c, b) - Math.PI / 2;
          frontLeft[1] = Math.atan2(d, b) - Math.PI / 2;
          backLeft[1] = Math.atan2(d, a) - Math.PI / 2;
          backRight[1] = Math.atan2(c, a) - Math.PI / 2;
      }

      // When a wheel moves more than half a circle in one direction, offsets so it
      // goes the shorter route
      if ((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1]))
          frontRight[3] -= 2 * Math.PI;
      if ((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] > frontRight[1]))
          frontRight[3] += 2 * Math.PI;
      if ((Math.abs(frontLeft[2] - frontLeft[1]) > Math.PI && frontLeft[2] < frontLeft[1]))
          frontLeft[3] -= 2 * Math.PI;
      if ((Math.abs(frontLeft[2] - frontLeft[1]) > Math.PI && frontLeft[2] > frontLeft[1]))
          frontLeft[3] += 2 * Math.PI;

      if ((Math.abs(backLeft[2] - backLeft[1]) > Math.PI && backLeft[2] < backLeft[1]))
          backLeft[3] -= 2 * Math.PI;
      if ((Math.abs(backLeft[2] - backLeft[1]) > Math.PI && backLeft[2] > backLeft[1]))
          backLeft[3] += 2 * Math.PI;
      if ((Math.abs(backRight[2] - backRight[1]) > Math.PI && backRight[2] < backRight[1]))
          backRight[3] -= 2 * Math.PI;
      if ((Math.abs(backRight[2] - backRight[1]) > Math.PI && backRight[2] > backRight[1]))
          backRight[3] += 2 * Math.PI;

      drive(m_speedMotorFrontRight, m_angleMotorFrontRight, frontRight[0],
              -(frontRight[1] + frontRight[3]) / (Math.PI * 2) * Constants.SWERVE_TPR);
      drive(m_speedMotorFrontLeft, m_angleMotorFrontLeft, frontLeft[0],
              -(frontLeft[1] + frontLeft[3]) / (Math.PI * 2) * Constants.SWERVE_TPR);
      drive(m_speedMotorBackLeft, m_angleMotorBackLeft, backLeft[0],
              -(backLeft[1] + backLeft[3]) / (Math.PI * 2) * Constants.SWERVE_TPR);
      drive(m_speedMotorBackRight, m_angleMotorBackRight, backRight[0],
              -(backRight[1] + backRight[3]) / (Math.PI * 2) * Constants.SWERVE_TPR);

      SmartDashboard.putNumber("Current Angle Back Right", backRight[1] + backRight[3]);  
      SmartDashboard.putNumber("Current Angle Back Left", backLeft[1] + backLeft[3]);  
      SmartDashboard.putNumber("Current Angle Front Right", frontRight [1] + frontRight[3]);  
      SmartDashboard.putNumber("Current Angle Front Left", frontLeft[1] + frontLeft[3]);  
  
      SmartDashboard.putNumber("Speed Back Right", backRight[0]);
      SmartDashboard.putNumber("Speed Back Left", backLeft[0]);
      SmartDashboard.putNumber("Speed Front Right", frontRight[0]);
      SmartDashboard.putNumber("Speed Front Left", frontLeft[0]);
  
      SmartDashboard.putNumber("Swerve Yaw", m_ahrs.getYaw());
      SmartDashboard.putNumber("Swerve Compass", m_ahrs.getCompassHeading());
    }

  public void drive (WPI_TalonFX speedMotor, WPI_TalonFX angleMotor, double speed, double angle)
  {
    speedMotor.set(speed *0.7);

    double setpoint = angle * (Constants.SWERVE_DRIVE_MAX_VOLTAGE * 1.5);
    
    if (setpoint < 0) 
    {
      setpoint += Constants.SWERVE_DRIVE_MAX_VOLTAGE;
    }

    if (setpoint > Constants.SWERVE_DRIVE_MAX_VOLTAGE)
    {
      setpoint -= setpoint;
    }

    angleMotor.set(TalonFXControlMode.Position, angle* 0.8);

    System.out.println("Speed" + speed);
    System.out.println("Angle" + angle);
  }

  public double getAngleDifferenceWithLeftRight(double positiveAngle, double negativeAngle) {
    // Creating variables that will hold the difference between angles on the left and right sides of the spectrum.
    double leftDifference = 0;
    double rightDifference = 0;

    // These algorithms calculate the difference on the left and right side using
    // the positive and negative angle passed in.
    leftDifference = Math.abs(negativeAngle) + positiveAngle;
    rightDifference = (Math.PI - Math.abs(negativeAngle)) + (Math.PI - positiveAngle);

    // Return the fastest route (smaller difference) to reaching the desired angle from the gyro angle.
    // Depending on if gyro angle is positive/negative, difference may be positive/negative 
    // for motor ouput to be positive/negative.
    if (leftDifference < rightDifference) {
        return leftDifference;
    }
    else if (leftDifference > rightDifference) {
        return rightDifference;
    }
    else if (leftDifference == rightDifference)
        return Math.PI;
    else {
        System.out.println("\nERROR: Could not calculate angle difference. (left and right difference would not compute)\n");
        return 0;}
    }
  // public double getAngleDifference() {
  //   // Get gyro heading and desired heading from 0-360 degrees (x modulus 360 accomplishes this)
  //   double gyro_heading = this.getGyroAngle() % 360;
  //   double desired_heading = Pathfinder.r2d(left_follower.getHeading()) % 360;
  //   // Create variable that will hold the difference in angle
  //   double angleDifference = 0;

    // Change gyro heading and desired heading to a degree between -180 and 180
    // Example: 270 degrees is translated to -90 degrees
    // This allows angleDifference to be efficiently computed
//     if (gyro_heading > 180) {
//         gyro_heading = gyro_heading - 360;
//     } else if (gyro_heading < -180) {
//         gyro_heading = gyro_heading + 360;
//     }
//     if (desired_heading > 180) {
//         desired_heading = desired_heading - 360;
//     } else if (desired_heading < -180) {
//         desired_heading = desired_heading + 360;
//     }

//     // If path is reversed, robot will be facing in the opposite direction and 
//     // thus the gyro heading will be 180 degrees off --> this compensates for that
//     if (pathIsReversed)
//         gyro_heading -= 180;

//     // Calculate the angle difference keeping in mind the fastest route between angles
//     // Example: Gyro Angle: -179 degrees, Desired Angle: 179 degrees,
//     // set angle difference as -2 degrees, not 358 degrees.
//     if ((gyro_heading > 0 && desired_heading > 0) || (gyro_heading < 0 && desired_heading < 0))
//         angleDifference = desired_heading - gyro_heading;
//     else if (gyro_heading > 0 && desired_heading < 0)
//         angleDifference = getAngleDifferenceWithLeftRight(gyro_heading, desired_heading, true);
//     else if (gyro_heading < 0 && desired_heading > 0)
//         angleDifference = getAngleDifferenceWithLeftRight(desired_heading, gyro_heading, false);
//     else if ((gyro_heading == 0 && desired_heading > 0) || (gyro_heading == 0 && desired_heading < 0))
//         angleDifference = desired_heading;
//     else if ((gyro_heading > 0 && desired_heading == 0) || (gyro_heading < 0 && desired_heading == 0))
//         angleDifference = -gyro_heading;
//     else if (gyro_heading == desired_heading)
//         angleDifference = 0;
//     else
//         System.out.println("\nERROR: Could not calculate angle difference. (no criteria met)\n");

//     // Send angleDifference value back to caller
//     return angleDifference;
//   }

//   public double getAngleDifference() {
//     // Get gyro heading and desired heading from 0-360 degrees (x modulus 360 accomplishes this)
//     double gyro_heading = this.getGyroAngle() % 360;
//     double desired_heading = Pathfinder.r2d(left_follower.getHeading()) % 360;
//     // Create variable that will hold the difference in angle
//     double angleDifference = 0;

//     // Change gyro heading and desired heading to a degree between -180 and 180
//     // Example: 270 degrees is translated to -90 degrees
//     // This allows angleDifference to be efficiently computed
//     if (gyro_heading > 180) {
//         gyro_heading = gyro_heading - 360;
//     } else if (gyro_heading < -180) {
//         gyro_heading = gyro_heading + 360;
//     }
//     if (desired_heading > 180) {
//         desired_heading = desired_heading - 360;
//     } else if (desired_heading < -180) {
//         desired_heading = desired_heading + 360;
//     }

//     // If path is reversed, robot will be facing in the opposite direction and 
//     // thus the gyro heading will be 180 degrees off --> this compensates for that
//     if (pathIsReversed)
//         gyro_heading -= 180;

//     // Calculate the angle difference keeping in mind the fastest route between angles
//     // Example: Gyro Angle: -179 degrees, Desired Angle: 179 degrees,
//     // set angle difference as -2 degrees, not 358 degrees.
//     if ((gyro_heading > 0 && desired_heading > 0) || (gyro_heading < 0 && desired_heading < 0))
//         angleDifference = desired_heading - gyro_heading;
//     else if (gyro_heading > 0 && desired_heading < 0)
//         angleDifference = getAngleDifferenceWithLeftRight(gyro_heading, desired_heading);
//     else if (gyro_heading < 0 && desired_heading > 0)
//         angleDifference = getAngleDifferenceWithLeftRight(desired_heading, gyro_heading);
//     else if ((gyro_heading == 0 && desired_heading > 0) || (gyro_heading == 0 && desired_heading < 0))
//         angleDifference = desired_heading;
//     else if ((gyro_heading > 0 && desired_heading == 0) || (gyro_heading < 0 && desired_heading == 0))
//         angleDifference = -gyro_heading;
//     else if (gyro_heading == desired_heading)
//         angleDifference = 0;
//     else
//         System.out.println("\nERROR: Could not calculate angle difference. (no criteria met)\n");

//     // Send angleDifference value back to caller
//     return angleDifference;
// }

// }

  /**
   * Controls movement of robot drivetrain with passed in power and turn values
   * from autonomous input. Example: vision control.
   * Difference from teleopDrive is there's no deadband.
   */
  public void autoDrive(double power, double turn)
  {
    m_differentialDrive.arcadeDrive(-power, turn, false);
  }

  /**
   * Methods to control gearbox shifter.
   */
  public void shiftLowGear()
  {
    m_gearShift.set(Value.kReverse);
  }
  public void shiftHighGear()
  {
    m_gearShift.set(Value.kForward);
  }

  /**
   * Reset left and right encoder positions.
   */
  public void resetEncoders()
  {
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
  }

  /**
   * Get heading of the robot (no domain).
   * @return the angle of the gyro in degrees.
   */
  public double getAngle()
  {
    return m_ahrs.getAngle();
  }

  /**
   * Reset gyro to zero the heading of the robot.
   */
  public void zeroHeading()
  {
    m_ahrs.reset();
    m_ahrs.setAngleAdjustment(0.0);
  }

  /**
   * Set gyro to a certain heading.
   */
  public void setHeading(double heading)
  {
    m_ahrs.setAngleAdjustment(heading);
  }

  /**
   * Methods for path following.
   */

  /**
   * Get the distance the left and right sides of the robot have driven with encoder feedback.
   * Convert position (units) to distance (meters).
   * @return the distance travelled of the specified drive train side.
   */
  public double getLeftEncoderDistance()
  {
    return m_rightMaster.getSelectedSensorPosition() * Constants.K_ENCODER_DISTANCE_PER_PULSE;
  }
  public double getRightEncoderDistance()
  {
    return -m_leftMaster.getSelectedSensorPosition() * Constants.K_ENCODER_DISTANCE_PER_PULSE;
  }

  /**
   * Get rate of left and right encoders in distance (meters) per second.
   * Convert velocity (units/100ms) to rate (m/s).
   * @return the current rate of the encoder.
   */
  public double getLeftEncoderRate()
  {
    return m_rightMaster.getSelectedSensorVelocity() * Constants.K_ENCODER_DISTANCE_PER_PULSE * 1000;
  }
  public double getRightEncoderRate()
  {
    return -m_leftMaster.getSelectedSensorVelocity() * Constants.K_ENCODER_DISTANCE_PER_PULSE * 1000;
  }

  /**
   * Get current wheel speeds of the robot based on encoder feedback.
   * @return the current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(this.getLeftEncoderRate(), this.getRightEncoderRate());
  }

  /**
   * Get gyro heading between -180 to 180.
   * Uses Math.IEEEremainder to get range of -180 to 180 --> dividend - (divisor * Math.Round(dividend / divisor)).
   * @return the robot's heading in degrees.
   */
  public double getHeading()
  {
    return Math.IEEEremainder(m_ahrs.getAngle(), 360) * (Constants.K_GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Updates the odometry with current gyro angle and encoder distances.
   */
  public void updateOdometry()
  {
    m_odometry.update(Rotation2d.fromDegrees(this.getHeading()), this.getLeftEncoderDistance(), this.getRightEncoderDistance());
  }

  /**
   * Get an estimation for the current pose of the robot.
   * @return the pose in meters.
   */
  public Pose2d getPose()
  {
    return m_odometry.getPoseMeters();
  }



  /**
   * Controls the left and right sides of the drive train directly with voltages.
   * Use setVoltage() rather than set() as it will compensate for battery "voltage sag," required for accuracy.
   * @param leftVoltage  the commanded left voltage output.
   * @param rightVoltage the commanded right voltage output.
   */
  public void driveWithVoltage(double leftVoltage, double rightVoltage)
  {
    m_leftMaster.setVoltage(-rightVoltage); // negative
    m_rightMaster.setVoltage(leftVoltage); // positive because right side is inverted for the arcadeDrive method.
    m_differentialDrive.feed();
  }

  /**
   * Generates ramsete command for following passed in path in autonomous.
   * @param startingPose is the position at which the robot starts up at.
   * @param waypoints are the points in which the robot travels through to arrive at its end point.
   * @param endingPose is the position at which the robot ends up at.
   * @param maxVelocity controls how fast the robot will move through the trajectory/path.
   * @param isReversed controls whether the robot travels forwards or backwards through the waypoints.
   * @return sequential command group that follows the path and stops when complete.
   */
  public SequentialCommandGroup generateRamsete(Pose2d startingPose, List<Translation2d> waypoints, Pose2d endingPose, double maxVelocity, boolean isReversed)
  {
    /* Voltage constraint so never telling robot to move faster than it is capable of achieving. */
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.K_S_VOLTS,
                                   Constants.K_V_VOLT_SECONDS_PER_METER,
                                   Constants.K_A_VOLT_SECONDS_SQUARED_PER_METER), 
        Constants.K_DRIVE_KINEMATICS, 
        10);
    
    /* Configuration for trajectory that wraps path constraints. */
    TrajectoryConfig trajConfig =
      new TrajectoryConfig(maxVelocity,
                           Constants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          /* Add kinematics to track robot speed and ensure max speed is obeyed. */
          .setKinematics(Constants.K_DRIVE_KINEMATICS)
          /* Apply voltage constraint created above. */
          .addConstraint(autoVoltageConstraint)
          /* Reverse the trajectory based on passed in parameter. */
          .setReversed(isReversed);

    /* Generate trajectory: initialPose, interiorWaypoints, endPose, trajConfig */
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      // Starting pose
      startingPose,
      // Pass through these interior waypoints
      waypoints,
      // Ending pose
      endingPose,
      // Pass config
      trajConfig
    );

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
    return ramseteCommand.andThen(new InstantCommand(() -> RobotContainer.m_chassis.driveWithVoltage(0, 0)));
  }
}