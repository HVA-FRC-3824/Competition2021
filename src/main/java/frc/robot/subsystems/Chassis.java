package frc.robot.subsystems;

import frc.robot.Constants;
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

  private WPI_TalonFX m_angleMotor;
  private WPI_TalonFX m_speedMotor;

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

    m_angleMotor = new WPI_TalonFX(Constants.WHEEL_DRIVE_ANGLE_MOTOR_ID);
    RobotContainer.configureTalonFX(m_angleMotor, false, false, 0.0, Constants.K_CHASSIS_ANGLE_P, 
    Constants.K_CHASSIS_ANGLE_I, Constants.K_CHASSIS_ANGLE_D);

    m_speedMotor = new WPI_TalonFX(Constants.WHEEL_DRIVE_SPEED_MOTOR_ID);
    RobotContainer.configureTalonFX(m_speedMotor, false, false, 0.0, 0.0, 0.0, 0.0);

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
      //width and length
      double w = 150;
      double l = 200;

      //width and length relative ratios
      double wr;
      double lr;

      //input velocities and turn
      double VX = 0;
      double VY = 0;
      double turn = 0;

      //turn velocities
      double tvx;
      double tvy;

      //simplificaton for adding turn and strafe velocity for each wheel
      double a;
      double b;
      double c;
      double d;

      //turn
      if (Math.abs(x2) > 0.015) {turn = x2;}
      //Input Velocity
      if (Math.abs(x1) > 0.15) {VX = x1;}
      if (Math.abs(y1) > 0.15) {VY = y1;}


      //ratio of width to length
      wr = w / Math.max(l, w) * 2;
      lr = l / Math.max(l, w) * 2;

      //turn velocities
      tvx = turn * lr / 2;
      tvy = turn * wr / 2;

      //calculation

      a = VX - turn * lr / 2;
      b = VX + turn * lr / 2;
      c = VY - turn * wr / 2;
      d = VY + turn * wr / 2;

      //output wheel velocities and angles
      //[vx, vy, speed, angle];
      double [] wheel_one = {b, c, 0, 0};
      double [] wheel_two = {b, d, 0, 0};
      double [] wheel_three = {a, d, 0, 0};
      double [] wheel_four = {a, c, 0, 0};

      // finding speed of each wheel based off their x and y velocities
      wheel_one[2] = Math.sqrt(Math.abs(b * b + c * c));
      wheel_two[2] = Math.sqrt(Math.abs(b * b + d * d));
      wheel_three[2] = Math.sqrt(Math.abs(a * a + d * d));
      wheel_four[2] = Math.sqrt(Math.abs(a * a + c * c));

      //adjust for overturning of wheels
      double highest_wheelspeed = Math.max(Math.max(Math.max(Math.abs(wheel_one[2]), 
                                  Math.abs(wheel_two[2])), Math.abs(wheel_three[2])), Math.abs(wheel_four[2]));
      if (highest_wheelspeed > 1) {
          wheel_one[2] = wheel_one[2] / highest_wheelspeed;
          wheel_two[2] = wheel_two[2] / highest_wheelspeed;
          wheel_three[2] = wheel_three[2] / highest_wheelspeed;
          wheel_four[2] = wheel_four[2] / highest_wheelspeed;
      }

      //finding angle of each wheel based off their velocities
      wheel_one[3] = Math.atan2(b, c) / (2 * Math.PI) * 6000;
      wheel_two[3] = Math.atan2(b, d) / (2 * Math.PI) * 6000;
      wheel_three[3] = Math.atan2(a, d) / (2 * Math.PI) * 6000;
      wheel_four[3] = Math.atan2(a, c) / (2 * Math.PI) * 6000;

      drive(wheel_one[2], wheel_one[3]);

      SmartDashboard.putNumber("Angle", wheel_one[3]);
      SmartDashboard.putNumber("Speed", wheel_one[2]);
  }

  public void drive (double speed, double angle)
  {
    m_speedMotor.set(speed);

    double setpoint = angle * (Constants.SWERVE_DRIVE_MAX_VOLTAGE * 0.5) + (Constants.SWERVE_DRIVE_MAX_VOLTAGE);
    
    if (setpoint < 0) 
    {
      setpoint = Constants.SWERVE_DRIVE_MAX_VOLTAGE + setpoint;
    }

    if (setpoint > Constants.SWERVE_DRIVE_MAX_VOLTAGE)
    {
      setpoint = setpoint - Constants.SWERVE_DRIVE_MAX_VOLTAGE;
    }

    m_angleMotor.set(TalonFXControlMode.Position, angle);

    System.out.println("Speed" + speed);
    System.out.println("Angle" + angle);
  }

  public void setPosition ()
  {
    m_angleMotor.set(TalonFXControlMode.Position, 12000);
  }

  public WPI_TalonFX getMotor ()
  {
    return m_angleMotor;
  }
  
  public void resetPosition ()
  {
    m_angleMotor.set(TalonFXControlMode.Position, 0);
  }

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