package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

    m_gearShift = new DoubleSolenoid(Constants.CHASSIS_GEARSHIFT_PORT_B, Constants.CHASSIS_GEARSHIFT_PORT_A);

    /**
     * Autonomous path following objects
     */

    /* Reset encoders & gyro to ensure autonomous path following is correct. */
    this.resetEncoders();
    this.zeroHeading();

    /* Used for tracking robot pose. */
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeading()));
  }

  /**
   * This method will be called once per scheduler run.
   */
  @Override
  public void periodic()
  {
    this.updateOdometry();

    SmartDashboard.putNumber("LEFT ENCODER", m_leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("RIGHT ENCODER", -m_rightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("LEFT VELOCITY", m_leftMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RIGHT VELOCITY", -m_rightMaster.getSelectedSensorVelocity());
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
    /* Creates deadband for joystick twist input. */
    if (turn > -0.1 && turn < 0.1)
      turn = 0;

    m_differentialDrive.arcadeDrive(-power, turn, true);
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
   * Get heading of the robot (no domain).
   * @return the angle of the gyro in degrees.
   */
  public double getAngle()
  {
    return m_ahrs.getAngle();
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
   * Reset gyro to zero the heading of the robot.
   */
  public void zeroHeading()
  {
    m_ahrs.reset();
    m_ahrs.setAngleAdjustment(0.0);
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
    return m_leftMaster.getSelectedSensorPosition() * Constants.K_ENCODER_DISTANCE_PER_PULSE;
  }
  public double getRightEncoderDistance()
  {
    return -m_rightMaster.getSelectedSensorPosition() * Constants.K_ENCODER_DISTANCE_PER_PULSE;
  }

  /**
   * Get rate of left and right encoders in distance (meters) per second.
   * Convert velocity (units/100ms) to rate (m/s).
   * @return the current rate of the encoder.
   */
  public double getLeftEncoderRate()
  {
    return m_leftMaster.getSelectedSensorVelocity() * Constants.K_ENCODER_DISTANCE_PER_PULSE * 1000;
  }
  public double getRightEncoderRate()
  {
    return -m_rightMaster.getSelectedSensorVelocity() * Constants.K_ENCODER_DISTANCE_PER_PULSE * 1000;
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
   * Resets odometry for following new paths during one runtime.
   */
  public void resetOdometry()
  {
    this.zeroHeading();
    this.resetEncoders();
    Pose2d pose = new Pose2d();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(this.getHeading()));
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
    m_leftMaster.setVoltage(leftVoltage);
    m_rightMaster.setVoltage(-rightVoltage);
    m_differentialDrive.feed();
  }
}