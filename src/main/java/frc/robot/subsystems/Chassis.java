package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
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


  /**
   * Declaring objects for autonomous path following.
   */
  private final DifferentialDriveOdometry m_odometry;

  private Transform2d m_trajTransform;

  /* HashMap for trajectories serves as an array with referable strings. */
  private HashMap<String, Trajectory> m_trajs;

  /* These variables will be used in allowing the robot to go forwards AND backwards with trajectories. */
  private boolean m_isReversed;

  /* Left/right sides for accumulated distance relative to left and right side of field. */
  private double m_leftAccumDist;
  private double m_rightAccumDist;

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

    m_trajTransform = null;
    
    m_trajs = new HashMap<String, Trajectory>();

    m_isReversed = false;

    m_leftAccumDist = 0.0;
    m_rightAccumDist = 0.0;

    /**
     * Various methods to call when chassis subsystem first starts up.
     */

    /* Reset encoders & gyro to ensure autonomous path following is correct. */
    this.resetEncoders();
    this.zeroHeading();
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
    this.displayDrivetrainInfo();
  }

  /**
   * Puts various drivetrain parameters on the SmartDashboard for testing.
   */
  private void displayDrivetrainInfo()
  {
    SmartDashboard.putNumber("LEFT ENCODER", m_rightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("RIGHT ENCODER", -m_leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("LEFT VELOCITY", m_rightMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RIGHT VELOCITY", -m_leftMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("LEFT DISTANCE", this.getLeftEncoderDistance());
    SmartDashboard.putNumber("RIGHT DISTANCE", this.getRightEncoderDistance());
    SmartDashboard.putNumber("LEFT ACCUM DIST", m_leftAccumDist);
    SmartDashboard.putNumber("RIGHT ACCUM DIST", m_rightAccumDist);
    SmartDashboard.putNumber("LEFT TOTAL DISTANCE", this.getLeftTotalDistance());
    SmartDashboard.putNumber("RIGHT TOTAL DISTANCE", this.getRightTotalDistance());
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

    m_differentialDrive.arcadeDrive(power, turn, true);
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
   * Read all paths' json files generated by PathWeaver and store them in HashMap 
   * for later reference. Without doing this on startup, autonomous commands will 
   * be delayed as they have to spend time reading through all the data points in 
   * the json file. 
   */
  public void readTrajectories()
  {
    for (int i = 0; i < Constants.CHASSIS_TRAJECTORIES_NAMES.length; i++)
    {
      String pathName = Constants.CHASSIS_TRAJECTORIES_NAMES[i];
      String trajectoryJSONFilePath = "paths/" + pathName + ".wpilib.json";
      try
      {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONFilePath);
        m_trajs.put(pathName, TrajectoryUtil.fromPathweaverJson(trajectoryPath));

        System.out.println("\nPath being read: " + pathName + "\n");
      } catch (IOException ex)
      {
        System.out.println("\nUnable to open trajectory: " + trajectoryJSONFilePath + "\n" + ex.getStackTrace() + "\n");
      }
    }
  }

  /**
   * Get the distance the left and right sides of the robot have driven with encoder feedback.
   * Convert position (units) to distance (meters).
   * @return the distance travelled of the specified drive train side.
   */
  public double getLeftEncoderDistance()
  {
    if (m_isReversed)
    {
      return m_leftMaster.getSelectedSensorPosition() * Constants.K_ENCODER_DISTANCE_PER_PULSE;
    } else if (!m_isReversed)
    {
      return m_rightMaster.getSelectedSensorPosition() * Constants.K_ENCODER_DISTANCE_PER_PULSE;
    } else
    {
      System.out.println("\nThere was an error trying to get the left encoder distance.");
      return 0.0;
    }
  }
  public double getRightEncoderDistance()
  {
    if (m_isReversed)
    {
      return -m_rightMaster.getSelectedSensorPosition() * Constants.K_ENCODER_DISTANCE_PER_PULSE;
    } else if (!m_isReversed)
    {
      return -m_leftMaster.getSelectedSensorPosition() * Constants.K_ENCODER_DISTANCE_PER_PULSE;
    } else
    {
      System.out.println("\nThere was an error trying to get the right encoder distance.");
      return 0.0;
    }
  }

  /**
   * Add the accumulated distances to the current encoder positions to find total distance traveled.
   * This total distance is required for reversing the drive train.
   * @return the total distance traveled by the specified drive train side.
   */
  public double getLeftTotalDistance()
  {
    if (m_isReversed)
    {
      return m_rightAccumDist + this.getLeftEncoderDistance();
    } else if (!m_isReversed)
    {
      return m_leftAccumDist + this.getLeftEncoderDistance();
    } else
    {
      System.out.println("\nThere was an error when trying to get the left total distance.");
      return 0.0;
    }
  }
  public double getRightTotalDistance()
  {
    if (m_isReversed)
    {
      return m_leftAccumDist + this.getRightEncoderDistance();
    } else if (!m_isReversed)
    {
      return m_rightAccumDist + this.getRightEncoderDistance();
    } else
    {
      System.out.println("\nThere was an error when trying to get the right total distance.");
      return 0.0;
    }
  }

  /**
   * Get rate of left and right encoders in distance (meters) per second.
   * Convert velocity (units/100ms) to rate (m/s).
   * @return the current rate of the encoder.
   */
  public double getLeftEncoderRate()
  {
    if (m_isReversed)
    {
      return m_leftMaster.getSelectedSensorVelocity() * Constants.K_ENCODER_DISTANCE_PER_PULSE * 1000;
    } else if (!m_isReversed)
    {
      return m_rightMaster.getSelectedSensorVelocity() * Constants.K_ENCODER_DISTANCE_PER_PULSE * 1000;
    } else
    {
      System.out.println("\nThere was an error trying to get the velocity of the left side.\n");
      return 0.0;
    }
  }
  public double getRightEncoderRate()
  {
    if (m_isReversed)
    {
      return -m_rightMaster.getSelectedSensorVelocity() * Constants.K_ENCODER_DISTANCE_PER_PULSE * 1000;
    } else if (!m_isReversed)
    {
      return -m_leftMaster.getSelectedSensorVelocity() * Constants.K_ENCODER_DISTANCE_PER_PULSE * 1000;
    } else
    {
      System.out.println("\nThere was an error trying to get the velocity of the left side.\n");
      return 0.0;
    }
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
    m_odometry.update(Rotation2d.fromDegrees(this.getHeading()), this.getLeftTotalDistance(), this.getRightTotalDistance());
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
    if (m_isReversed)
    {
      m_leftMaster.setVoltage(leftVoltage); // positive
      m_rightMaster.setVoltage(-rightVoltage); // negative because right side is inverted for the arcadeDrive method.
    } else if (!m_isReversed)
    {
      m_leftMaster.setVoltage(-rightVoltage); // negative
      m_rightMaster.setVoltage(leftVoltage); // positive because right side is inverted for the arcadeDrive method.
    } else
    {
      System.out.println("\nThere was an error setting the drive motors' voltages.\n");
      m_leftMaster.setVoltage(0.0);
      m_rightMaster.setVoltage(0.0);
    }
    m_differentialDrive.feed();
  }

  /**
   * Performs actions that will allow the robot to follow trajectories in the opposite direction.
   */
  public void setReversed(boolean reverse)
  {
    /* Reverse drive train for forwards/backwards trajectories if passed in reverse is not equal to m_isReversed. */
    if (m_isReversed != reverse)
    {
      /* Add current position to accumulated distance. Then, reset encoders to go in opposite direction. */
      if (m_isReversed)
      {
        m_rightAccumDist += this.getLeftEncoderDistance();
        m_leftAccumDist += this.getRightEncoderDistance();
      } else if (!m_isReversed)
      {
        m_leftAccumDist += this.getLeftEncoderDistance();
        m_rightAccumDist += this.getRightEncoderDistance();
      }
      this.resetEncoders();

      /* Reverse gyro angle -- change by 180 degrees. */
      double tempHeading = this.getHeading();
      this.zeroHeading();
      this.setHeading(tempHeading - 180);

      m_isReversed = reverse;
    }
  }

  /**
   * Creates a zeroed pose object to transform trajectory relative to robot's initial position.
   * Transform is saved in this subsystem class as a variable for other trajectories to reference.
   */
  public void initializeTrajTransform(Trajectory initialTraj)
  {
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
    Pose2d zeroedPose = odometry.getPoseMeters();
    m_trajTransform = zeroedPose.minus(initialTraj.getInitialPose());
  }

  /**
   * Generates ramsete command for following passed in path in autonomous.
   * @param pathName is the path file generated from PathWeaver.
   * @param isFirstPath is to set the transform for all trajectories if is first path.
   * @return sequential command group that follows the path and stops when complete.
   */
  public SequentialCommandGroup generateRamsete(String pathName, boolean isFirstPath)
  {
    /* Get specified path/trajectory from m_trajs HashMap. */
    Trajectory trajectory = m_trajs.get(pathName);

    /**
     * Make trajectory relative to robot rather than relative to field. 
     * If is the first path being followed, initialize the trajectory transform.
     * Transform trajectory based on the first path's initial position.
     */
    if (isFirstPath)
    {
      this.initializeTrajTransform(trajectory);
    }
    trajectory = trajectory.transformBy(m_trajTransform);

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