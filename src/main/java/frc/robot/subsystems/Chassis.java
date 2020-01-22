package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {

  /**
   * Declaring objects for the drivetrain
   */
  private WPI_TalonFX m_left_front;
  private WPI_TalonFX m_left_back;
  private SpeedControllerGroup m_left;

  private WPI_TalonFX m_right_front;
  private WPI_TalonFX m_right_back;
  private SpeedControllerGroup m_right;

  private DifferentialDrive m_differentialDrive;

  private static AHRS m_ahrs;

  private Compressor m_compressor;

  private DoubleSolenoid m_gearShift;

  public Chassis() {
    
    /**
     * Instantiating drivetrain objects and setting attributes (inverted/sensor phase)
     */
    m_left_front = new WPI_TalonFX(Constants.CHASSIS_LEFT_FRONT_ID);
    m_left_front.setInverted(false);
    m_left_front.setSensorPhase(false);

    m_left_back = new WPI_TalonFX(Constants.CHASSIS_LEFT_BACK_ID);
    m_left_back.setInverted(false);
    m_left_back.setSensorPhase(false);

    m_left = new SpeedControllerGroup(m_left_front, m_left_back);

    m_right_front = new WPI_TalonFX(Constants.CHASSIS_RIGHT_FRONT_ID);
    m_right_front.setInverted(false);
    m_right_front.setSensorPhase(false);

    m_right_back = new WPI_TalonFX(Constants.CHASSIS_RIGHT_BACK_ID);
    m_right_back.setInverted(false);
    m_right_back.setSensorPhase(false);

    m_right = new SpeedControllerGroup(m_right_front, m_right_back);

    m_differentialDrive = new DifferentialDrive(m_left, m_right);

    /**
     * Try to instantiate the navx gyro with exception catch
     */
    try {
      m_ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
        System.out.println("\nError instantiating navX-MXP:\n" + ex.getMessage() + "\n");
    }

    /**
     * Pneumatics objects
     */
    m_compressor = new Compressor();

    m_gearShift = new DoubleSolenoid(Constants.CHASSIS_GEARSHIFT_PORT_A, Constants.CHASSIS_GEARSHIFT_PORT_B);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Controls movement of robot drivetrain with passed in power and turn values.
   * Allows external commands to control the private differentialDrive object.
   * Can be used for manual and autonomous input.
   */
  public void drive(double power, double turn) {
    m_differentialDrive.arcadeDrive(power, turn);
  }

}