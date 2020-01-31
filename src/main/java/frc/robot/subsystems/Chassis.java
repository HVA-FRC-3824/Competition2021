package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  public Chassis() 
  {
    /**
     * Instantiating drivetrain objects
     */
    m_leftMaster = new WPI_TalonFX(Constants.CHASSIS_LEFT_MASTER_ID);
    RobotContainer.configureTalonFX(m_leftMaster, false, false, Constants.CHASSIS_LEFT_MASTER_F, Constants.CHASSIS_LEFT_MASTER_P,
                                    Constants.CHASSIS_LEFT_MASTER_I, Constants.CHASSIS_LEFT_MASTER_D);

    m_leftSlave = new WPI_TalonFX(Constants.CHASSIS_LEFT_SLAVE_ID);
    RobotContainer.configureTalonFX(m_leftSlave, false, false, Constants.CHASSIS_LEFT_SLAVE_F, Constants.CHASSIS_LEFT_SLAVE_P,
                                    Constants.CHASSIS_LEFT_SLAVE_I, Constants.CHASSIS_LEFT_SLAVE_D);

    m_leftSlave.follow(m_leftMaster);

    m_rightMaster = new WPI_TalonFX(Constants.CHASSIS_RIGHT_MASTER_ID);
    RobotContainer.configureTalonFX(m_rightMaster, false, false, Constants.CHASSIS_RIGHT_MASTER_F, Constants.CHASSIS_RIGHT_MASTER_P,
                                    Constants.CHASSIS_RIGHT_MASTER_I, Constants.CHASSIS_RIGHT_MASTER_D);

    m_rightSlave = new WPI_TalonFX(Constants.CHASSIS_RIGHT_SLAVE_ID);
    RobotContainer.configureTalonFX(m_rightSlave, false, false, Constants.CHASSIS_RIGHT_SLAVE_F, Constants.CHASSIS_RIGHT_SLAVE_P,
                                    Constants.CHASSIS_RIGHT_SLAVE_I, Constants.CHASSIS_RIGHT_SLAVE_D);

    m_rightSlave.follow(m_rightMaster);

    m_differentialDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

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
  }

  /**
   * This method will be called once per scheduler run.
   */
  @Override
  public void periodic()
  {
  }

  public double getAngle()
  {
    return m_ahrs.getAngle();
  }
  /**
   * Controls movement of robot drivetrain with passed in power and turn values.
   * Allows external commands to control the private differentialDrive object.
   * Can be used for manual and autonomous input.
   */
  public void drive(double power, double turn)
  {
    m_differentialDrive.arcadeDrive(power, turn);
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

  /* Anthony works in the grocery store
  Savin' his pennies for someday
  Mama Leone left a note on the door
  She said
  "Sonny, move out to the country."
  Workin' too hard can give me
  A heart attack
  You oughta know by now
  Who needs a house out in Hackensack?
  Is that what you get with your money?
  It seems such a waste of time
  If that's what it's all about
  If that's movin' up then I'm movin' out
  Sergeant O'Leary is walkin' the beat
  At night he becomes a bartender
  He works at Mister Cacciatore's down
  On Sullivan Street
  Across from the medical center
  He's tradin' in his Chevy for a Cadillac
  You oughta know by now
  And if he can't drive
  With a broken back
  At least he can polish the fenders
  It seems such a waste of time
  If that's what it's all about mama,
  If that's movin' up then I'm movin' out
  I'm movin out!
  You should never argue with a crazy mind
  You oughta know by now
  You can pay Uncle Sam with overtime
  Is that all you get for your money?
  It seems such a waste of time
  If that's what it's all about
  If that's movin' up then I'm movin' out */
}