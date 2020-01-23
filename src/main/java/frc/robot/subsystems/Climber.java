package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
  private WPI_TalonSRX m_stringPuller;

  private DoubleSolenoid m_PTO;

  private Servo m_ratchetLeft;
  private Servo m_ratchetRight;
  
  public Climber()
  {
    m_stringPuller = new WPI_TalonSRX(Constants.CLIMBER_STRING_PULLER_ID);

    m_PTO = new DoubleSolenoid(Constants.CLIMBER_PTO_PORT_A, Constants.CLIMBER_PTO_PORT_B);

    m_ratchetLeft = new Servo(Constants.CLIMBER_RATCHET_LEFT_PORT);
    m_ratchetRight = new Servo(Constants.CLIMBER_RATCHET_RIGHT_PORT);
  }

  /**
   * This method will be called once per scheduler run.
   */
  @Override
  public void periodic()
  {
  }

  /**
   * Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  public WPI_TalonSRX getStringPullerTalonSRX()
  {
      return m_stringPuller;
  }
}