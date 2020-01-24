package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chamber extends SubsystemBase
{
  private WPI_TalonSRX m_chamberStart;
  private WPI_TalonSRX m_chamberEnd;

  private DigitalInput m_ballPos_1;
  private DigitalInput m_ballPos_2;
  private DigitalInput m_ballPos_3;
  private DigitalInput m_ballPos_4;

  public Chamber()
  {
    m_chamberStart = new WPI_TalonSRX(Constants.CHAMBER_START_ID);
    Robot.configureTalonSRX(m_chamberStart, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false,
                            Constants.CHAMBER_CHAMBER_START_F, Constants.CHAMBER_CHAMBER_START_P,
                            Constants.CHAMBER_CHAMBER_START_I, Constants.CHAMBER_CHAMBER_START_D, Constants.CHAMBER_CHAMBER_START_CRUISECONTROL,
                            Constants.CHAMBER_CHAMBER_START_ACCELERATION);
                            
    m_chamberEnd = new WPI_TalonSRX(Constants.CHAMBER_END_ID);
    Robot.configureTalonSRX(m_chamberEnd, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false,
                            Constants.CHAMBER_CHAMBER_END_F, Constants.CHAMBER_CHAMBER_END_P,
                            Constants.CHAMBER_CHAMBER_END_I, Constants.CHAMBER_CHAMBER_END_D, Constants.CHAMBER_CHAMBER_END_CRUISECONTROL,
                            Constants.CHAMBER_CHAMBER_END_ACCELERATION);
    
    m_ballPos_1 = new DigitalInput(Constants.CHAMBER_BALL_POS_1_PORT);
    m_ballPos_2 = new DigitalInput(Constants.CHAMBER_BALL_POS_2_PORT);
    m_ballPos_3 = new DigitalInput(Constants.CHAMBER_BALL_POS_3_PORT);
    m_ballPos_4 = new DigitalInput(Constants.CHAMBER_BALL_POS_4_PORT);
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
  }

  /**
   * Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  public WPI_TalonSRX getChamberStartTalonSRX()
  {
    return m_chamberStart;
  }
  public WPI_TalonSRX getChamberEndTalonSRX()
  {
    return m_chamberEnd;
  }
}