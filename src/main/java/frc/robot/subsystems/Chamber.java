package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chamber extends SubsystemBase
{
  private WPI_TalonSRX m_chamberElevator;

  private DigitalInput m_ballPos_1;
  private DigitalInput m_ballPos_2;
  private DigitalInput m_ballPos_3;
  private DigitalInput m_ballPos_4;

  public Chamber()
  {
    m_chamberElevator = new WPI_TalonSRX(Constants.CHAMBER_ELEVATOR_ID);
    RobotContainer.configureTalonSRX(m_chamberElevator, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false,
                            Constants.CHAMBER_ELEVATOR_F, Constants.CHAMBER_ELEVATOR_P, Constants.CHAMBER_ELEVATOR_I, 
                            Constants.CHAMBER_ELEVATOR_D, Constants.CHAMBER_ELEVATOR_CRUISEVELOCITY, 
                            Constants.CHAMBER_ELEVATOR_ACCELERATION);

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
  public WPI_TalonSRX getChamberElevatorTalonSRX()
  {
    return m_chamberElevator;
  }

  /**
   * Method to spin elevator chamber wheels with power.
   * @param power range is from 1.0 to -1.0
   */
  public void setChamberElevatorPower(double power)
  {
    m_chamberElevator.set(ControlMode.PercentOutput, power);
  }
 
  /**
   * Method to set elevator chamber wheels RPM with ControlMode.Velocity.
   * @param rpm is converted to units per 100 milliseconds for ControlMode.Velocity.
   */
  public void setChamberElevatorRMP(int rpm)
  {
    m_chamberElevator.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm));
  }
}