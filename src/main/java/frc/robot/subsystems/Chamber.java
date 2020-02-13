package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chamber extends SubsystemBase
{
  private WPI_TalonSRX m_chamberElevator;

  private Ultrasonic m_ballPos_entering;
  private Ultrasonic m_ballPos_middle;
  private Ultrasonic m_ballPos_exiting;
  public Chamber()
  {
    m_chamberElevator = new WPI_TalonSRX(Constants.CHAMBER_ELEVATOR_ID);
    RobotContainer.configureTalonSRX(m_chamberElevator, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false,
                                    Constants.CHAMBER_ELEVATOR_F, Constants.CHAMBER_ELEVATOR_P, Constants.CHAMBER_ELEVATOR_I, 
                                    Constants.CHAMBER_ELEVATOR_D, Constants.CHAMBER_ELEVATOR_CRUISEVELOCITY, 
                                    Constants.CHAMBER_ELEVATOR_ACCELERATION);

    m_ballPos_entering = new Ultrasonic(Constants.CHAMBER_BALL_POS_1_PORT_A, Constants.CHAMBER_BALL_POS_1_PORT_B);
    m_ballPos_middle = new Ultrasonic(Constants.CHAMBER_BALL_POS_2_PORT_A, Constants.CHAMBER_BALL_POS_2_PORT_B);
    m_ballPos_exiting = new Ultrasonic(Constants.CHAMBER_BALL_POS_3_PORT_A, Constants.CHAMBER_BALL_POS_3_PORT_B);
  }
  
  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("BALL POS 1 DISTANCE", this.getRange());
    SmartDashboard.putNumber("BALL POS 2 DISTANCE", this.getRange1());
    SmartDashboard.putNumber("BALL POS 3 DISTANCE", this.getRange2());
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

  public double getRange()
  {
    return m_ballPos_entering.getRangeInches();
  }
  public double getRange1()
  {
    return m_ballPos_middle.getRangeInches();
  }
  public double getRange2()
  {
    return m_ballPos_exiting.getRangeInches();
  }

  public void startUltrasonic()
  {
    m_ballPos_entering.setAutomaticMode(true);
    m_ballPos_middle.setAutomaticMode(true);
    m_ballPos_exiting.setAutomaticMode(true);
  }
}