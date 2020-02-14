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
  private Ultrasonic m_ballPos_exiting;
  public Chamber()
  {
    m_chamberElevator = new WPI_TalonSRX(Constants.CHAMBER_ELEVATOR_ID);
    RobotContainer.configureTalonSRX(m_chamberElevator, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false,
                                    Constants.CHAMBER_ELEVATOR_F, Constants.CHAMBER_ELEVATOR_P, Constants.CHAMBER_ELEVATOR_I, 
                                    Constants.CHAMBER_ELEVATOR_D, Constants.CHAMBER_ELEVATOR_CRUISEVELOCITY, 
                                    Constants.CHAMBER_ELEVATOR_ACCELERATION);

    m_ballPos_entering = new Ultrasonic(Constants.CHAMBER_BALL_POS_1_PORT_B, Constants.CHAMBER_BALL_POS_1_PORT_A);
    m_ballPos_exiting = new Ultrasonic(Constants.CHAMBER_BALL_POS_2_PORT_B, Constants.CHAMBER_BALL_POS_2_PORT_A);
  }
  
  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("BALL ENTERING POS DISTANCE", this.getEnteringRange());
    SmartDashboard.putNumber("BALL EXITING POS DISTANCE", this.getExitingRange());
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

  /**
   * method to move the chamber the specified distance from its current position
   */
  public void stepChamberDistance(double distance)
  {
    double presentPosition = m_chamberElevator.getSelectedSensorPosition(0);
    m_chamberElevator.set(ControlMode.MotionMagic, presentPosition + distance);
  }

  public double getEnteringRange()
  {
    return m_ballPos_entering.getRangeInches();
  }
  public double getExitingRange()
  {
    return m_ballPos_exiting.getRangeInches();
  }

  public void startUltrasonic()
  {
    m_ballPos_entering.setAutomaticMode(true);
    m_ballPos_exiting.setAutomaticMode(true);
  }

  public double SensorDistance(int sensor)
  {
    Ultrasonic [] sensors = {m_ballPos_entering, m_ballPos_exiting};
    return sensors[sensor].getRangeInches();
  }
}