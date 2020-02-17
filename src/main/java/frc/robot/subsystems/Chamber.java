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
  private WPI_TalonSRX m_chamberElevatorFront;
  private WPI_TalonSRX m_chamberElevatorBack;
  private WPI_TalonSRX m_chamberBase;

  private Ultrasonic m_ballPos_entering;
  private Ultrasonic m_ballPos_exiting;

  public Chamber()
  {
    m_chamberElevatorFront = new WPI_TalonSRX(Constants.CHAMBER_ELEVATOR_FRONT_ID);
    RobotContainer.configureTalonSRX(m_chamberElevatorFront, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false,
                                    Constants.CHAMBER_ELEVATOR_FRONT_F, Constants.CHAMBER_ELEVATOR_FRONT_P, Constants.CHAMBER_ELEVATOR_FRONT_I, 
                                    Constants.CHAMBER_ELEVATOR_FRONT_D, Constants.CHAMBER_ELEVATOR_FRONT_CRUISEVELOCITY, 
                                    Constants.CHAMBER_ELEVATOR_FRONT_ACCELERATION);
    
    m_chamberElevatorBack = new WPI_TalonSRX(Constants.CHAMBER_ELEVATOR_BACK_ID);
    RobotContainer.configureTalonSRX(m_chamberElevatorBack, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false,
                                    Constants.CHAMBER_ELEVATOR_BACK_F, Constants.CHAMBER_ELEVATOR_BACK_P, Constants.CHAMBER_ELEVATOR_BACK_I, 
                                    Constants.CHAMBER_ELEVATOR_BACK_D, Constants.CHAMBER_ELEVATOR_BACK_CRUISEVELOCITY, 
                                    Constants.CHAMBER_ELEVATOR_BACK_ACCELERATION);

    m_chamberBase = new WPI_TalonSRX(Constants.CHAMBER_BASE_ID);
    RobotContainer.configureTalonSRX(m_chamberBase, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false,
                                    Constants.CHAMBER_BASE_F, Constants.CHAMBER_BASE_P, Constants.CHAMBER_BASE_I, 
                                    Constants.CHAMBER_BASE_D, Constants.CHAMBER_BASE_CRUISEVELOCITY, 
                                    Constants.CHAMBER_BASE_ACCELERATION);

    m_ballPos_entering = new Ultrasonic(Constants.CHAMBER_BALL_POS_ENTER_PORT_A, Constants.CHAMBER_BALL_POS_ENTER_PORT_B);
    m_ballPos_exiting = new Ultrasonic(Constants.CHAMBER_BALL_POS_EXIT_PORT_A, Constants.CHAMBER_BALL_POS_EXIT_PORT_B);
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
  public WPI_TalonSRX getChamberElevatorFrontTalonSRX()
  {
    return m_chamberElevatorFront;
  }
  public WPI_TalonSRX getChamberElevatorBackTalonSRX()
  {
    return m_chamberElevatorBack;
  }
  public WPI_TalonSRX getChamberBaseTalonSRX()
  {
    return m_chamberBase;
  }

  /**
   * Method to spin elevator chamber wheels with power.
   * @param power range is from 1.0 to -1.0
   */
  public void setChamberElevatorPower(double power)
  {
    m_chamberElevatorFront.set(ControlMode.PercentOutput, power);
    m_chamberElevatorBack.set(ControlMode.PercentOutput, power);
  }

  public void setChamberBasePower(double power)
  {
    m_chamberBase.set(ControlMode.PercentOutput, -power);
  }
 
  /**
   * Method to set elevator chamber wheels RPM with ControlMode.Velocity.
   * @param rpm is converted to units per 100 milliseconds for ControlMode.Velocity.
   */
  public void setChamberElevatorRMP(int rpm)
  {
    m_chamberElevatorFront.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm));
    m_chamberElevatorBack.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm));
  }

  /**
   * Method to move the chamber the specified distance from its current position.
   */
  public void stepChamberDistance(double distance)
  {
    double presentPositionFront = m_chamberElevatorFront.getSelectedSensorPosition(0);
    m_chamberElevatorFront.set(ControlMode.MotionMagic, presentPositionFront + distance);
    double presentPositionBack = m_chamberElevatorBack.getSelectedSensorPosition(0);
    m_chamberElevatorBack.set(ControlMode.MotionMagic, presentPositionBack + distance);
  }

  /**
   * Method to initialize the ultrasonics to start sensing distance.
   */
  public void startUltrasonics()
  {
    m_ballPos_entering.setAutomaticMode(true);
    m_ballPos_exiting.setAutomaticMode(true);
  }

  public double getEnteringRange()
  {
    return m_ballPos_entering.getRangeInches();
  }
  public double getExitingRange()
  {
    return m_ballPos_exiting.getRangeInches();
  }

  public double SensorDistance(int sensor)
  {
    Ultrasonic [] sensors = {m_ballPos_entering, m_ballPos_exiting};
    return sensors[sensor].getRangeInches();
  }
}