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
  private WPI_TalonSRX m_base;
  private WPI_TalonSRX m_elevatorMaster;
  private WPI_TalonSRX m_elevatorSlave;

  private Ultrasonic m_ballPos_entering;
  private Ultrasonic m_ballPos_exiting;

  public Chamber()
  {
    m_base = new WPI_TalonSRX(Constants.CHAMBER_BASE_ID);
    RobotContainer.configureTalonSRX(m_base, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, true,
                                    Constants.CHAMBER_BASE_F, Constants.CHAMBER_BASE_P, Constants.CHAMBER_BASE_I, 
                                    Constants.CHAMBER_BASE_D, 0, 0, true);

    m_elevatorMaster = new WPI_TalonSRX(Constants.CHAMBER_ELEVATOR_MASTER_ID);
    RobotContainer.configureTalonSRX(m_elevatorMaster, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, true,
                                    Constants.CHAMBER_ELEVATOR_F, Constants.CHAMBER_ELEVATOR_P, Constants.CHAMBER_ELEVATOR_I, 
                                    Constants.CHAMBER_ELEVATOR_D, Constants.CHAMBER_ELEVATOR_CRUISEVELOCITY, 
                                    Constants.CHAMBER_ELEVATOR_ACCELERATION, true);

    m_elevatorSlave = new WPI_TalonSRX(Constants.CHAMBER_ELEVATOR_SLAVE_ID);
    RobotContainer.configureTalonSRX(m_elevatorSlave, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false,
                                    0.0, 0.0, 0.0, 0.0, 0, 0, true);

    m_elevatorSlave.follow(m_elevatorMaster);

    m_ballPos_entering = new Ultrasonic(Constants.CHAMBER_BALL_POS_ENTER_PORT_A, Constants.CHAMBER_BALL_POS_ENTER_PORT_B);
    m_ballPos_exiting = new Ultrasonic(Constants.CHAMBER_BALL_POS_EXIT_PORT_A, Constants.CHAMBER_BALL_POS_EXIT_PORT_B);
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
  public WPI_TalonSRX getBaseTalonSRX()
  {
    return m_base;
  }
  public WPI_TalonSRX getElevatorMasterTalonSRX()
  {
    return m_elevatorMaster;
  }
  public WPI_TalonSRX getElevatorSlaveTalonSRX()
  {
    return m_elevatorSlave;
  }

  /**
   * Method to spin base/elevator chamber belts with power.
   * @param power range is from 1.0 to -1.0
   */
  public void setBasePower(double power)
  {
    m_base.set(ControlMode.PercentOutput, power);
  }
  public void setElevatorPower(double power)
  {
    m_elevatorMaster.set(ControlMode.PercentOutput, power);
  }
 
  /**
   * Method to set base chamber belts RPM with ControlMode.Velocity.
   * @param rpm is converted to units per 100 milliseconds for ControlMode.Velocity.
   */
  public void setBaseRPM(int rpm)
  {
    m_base.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.CHAMBER_BASE_TPR));
  }

  /**
   * Method to set elevator chamber belts position with ControlMode.MotionMagic.
   * @param position is used as a setpoint for the PID controller.
   */
  public void setElevatorPosition(int position)
  {
    m_elevatorMaster.set(ControlMode.MotionMagic, position);
  }

  /**
   * Method to move the chamber the specified distance from its current position.
   */
  public void stepChamberDistance(double distance)
  {
    double presentPosition = m_elevatorMaster.getSelectedSensorPosition();
    m_elevatorMaster.set(ControlMode.MotionMagic, presentPosition + distance);
  }

  /**
   * Method to initialize the ultrasonics to start sensing distance.
   */
  public void startUltrasonics()
  {
    m_ballPos_entering.setAutomaticMode(true);
    m_ballPos_exiting.setAutomaticMode(true);
  }

  /**
   * Methods to get ultrasonic readings to track if a ball is present or not.
   * @return the distance the ultrasonic is reading.
   */
  public double getEnteringRange()
  {
    return m_ballPos_entering.getRangeInches();
  }
  public double getExitingRange()
  {
    return m_ballPos_exiting.getRangeInches();
  }

  /**
   * Gets the distance reading of a specified ultrasonic.
   * @param sensor is the index of the ultrasonic in the sensors array to read the distance from.
   * @return the distance of the specified ultrasonic.
   */
  public double SensorDistance(int sensor)
  {
    Ultrasonic [] sensors = {m_ballPos_entering, m_ballPos_exiting};
    return sensors[sensor].getRangeInches();
  }
}