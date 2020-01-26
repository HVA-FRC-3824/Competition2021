package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase 
{
  // private WPI_TalonFX m_topWheel;
  // private WPI_TalonFX m_bottomWheel;
  
  private WPI_TalonSRX m_topWheel;
  private WPI_TalonSRX m_bottomWheel;

  private WPI_TalonSRX m_feeder;

  private WPI_TalonSRX m_pivot;

  private DigitalInput m_ballSwitch;

  public Launcher()
  {
    // m_topWheel = new WPI_TalonFX(Constants.LAUNCHER_WHEEL_TOP_ID);
    // Robot.configureTalonFX(m_topWheel, false, false, Constants.LAUNCHER_TOP_WHEEL_F, Constants.LAUNCHER_TOP_WHEEL_P,
    //                         Constants.LAUNCHER_TOP_WHEEL_I, Constants.LAUNCHER_TOP_WHEEL_D);

    // m_bottomWheel = new WPI_Talon(Constants.LAUNCHER_WHEEL_BOTTOM_ID);
    // Robot.configureTalonFX(m_bottomWheel, false, false, Constants.LAUNCHER_BOTTOM_WHEEL_F, Constants.LAUNCHER_BOTTOM_WHEEL_P,
    //                         Constants.LAUNCHER_BOTTOM_WHEEL_I, Constants.LAUNCHER_BOTTOM_WHEEL_D);
    
    m_topWheel = new WPI_TalonSRX(Constants.LAUNCHER_WHEEL_TOP_ID);
    Robot.configureTalonSRX(m_topWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, Constants.LAUNCHER_TOP_WHEEL_F,
                            Constants.LAUNCHER_TOP_WHEEL_P, Constants.LAUNCHER_TOP_WHEEL_I, Constants.LAUNCHER_TOP_WHEEL_D, 0, 0);
                            
    m_bottomWheel = new WPI_TalonSRX(Constants.LAUNCHER_WHEEL_BOTTOM_ID);
    Robot.configureTalonSRX(m_bottomWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, false, Constants.LAUNCHER_BOTTOM_WHEEL_F,
                            Constants.LAUNCHER_BOTTOM_WHEEL_P, Constants.LAUNCHER_BOTTOM_WHEEL_I, Constants.LAUNCHER_BOTTOM_WHEEL_D, 0, 0);

    m_feeder = new WPI_TalonSRX(Constants.LAUNCHER_FEEDER_ID);
    Robot.configureTalonSRX(m_feeder, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, Constants.LAUNCHER_FEEDER_F,
                            Constants.LAUNCHER_FEEDER_P,Constants.LAUNCHER_FEEDER_I, Constants.LAUNCHER_FEEDER_D,
                            Constants.LAUNCHER_FEEDER_CRUISEVELOCITY, Constants.LAUNCHER_FEEDER_ACCELERATION);

    m_pivot = new WPI_TalonSRX(Constants.LAUNCHER_PIVOT_ID);
    Robot.configureTalonSRX(m_pivot, true, FeedbackDevice.Analog, false, false, Constants.LAUNCHER_PIVOT_F,
                            Constants.LAUNCHER_PIVOT_P, Constants.LAUNCHER_PIVOT_I, Constants.LAUNCHER_PIVOT_D,
                            Constants.LAUNCHER_PIVOT_CRUISEVELOCITY, Constants.LAUNCHER_PIVOT_ACCELERATION);

    m_ballSwitch = new DigitalInput(Constants.LAUNCHER_BALL_SWITCH_PORT);
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
  // public WPI_TalonFX getTopWheelTalonFX()
  // {
  //     return m_topWheel;
  // }
  // public WPI_TalonFX getBottomWheelTalonFX()
  // {
  //     return m_bottomWheel;
  // }
  public WPI_TalonSRX getFeederTalonSRX()
  {
      return m_feeder;
  }
  public WPI_TalonSRX getPivotTalonSRX()
  {
      return m_pivot;
  }

  /**
   * Sets the power output of the top and bottom launcher wheels.
   * @param power is between -1.0 and 1.0.
   */
  public void setTopWheelPower(double power)
  {
    m_topWheel.set(ControlMode.PercentOutput, power);
  }
  public void setBottomWheelPower(double power)
  {
    m_bottomWheel.set(ControlMode.PercentOutput, power);
  }

  /**
   * Sets the RPM of the top and bottom launcher wheels.
   * @param rpm is converted to a velocity (units/100ms) for the launcher wheels PID to be set to.
   */
  public void setTopWheelRPM(int rpm)
  {
    m_topWheel.set(ControlMode.Velocity, Robot.convertRPMToVelocity(rpm));
  }
  public void setBottomWheelRPM(int rpm)
  {
    m_bottomWheel.set(ControlMode.Velocity, Robot.convertRPMToVelocity(rpm));
  }

  /**
   * Stops launcher wheels.
   */
  public void stopWheels()
  {
    m_topWheel.set(0.0);
    m_bottomWheel.set(0.0);
  }
}