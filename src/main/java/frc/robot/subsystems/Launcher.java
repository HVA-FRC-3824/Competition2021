package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase 
{
  private WPI_TalonFX m_topWheel;
  private WPI_TalonFX m_bottomWheel;
  
  // private WPI_TalonSRX m_topWheel;
  // private WPI_TalonSRX m_bottomWheel;

  private WPI_TalonSRX m_feeder;

  private WPI_TalonSRX m_pivot;

  private DigitalInput m_ballSwitch;
  
  /**
   * Displays current desired angle of pivot. 
   * Used for jogging up and down feature and displaying on SmartDashboard for operator.
   */
  private int pivotCurrentAngle = 0;

  public Launcher()
  {
    m_topWheel = new WPI_TalonFX(Constants.LAUNCHER_TOP_WHEEL_ID);
    RobotContainer.configureTalonFX(m_topWheel, false, false, Constants.LAUNCHER_TOP_WHEEL_F, Constants.LAUNCHER_TOP_WHEEL_P,
                            Constants.LAUNCHER_TOP_WHEEL_I, Constants.LAUNCHER_TOP_WHEEL_D);

    m_bottomWheel = new WPI_TalonFX(Constants.LAUNCHER_BOTTOM_WHEEL_ID);
    RobotContainer.configureTalonFX(m_bottomWheel, false, false, Constants.LAUNCHER_BOTTOM_WHEEL_F, Constants.LAUNCHER_BOTTOM_WHEEL_P,
                            Constants.LAUNCHER_BOTTOM_WHEEL_I, Constants.LAUNCHER_BOTTOM_WHEEL_D);
    
    // m_topWheel = new WPI_TalonSRX(Constants.LAUNCHER_TOP_WHEEL_ID);
    // RobotContainer.configureTalonSRX(m_topWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
    //                                 Constants.LAUNCHER_TOP_WHEEL_F, Constants.LAUNCHER_TOP_WHEEL_P, 
    //                                 Constants.LAUNCHER_TOP_WHEEL_I, Constants.LAUNCHER_TOP_WHEEL_D, 0, 0);
                            
    // m_bottomWheel = new WPI_TalonSRX(Constants.LAUNCHER_BOTTOM_WHEEL_ID);
    // RobotContainer.configureTalonSRX(m_bottomWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
    //                                 Constants.LAUNCHER_BOTTOM_WHEEL_F, Constants.LAUNCHER_BOTTOM_WHEEL_P, 
    //                                 Constants.LAUNCHER_BOTTOM_WHEEL_I, Constants.LAUNCHER_BOTTOM_WHEEL_D, 0, 0);

    m_feeder = new WPI_TalonSRX(Constants.LAUNCHER_FEEDER_ID);
    RobotContainer.configureTalonSRX(m_feeder, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                    Constants.LAUNCHER_FEEDER_F, Constants.LAUNCHER_FEEDER_P, Constants.LAUNCHER_FEEDER_I, 
                                    Constants.LAUNCHER_FEEDER_D, Constants.LAUNCHER_FEEDER_CRUISEVELOCITY, 
                                    Constants.LAUNCHER_FEEDER_ACCELERATION);

    m_pivot = new WPI_TalonSRX(Constants.LAUNCHER_PIVOT_ID);
    RobotContainer.configureTalonSRX(m_pivot, true, FeedbackDevice.Analog, false, false, Constants.LAUNCHER_PIVOT_F,
                                    Constants.LAUNCHER_PIVOT_P, Constants.LAUNCHER_PIVOT_I, Constants.LAUNCHER_PIVOT_D,
                                    Constants.LAUNCHER_PIVOT_CRUISEVELOCITY, Constants.LAUNCHER_PIVOT_ACCELERATION);

    m_ballSwitch = new DigitalInput(Constants.LAUNCHER_BALL_SWITCH_PORT);

    /**
     * Put pivot angle on SmartDashboard.
     * Hard to visually see on the field, so operator can use this value to measure.
     */
    SmartDashboard.putNumber("Pivot Angle", this.pivotCurrentAngle);
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
    m_topWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm));
  }
  public void setBottomWheelRPM(int rpm)
  {
    m_bottomWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm));
  }

  /**
   * Stops launcher wheels.
   */
  public void stopWheels()
  {
    m_topWheel.set(0.0);
    m_bottomWheel.set(0.0);
  }

  /**
   * Sets the desired position of the launcher pivot angle.
   * @param position will be used as the setpoint for the PID controller
   */
  public void setAngle(int angle)
  {
    /* Verify desired angle is within the minimum and maximum launcher angle range. */
    if (angle < Constants.LAUNCHER_PIVOT_MIN_ANGLE)
      angle = Constants.LAUNCHER_PIVOT_MIN_ANGLE;
    else if (angle > Constants.LAUNCHER_PIVOT_MAX_ANGLE)
      angle = Constants.LAUNCHER_PIVOT_MAX_ANGLE;

    /* Update current desired angle variable for other features like jogging up and down. */
    this.pivotCurrentAngle = angle;

    /* Update value on SmartDashboard. Hard to visually see angle on field, so operator can use this value to measure. */
    SmartDashboard.putNumber("Pivot Angle", this.pivotCurrentAngle);

    /**
     * Convert desired angle to encoder ticks for PID setpoint.
     * To do this, use pheonix software to get several points with x = angle and y = encoder ticks.
     * Compile points into a table and use linear regression (y=mx+b) to find an equation to convert angle (x) to encoder ticks (y).
     */
    int setpoint = angle; // TODO: Find equation and convert angle to setpoint.

    m_pivot.set(ControlMode.MotionMagic, setpoint);
  }

  /**
   * Returns the current angle.
   * Used for jogging the pivot angle up and down by taking the current angle and adding the jog magnitude.
   */
  public int getCurrentAngle()
  {
    return this.pivotCurrentAngle;
  }
  
  /**
   * This method is used to set the wheel RPMs and pivot angle for presets.
   */
  public void setPreset(int topRPM, int bottomRPM, int pivotAngle)
  {
    this.setTopWheelRPM(topRPM);
    this.setBottomWheelRPM(bottomRPM);
    this.setAngle(pivotAngle);
  }

  /**
   * Activates the feeder mechanism with a power output to the motor.
   * @param power range is from -1.0 to 1.0.
   */
  public void setFeederPower(double power)
  {
    m_feeder.set(ControlMode.PercentOutput, power);
  }

  /**
   * Activates the feeder mechanism with a velocity PID using RPM.
   * @param rpm is converted to a velocity (units/100ms) for a PID to use.
   */
  public void setFeederRPM(int rpm)
  {
    m_feeder.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm));
  }
}