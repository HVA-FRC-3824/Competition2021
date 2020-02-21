package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase 
{
  private WPI_TalonFX m_topWheel;
  private WPI_TalonFX m_bottomWheel;

  private WPI_TalonSRX m_pivot;
  private AnalogInput m_pivotFeedback;

  /**
   * Used to track current desired angle of pivot. 
   * Tracking for jogging up and down feature.
   */
  private int currentAngle;

  public Launcher()
  {
    m_topWheel = new WPI_TalonFX(Constants.LAUNCHER_TOP_WHEEL_ID);
    RobotContainer.configureTalonFX(m_topWheel, false, false, Constants.LAUNCHER_TOP_WHEEL_F, Constants.LAUNCHER_TOP_WHEEL_P,
                            Constants.LAUNCHER_TOP_WHEEL_I, Constants.LAUNCHER_TOP_WHEEL_D);

    m_bottomWheel = new WPI_TalonFX(Constants.LAUNCHER_BOTTOM_WHEEL_ID);
    RobotContainer.configureTalonFX(m_bottomWheel, false, false, Constants.LAUNCHER_BOTTOM_WHEEL_F, Constants.LAUNCHER_BOTTOM_WHEEL_P,
                            Constants.LAUNCHER_BOTTOM_WHEEL_I, Constants.LAUNCHER_BOTTOM_WHEEL_D);

    m_pivot = new WPI_TalonSRX(Constants.LAUNCHER_PIVOT_ID);
    RobotContainer.configureTalonSRX(m_pivot, false, FeedbackDevice.Analog, false, true, 
                                    Constants.LAUNCHER_PIVOT_F, Constants.LAUNCHER_PIVOT_P, Constants.LAUNCHER_PIVOT_I, 
                                    Constants.LAUNCHER_PIVOT_D, 0, 0, false);

    m_pivotFeedback = new AnalogInput(Constants.LAUNCHER_PIVOT_FEEDBACK_PORT);

    /**
     * Because the analog position value of the pivot is absolute, current angle will not be zero on startup. To keep
     * this in mind, set currentAngle to -1.
     */
    currentAngle = -1;
    SmartDashboard.putData("Set Launcher 1", new InstantCommand(() -> this.setTopWheelRPM(2500)));
    SmartDashboard.putData("Set Launcher 2", new InstantCommand(() -> this.setTopWheelRPM(1000)));
  }

  /**
   * This method will be called once per scheduler run.
   */
  @Override
  public void periodic()
  {
    // SmartDashboard.putNumber("Pivot Angle Setpoint", currentAngle);
    // SmartDashboard.putNumber("Pivot Setpoint", m_pivot.getClosedLoopTarget());
    // SmartDashboard.putNumber("Pivot Position", m_pivot.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Pivot Error", m_pivot.getClosedLoopError());    
    // SmartDashboard.putNumber("Pivot Voltage", m_pivot.getMotorOutputVoltage());

    SmartDashboard.putNumber("Pivot Angle Encoder", this.getPivotADC());
    RobotContainer.displayTalonFXInfo(m_topWheel, "Top Wheel");
  }

  /**
   * Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  public WPI_TalonFX getTopWheelTalonFX()
  {
    return m_topWheel;
  }
  public WPI_TalonFX getBottomWheelTalonFX()
  {
    return m_bottomWheel;
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
   * Sets power of linear actuator for launcher pivot angle.
   */
  public void setPivotPower(double power)
  {
    m_pivot.set(ControlMode.PercentOutput, -power);
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

    /**
     * Convert desired angle to encoder ticks for PID setpoint.
     * To do this, use pheonix software to get several points with x = angle and y = encoder ticks.
     * Compile points into a table and use linear regression (y=mx+b) to find an equation to convert angle (x) to encoder ticks (y).
     */
    int setpoint = angle; // TODO: Find equation and convert angle to setpoint.

    /* Angle error is how off the robot is from the target in the "y" direction. */
    int angleError = setpoint - this.getPivotADC();
    double pivotOutput = 0.0;

    /* Calculate output to launcher angle pivot based on offset from target. */
    if (angleError > 0.5)
    {
      pivotOutput = Constants.LAUNCHER_AIM_VISION_P * angleError - Constants.LAUNCHER_AIM_VISION_MIN;
    }
    else if (angleError < -0.5)
    {
      pivotOutput = Constants.LAUNCHER_AIM_VISION_P * angleError + Constants.LAUNCHER_AIM_VISION_MIN;
    }
    this.verifyPivotValue(pivotOutput);
  }
  
  /**
   * Gets current angle to step the pivot angle up and down from its current position.
   */
  public int getCurrentDesiredAngle()
  {
    return currentAngle;
  }

  /**
   * Get current ADC of pivot linear actuator to verify it is not overdriven.
   */
  private int getPivotADC()
  {
    return m_pivotFeedback.getValue();
  }

  /**
   * Method to prevent linear actuator from overdriving itself.
   * @return whether or not to allow pivot to be set.
   */
  public void verifyPivotValue(double output)
  {
    if (this.getPivotADC() > Constants.LAUNCHER_PIVOT_MAX_ADC && output < 0.0)
    {
      m_pivot.set(ControlMode.PercentOutput, output);
    }
    else if (this.getPivotADC() > Constants.LAUNCHER_PIVOT_MAX_ADC)
    {
      m_pivot.set(ControlMode.PercentOutput, 0.0);
    }
    else if (this.getPivotADC() <= Constants.LAUNCHER_PIVOT_MIN_ADC && output > 0.0)
    {
      m_pivot.set(ControlMode.PercentOutput, output);
    }
    else if (this.getPivotADC() <= Constants.LAUNCHER_PIVOT_MIN_ADC)
    {
      m_pivot.set(ControlMode.PercentOutput, 0.0);
    }
    else
    {
      m_pivot.set(ControlMode.PercentOutput, 0.0);
      System.out.println("\nPIVOT ANGLE ERROR- Pivot ADC:" + this.getPivotADC() + "\n");
    }
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
}