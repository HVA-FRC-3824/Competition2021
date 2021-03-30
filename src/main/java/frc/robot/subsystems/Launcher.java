package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.LauncherSetAngle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase 
{
  private WPI_TalonFX m_topWheel;
  // private WPI_TalonFX m_bottomWheel;

  private WPI_TalonSRX m_pivot;
  private AnalogInput m_pivotFeedback;

  private boolean[] m_readyToLaunch;

  private WPI_TalonSRX m_topRightWheel;
  private WPI_TalonSRX m_topLeftWheel;
  private WPI_TalonSRX m_bottomWheel;

  public Launcher()
  {
    m_topWheel = new WPI_TalonFX(Constants.LAUNCHER_TOP_WHEEL_ID);
    RobotContainer.configureTalonFX(m_topWheel, false, false, Constants.LAUNCHER_TOP_WHEEL_F, Constants.LAUNCHER_TOP_WHEEL_P,
                                    Constants.LAUNCHER_TOP_WHEEL_I, Constants.LAUNCHER_TOP_WHEEL_D);

    m_topRightWheel = new WPI_TalonSRX(Constants.LAUNCHER_TOP_RIGHT_MOTOR_ID);
    RobotContainer.configureTalonSRX(m_topRightWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                    Constants.LAUNCHER_TOP_WHEEL_F, Constants.LAUNCHER_TOP_WHEEL_P,
                                    Constants.LAUNCHER_TOP_WHEEL_I, Constants.LAUNCHER_TOP_WHEEL_D, 0, 0, false);

    m_topLeftWheel = new WPI_TalonSRX(Constants.LAUNCHER_TOP_LEFT_MOTOR_ID);
    RobotContainer.configureTalonSRX(m_topLeftWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                    Constants.LAUNCHER_TOP_WHEEL_F, Constants.LAUNCHER_TOP_WHEEL_P,
                                    Constants.LAUNCHER_TOP_WHEEL_I, Constants.LAUNCHER_TOP_WHEEL_D, 0, 0, false);

    m_bottomWheel = new WPI_TalonSRX(Constants.LAUNCHER_BOTTOM_WHEEL_ID);
    RobotContainer.configureTalonSRX(m_bottomWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                    Constants.LAUNCHER_TOP_WHEEL_F, Constants.LAUNCHER_TOP_WHEEL_P,
                                    Constants.LAUNCHER_TOP_WHEEL_I, Constants.LAUNCHER_TOP_WHEEL_D, 0, 0, false);

    m_pivot = new WPI_TalonSRX(Constants.LAUNCHER_PIVOT_ID);
    RobotContainer.configureTalonSRX(m_pivot, false, FeedbackDevice.Analog, false, false, 
                                     0.0, 0.0, 0.0, 0.0, 0, 0, false);

    m_pivotFeedback = new AnalogInput(Constants.LAUNCHER_PIVOT_FEEDBACK_PORT);

    /**
     * Ready to launch array of booleans tells LEDs (thus telling driver/operator) if ready to launch.
     */
    m_readyToLaunch = new boolean[4];
    m_readyToLaunch[0] = false; // Launcher Pivot is at the setpoint angle (little to no error).
    m_readyToLaunch[1] = false; // Launcher Top is at the setpoint RPM (within a range).
    m_readyToLaunch[2] = false; // Launcher Bottom is at the setpoint RPM (within a range).
    m_readyToLaunch[3] = false; // (VISION ONLY) Chassis is turned towards target (little to no error).
  }

  /**
   * This method will be called once per scheduler run.
   */
  @Override
  public void periodic()
  {
    // SmartDashboard.putNumber("LAUNCHER TOP SETPOINT", m_topWheel.getClosedLoopTarget());
    // SmartDashboard.putNumber("LAUNCHER TOP VELOCITY", m_topWheel.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("LAUNCHER BOTTOM SETPOINT", m_bottomWheel.getClosedLoopTarget());
    // SmartDashboard.putNumber("LAUNCHER BOTTOM VELOCITY", m_bottomWheel.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("LAUNCHER PIVOT ADC", this.getPivotADC());

    /**
     * Updates whether or not the launcher RPMs are ready to launch.
     */
    this.updateRPMsLaunchReadyStatus(m_topWheel, 1);
    // this.updateRPMsLaunchReadyStatus(m_bottomWheel, 2);
  }

  /**
   * Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  public WPI_TalonFX getTopWheelTalonFX()
  {
    return m_topWheel;
  }
  public WPI_TalonSRX getBottomWheelTalonSRX()
  {
    return m_bottomWheel;
  }

  public WPI_TalonSRX getTopRightWheelTalonSRX()
  {
    return m_topRightWheel;
  }
  public WPI_TalonSRX getTopLeftWheelTalonSRX()
  {
    return m_topLeftWheel;
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
  public void setTopRightWheelPower(double power)
  {
    m_topRightWheel.set(ControlMode.PercentOutput, power);
  }
  public void setTopLeftWheelPower(double power)
  {
    m_topLeftWheel.set(ControlMode.PercentOutput, power);
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
    m_topWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.LAUNCHER_TOP_WHEEL_TPR));
  }

  public void setTopLeftWheelRPM(int rpm)
  {
    m_topLeftWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.LAUNCHER_TOP_WHEEL_TPR));
  }
  public void setTopRightWheelRPM(int rpm)
  {
    m_topRightWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.LAUNCHER_TOP_WHEEL_TPR));
  }

  public void setBottomWheelRPM(int rpm)
  {
    m_bottomWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.LAUNCHER_BOTTOM_WHEEL_TPR));
  }

  /**
   * Sets power of linear actuator for launcher pivot angle.
   */
  public void setPivotPower(double power)
  {
    if ((this.getPivotADC() <= 2000 && power <= 0.0) || (this.getPivotADC() >= 3500 && power >= 0.0))
    {
      m_pivot.set(ControlMode.PercentOutput, 0.0);
    }
    else
    {
      m_pivot.set(ControlMode.PercentOutput, -power);
    }
  }

  /**
   * Sets the desired position of the launcher pivot angle.
   * @param position will be used as the setpoint for the PID controller
   */
  public void setAngle(int setpoint)
  {
    Command setAngle = new LauncherSetAngle(setpoint);
    setAngle.schedule();
  }

  /**
   * Get current ADC of pivot linear actuator to verify it is not overdriven.
   */
  public int getPivotADC()
  {
    return m_pivotFeedback.getValue();
  }

  /**
   * This method is used to set the wheel RPMs and pivot angle for presets.
   */
  public void setPreset(int topLeftRPM, int topRightRPM, int bottomRPM, int pivotSetpoint)
  {
    this.setTopLeftWheelRPM(topLeftRPM);
    this.setTopRightWheelRPM(topRightRPM);
    this.setBottomWheelRPM(bottomRPM);

    // this.updateLaunchReadyStatus(0, true);

    this.setAngle(pivotSetpoint);
  }

  /**
   * Stops launcher wheels and pivot.
   */
  public void stopLauncher()
  {
    this.setTopLeftWheelPower(0.0);
    this.setTopRightWheelPower(0.0);
    this.setBottomWheelPower(0.0);

    this.setPivotPower(0.0);
  }

  /**
   * Updates launch ready status for RPMs of top and bottom wheels.
   */
  public void updateRPMsLaunchReadyStatus(WPI_TalonFX talonFX, int launchReadyIndex)
  {
    int error = Math.abs((int) talonFX.getClosedLoopTarget() - talonFX.getSelectedSensorVelocity());
    if (error < Constants.LAUNCHER_RPM_ERROR_TRESHOLD)
    {
      this.updateLaunchReadyStatus(launchReadyIndex, true);
    }
    else
    {
      this.updateLaunchReadyStatus(launchReadyIndex, false);
    }
  }

  /**
   * Method to update m_readyToLaunch array to tell whether the launcher is ready to launch or not.
   * @param index is the item in the m_readyToLaunch array to edit.
   * @param value is the value to update the item to. (0: Angle, 1: Top, 2: Bottom, 3: Chassis)
   */
  public void updateLaunchReadyStatus(int index, boolean value)
  {
    m_readyToLaunch[index] = value;
  }

  /**
   * Get whether or not the launcher is ready to launch.
   * This factors in angle of chassis, launcher angle, and rpms.
   * @return true if ready to launch, false if not.
   */
  public boolean getLaunchReadyStatus()
  {
    boolean ready = true;
    String factorsReady = "";

    for (int i = 0; i < m_readyToLaunch.length; i++)
    {
      if (m_readyToLaunch[i])
      {
        factorsReady += i + "-";
      }
      else
      {
        ready = false;
      }
    }

    /* Display which factors are ready (0: Pivot, 1: Top, 2: Bottom, 3: Chassis) */
    SmartDashboard.putString("LAUNCHER STATUS", factorsReady);

    return ready;
  }
}