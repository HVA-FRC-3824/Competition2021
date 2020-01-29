package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanel extends SubsystemBase
{
  private WPI_TalonSRX m_wheelSpinner;

  private DigitalInput m_colorSensor;

  public ControlPanel() 
  {
    m_wheelSpinner = new WPI_TalonSRX(Constants.CONTROL_PANEL_SPINNER_ID);
    RobotContainer.configureTalonSRX(m_wheelSpinner, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                            Constants.CONTROL_PANEL_SPINNER_F, Constants.CONTROL_PANEL_SPINNER_P, 
                            Constants.CONTROL_PANEL_SPINNER_I, Constants.CONTROL_PANEL_SPINNER_D, 0, 0);

    m_colorSensor = new DigitalInput(Constants.CONTROLPANEL_COLOR_SENSOR_PORT);
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
  public WPI_TalonSRX getWheelSpinnerTalonSRX() 
  {
      return m_wheelSpinner;
  }

  public void setWheelSpinnerPower(double power)
  {
    m_wheelSpinner.set(ControlMode.PercentOutput, power);
  }

  public void setWheelSpinnerRPM(int rpm)
  {
    m_wheelSpinner.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm));
  }
}