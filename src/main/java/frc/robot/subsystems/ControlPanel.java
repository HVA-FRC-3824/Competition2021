package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanel extends SubsystemBase {

  private TalonSRX m_wheelSpinner;
  private DigitalInput m_colorSensor;

  public ControlPanel() {

    m_wheelSpinner = new TalonSRX(Constants.CONTROLPANEL_WHEEL_SPINNER_ID);
    m_colorSensor = new DigitalInput(Constants.CONTROLPANEL_COLOR_SENSOR_PORT);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}