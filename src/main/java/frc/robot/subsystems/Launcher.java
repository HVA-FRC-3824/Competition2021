package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

  private TalonFX m_topWheel;
  private TalonFX m_bottomWheel;
  private TalonSRX m_feeder;
  private DigitalInput m_ballSwitch;
  private TalonSRX m_pivot;

  public Launcher() {

    m_topWheel = new TalonFX(Constants.LAUNCHER_WHEEL_TOP_ID);
    m_bottomWheel = new TalonFX(Constants.LAUNCHER_WHEEL_BOTTOM_ID);
    m_feeder = new TalonSRX(Constants.LAUNCHER_FEEDER_ID);
    m_ballSwitch = new DigitalInput(Constants.LAUNCHER_BALL_SWITCH_PORT);
    m_pivot = new TalonSRX(Constants.LAUNCHER_PIVOT_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}