package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private DoubleSolenoid m_extender;
  private TalonSRX m_wheelIntake;

  public Intake() {

    m_extender = new DoubleSolenoid(Constants.INTAKE_EXTENDER_PORT_A, Constants.INTAKE_EXTENDER_PORT_B);
    m_wheelIntake = new TalonSRX(Constants.INTAKE_WHEEL_INTAKE_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}