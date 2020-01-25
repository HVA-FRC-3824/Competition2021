package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase 
{
  private DoubleSolenoid m_extender;

  private WPI_TalonSRX m_wheelIntake;

  public Intake() 
  {
    m_extender = new DoubleSolenoid(Constants.INTAKE_EXTENDER_PORT_A, Constants.INTAKE_EXTENDER_PORT_B);

    m_wheelIntake = new WPI_TalonSRX(Constants.INTAKE_WHEEL_INTAKE_ID);
    Robot.configureTalonSRX(m_wheelIntake, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, Constants.INTAKE_WHEEL_INTAKE_F, 
                            Constants.INTAKE_WHEEL_INTAKE_P, Constants.INTAKE_WHEEL_INTAKE_I, Constants.INTAKE_WHEEL_INTAKE_D, 0, 0);
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
  public WPI_TalonSRX getWheelIntakeTalonSRX()
  {
      return m_wheelIntake;
  }

}