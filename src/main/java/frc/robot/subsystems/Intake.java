package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase 
{
  private DoubleSolenoid m_extender;

  private WPI_TalonSRX m_wheelIntake;

  public Intake() 
  {
    m_extender = new DoubleSolenoid(Constants.INTAKE_EXTENDER_PORT_A, Constants.INTAKE_EXTENDER_PORT_B);

    m_wheelIntake = new WPI_TalonSRX(Constants.INTAKE_WHEEL_INTAKE_ID);
    RobotContainer.configureTalonSRX(m_wheelIntake, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                    Constants.INTAKE_WHEEL_INTAKE_F, Constants.INTAKE_WHEEL_INTAKE_P, 
                                    Constants.INTAKE_WHEEL_INTAKE_I, Constants.INTAKE_WHEEL_INTAKE_D, 0, 0, true);
  }

  /**
   * This method will be called once per scheduler run.
   */
  @Override
  public void periodic()
  {
    RobotContainer.displayTalonSRXInfo(m_wheelIntake, "Intake Wheels PID Info");
  }

  /**
   * Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  public WPI_TalonSRX getWheelIntakeTalonSRX()
  {
      return m_wheelIntake;
  }
 
  /**
   * Method that toggles the intake pistons between being retracted and extended.
   */
  public void toggleExtender()
  {
    m_extender.set(this.getExtenderValueToToggle());
  }

  /**
   * Gets the opposite value of the current solenoid value for toggling extender.
   * @return the solenoid value the extender should be set to in order to toggle.
   */
  public Value getExtenderValueToToggle()
  {
    if (m_extender.get() == Value.kForward)
      return Value.kReverse;
    else
      return Value.kForward;
  }

  /**
   * Method that spins intake wheels with power.
   * @param power range is from -1.0 to 1.0.
   */
  public void setWheelPower(double power)
  {
    m_wheelIntake.set(ControlMode.PercentOutput, power);
  }

  /**
   * Method sets intake wheel's RPM with ControlMode.Velocity.
   * @param rpm is converted to units per 100 milliseconds for ControlMode.Velocity.
   */
  public void setWheelRPM(int rpm)
  {
    m_wheelIntake.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm));
  }
}