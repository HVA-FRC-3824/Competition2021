package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class SetPIDValues extends CommandBase
{
  /**
   * Declare objects that will be used in configuring the PID of a TalonSRX/TalonFX.
   */
  private WPI_TalonSRX m_talonSRX = null;
  private WPI_TalonFX m_talonFX = null;
  private boolean m_controlMode;

  public SetPIDValues(WPI_TalonSRX talonSRX, WPI_TalonFX talonFX, boolean controlMode)
  {
    /* Instantiate class objects (m_talonSRX/m_talonFX) to passed TalonSRX/TalonFX (whichever is not null). */
    if (talonSRX != null)
      m_talonSRX = talonSRX;
    else if (talonFX != null)
      m_talonFX = talonFX;
    else
      System.out.println("\nError retrieving TalonSRX/TalonFX object to configure PIDs for via SmartDashboard.\n");

    /**
     * Initialize class controlMode variable to determine whether or not to configure with cruise 
     * velocity and acceleration (Motion Magic). 
     */
    m_controlMode = controlMode;
  }

  @Override
  public void initialize()
  {
    /* Configure the TalonSRX/TalonFX that was passed into the class constructor. */
    if (m_talonSRX != null)
      this.setPIDValuesTalonSRX();
    else if (m_talonFX != null)
      this.setPIDValuesTalonFX();  
    else
      System.out.println("\nError retrieving TalonSRX/TalonFX object to configure PIDs for via SmartDashboard.\n");
  }
  
  @Override
  public void execute()
  {
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }

  /**
   * Sets PID values for passed in TalonSRX.
   * By having this "universal" method in one class, rather than in each subsystem, reduces clutter on SmartDashboard and
   * redundancy in classes.
   */
  private void setPIDValuesTalonSRX()
  {
    /* Set FPID values for passed TalonSRX object using values from the SmartDashboard. */
    m_talonSRX.config_kF(Constants.K_SLOT_IDX, SmartDashboard.getNumber("F Value", 0.0), Constants.K_TIMEOUT_MS);
    m_talonSRX.config_kP(Constants.K_SLOT_IDX, SmartDashboard.getNumber("P Value", 0.0), Constants.K_TIMEOUT_MS);
    m_talonSRX.config_kI(Constants.K_SLOT_IDX, SmartDashboard.getNumber("I Value", 0.0), Constants.K_TIMEOUT_MS);
    m_talonSRX.config_kD(Constants.K_SLOT_IDX, SmartDashboard.getNumber("D Value", 0.0), Constants.K_TIMEOUT_MS);

    /* If using Motion Magic, configure cruise velocity and acceleration as well. */
    if (m_controlMode)
    {
      m_talonSRX.configMotionCruiseVelocity((int)SmartDashboard.getNumber("Cruise Velocity Value", 0), Constants.K_TIMEOUT_MS);
      m_talonSRX.configMotionAcceleration((int)SmartDashboard.getNumber("Acceleration Value", 0), Constants.K_TIMEOUT_MS);
    }
  }

  /**
   * Sets PID values for passed in TalonFX.
   * By having this universal method in one class, rather than in each subsystem, reduces clutter on SmartDashboard and
   * redundancy in classes.
   * Motion Magic will not be used with any TalonFX, thus cruise velocity and acceleration configuration are not required.
   */
  private void setPIDValuesTalonFX()
  {
    /* Set FPID values for passed TalonSRX object using values from the SmartDashboard. */
    m_talonFX.config_kF(Constants.K_SLOT_IDX, SmartDashboard.getNumber("F Value", 0.0), Constants.K_TIMEOUT_MS);
    m_talonFX.config_kP(Constants.K_SLOT_IDX, SmartDashboard.getNumber("P Value", 0.0), Constants.K_TIMEOUT_MS);
    m_talonFX.config_kI(Constants.K_SLOT_IDX, SmartDashboard.getNumber("I Value", 0.0), Constants.K_TIMEOUT_MS);
    m_talonFX.config_kD(Constants.K_SLOT_IDX, SmartDashboard.getNumber("D Value", 0.0), Constants.K_TIMEOUT_MS);
  }
}