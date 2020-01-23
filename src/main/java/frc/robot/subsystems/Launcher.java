package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase 
{
  private WPI_TalonFX m_topWheel;
  private WPI_TalonFX m_bottomWheel;

  private WPI_TalonSRX m_feeder;

  private WPI_TalonSRX m_pivot;

  private DigitalInput m_ballSwitch;

  public Launcher()
  {
    m_topWheel = new WPI_TalonFX(Constants.LAUNCHER_WHEEL_TOP_ID);
    m_bottomWheel = new WPI_TalonFX(Constants.LAUNCHER_WHEEL_BOTTOM_ID);

    m_feeder = new WPI_TalonSRX(Constants.LAUNCHER_FEEDER_ID);

    m_pivot = new WPI_TalonSRX(Constants.LAUNCHER_PIVOT_ID);

    m_ballSwitch = new DigitalInput(Constants.LAUNCHER_BALL_SWITCH_PORT);
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
  public WPI_TalonFX getTopWheelTalonFX()
  {
      return m_topWheel;
  }
  public WPI_TalonFX getBottomWheelTalonFX()
  {
      return m_bottomWheel;
  }
  public WPI_TalonSRX getFeederTalonSRX()
  {
      return m_feeder;
  }
  public WPI_TalonSRX getPivotTalonSRX()
  {
      return m_pivot;
  }
}