package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{

  private WPI_TalonSRX m_reelLeft;
  private WPI_TalonSRX m_reelRight;

  private WPI_TalonFX m_liftLeft;
  private WPI_TalonFX m_liftRight;

  private Servo m_lockRatchetLeft;
  private Servo m_lockRatchetRight;
  
  public Climber()
  {
    m_reelLeft = new WPI_TalonSRX(Constants.CLIMBER_REEL_LEFT_ID);
    RobotContainer.configureTalonSRX(m_reelLeft, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                    Constants.CLIMBER_REEL_LEFT_F, Constants.CLIMBER_REEL_LEFT_P, 
                                    Constants.CLIMBER_REEL_LEFT_I, Constants.CLIMBER_REEL_LEFT_D, 
                                    Constants.CLIMBER_REEL_LEFT_CRUISEVELOCITY, Constants.CLIMBER_REEL_LEFT_ACCELERATION, true);

    m_reelRight = new WPI_TalonSRX(Constants.CLIMBER_REEL_RIGHT_ID);
    RobotContainer.configureTalonSRX(m_reelRight, true, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                    Constants.CLIMBER_REEL_RIGHT_F, Constants.CLIMBER_REEL_RIGHT_P, 
                                    Constants.CLIMBER_REEL_RIGHT_I, Constants.CLIMBER_REEL_RIGHT_D, 
                                    Constants.CLIMBER_REEL_RIGHT_CRUISEVELOCITY, Constants.CLIMBER_REEL_RIGHT_ACCELERATION, true);

    m_liftLeft = new WPI_TalonFX(Constants.CLIMBER_LIFT_LEFT_ID);
    RobotContainer.configureTalonFX(m_liftLeft, true, false, Constants.CLIMBER_LIFT_LEFT_F, Constants.CLIMBER_LIFT_LEFT_P,
                            Constants.CLIMBER_LIFT_LEFT_I, Constants.CLIMBER_LIFT_LEFT_D);
    m_liftLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.K_TIMEOUT_MS);
    m_liftLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.K_TIMEOUT_MS);
    m_liftLeft.configMotionCruiseVelocity(Constants.CLIMBER_LIFT_LEFT_CRUISEVELOCITY, Constants.K_TIMEOUT_MS);
    m_liftLeft.configMotionAcceleration(Constants.CLIMBER_LIFT_LEFT_ACCELERATION, Constants.K_TIMEOUT_MS);
    /* Set brake mode to prevent the robot from falling after the match ends. */
    m_liftLeft.setNeutralMode(NeutralMode.Brake);

    m_liftRight = new WPI_TalonFX(Constants.CLIMBER_LIFT_RIGHT_ID);
    RobotContainer.configureTalonFX(m_liftRight, false, false, Constants.CLIMBER_LIFT_RIGHT_F, Constants.CLIMBER_LIFT_RIGHT_P,
                            Constants.CLIMBER_LIFT_RIGHT_I, Constants.CLIMBER_LIFT_RIGHT_D);
    m_liftRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.K_TIMEOUT_MS);
    m_liftRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.K_TIMEOUT_MS);
    m_liftRight.configMotionCruiseVelocity(Constants.CLIMBER_LIFT_RIGHT_CRUISEVELOCITY, Constants.K_TIMEOUT_MS);
    m_liftRight.configMotionAcceleration(Constants.CLIMBER_LIFT_RIGHT_ACCELERATION, Constants.K_TIMEOUT_MS);
    /* Set brake mode to prevent the robot from falling after the match ends. */
    m_liftRight.setNeutralMode(NeutralMode.Brake);
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
  public WPI_TalonSRX getReelLeftTalonSRX()
  {
    return m_reelLeft;
  }
  public WPI_TalonSRX getReelRightTalonSRX()
  {
    return m_reelRight;
  }
  public WPI_TalonFX getLiftLeftTalonFX()
  {
    return m_liftLeft;
  }
  public WPI_TalonFX getLiftRightTalonFX()
  {
    return m_liftRight;
  }

  /**
   * Method to extend/retract climber poles with power.
   * @param power range is from 1.0 to -1.0
   */
  public void setReelsPower(double power)
  {
    m_reelLeft.set(ControlMode.PercentOutput, power);
    m_reelRight.set(ControlMode.PercentOutput, power);
  }

  /**
   * Sets the desired position of the left and right reel.
   * @param position will be used as the setpoint for the PID controller
   */
  public void setReelsPosition(int position)
  {
    /* Verify desired position is within the minimum and maximum reel range. */
    // if (position < Constants.CLIMBER_REEL_MIN_POSITION)
    // {
    //   position = Constants.CLIMBER_REEL_MIN_POSITION;
    // }
    // else if (position > Constants.CLIMBER_REEL_MAX_POSITION)
    // {
    //   position = Constants.CLIMBER_REEL_MAX_POSITION;
    // }

    m_reelLeft.set(ControlMode.MotionMagic, position);
    m_reelRight.set(ControlMode.MotionMagic, position);
  }

  /**
   * Method to move climber while button is pressed and stop at current position when button is released.
   */
  public void stepReels(double distance)
  {
    double presentPositionLeft = m_reelLeft.getSelectedSensorPosition();
    m_reelLeft.set(ControlMode.MotionMagic, presentPositionLeft + distance);
    double presentPositionRight = m_reelRight.getSelectedSensorPosition();
    m_reelRight.set(ControlMode.MotionMagic, presentPositionRight + distance);
  }

  /**
   * Method to extend/retract climber poles with power.
   * @param power range is from 1.0 to -1.0
   */
  public void setLiftsPower(double power)
  {
    m_liftLeft.set(ControlMode.PercentOutput, power);
    m_liftRight.set(ControlMode.PercentOutput, power);
  }

  /**
   * Sets the desired position of the left and right reel.
   * @param position will be used as the setpoint for the PID controller
   */
  public void setLiftsPosition(int position)
  {
    /* Verify desired position is within the minimum and maximum reel range. */
    // if (position < Constants.CLIMBER_REEL_MIN_POSITION)
    // {
    //   position = Constants.CLIMBER_REEL_MIN_POSITION;
    // }
    // else if (position > Constants.CLIMBER_REEL_MAX_POSITION)
    // {
    //   position = Constants.CLIMBER_REEL_MAX_POSITION;
    // }

    m_liftLeft.set(ControlMode.MotionMagic, position);
    m_liftRight.set(ControlMode.MotionMagic, position);
  }

  /**
   * Method to move climber while button is pressed and stop at current position when button is released.
   */
  public void stepLifts(double distance)
  {
    double presentPositionLeft = m_liftLeft.getSelectedSensorPosition();
    m_liftLeft.set(ControlMode.MotionMagic, presentPositionLeft + distance);
    double presentPositionRight = m_liftRight.getSelectedSensorPosition();
    m_liftRight.set(ControlMode.MotionMagic, presentPositionRight + distance);
  }
}
  