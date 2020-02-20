package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{

  private WPI_TalonSRX m_reelLeft;
  private WPI_TalonSRX m_reelRight;

  private DoubleSolenoid m_PTO;

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
    
    m_PTO = new DoubleSolenoid(Constants.CLIMBER_PTO_PORT_A, Constants.CLIMBER_PTO_PORT_B);

    m_lockRatchetLeft  = new Servo(Constants.CLIMBER_LOCK_RATCHET_LEFT_PORT);
    m_lockRatchetRight = new Servo(Constants.CLIMBER_LOCK_RATCHET_RIGHT_PORT);
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
    if (position < Constants.CLIMBER_REEL_MIN_POSITION)
    {
      position = Constants.CLIMBER_REEL_MIN_POSITION;
    }
    else if (position > Constants.CLIMBER_REEL_MAX_POSITION)
    {
      position = Constants.CLIMBER_REEL_MAX_POSITION;
    }

    m_reelLeft.set(ControlMode.MotionMagic, position);
    m_reelRight.set(ControlMode.MotionMagic, position);
  }

  /**
   * Gets the opposite value of the current solenoid value for toggling the PTO.
   * @return the solenoid value the PTO should be set to in order to toggle.
   */
  private Value getPTOValueToToggle()
  {
    if (m_PTO.get() == Value.kForward)
      return Value.kReverse;
    else
      return Value.kForward;
  }

  /**
   * Method that toggles the PTO between being engaged and disengaged.
   */
  public void togglePTO()
  {
    m_PTO.set(this.getPTOValueToToggle());
  }

  /**
   * Method to determine which value to toggle the passed in servo to and then set said value.
   * @param lockRatchet is the specified servo to set.
   */
  private void setLockRatchets(Servo lockRatchet)
  {
    /**
     * Calculate the average setpoint in order to determine which value to toggle the servos to.
     * This ensures that the servos are still toggled properly even if they drift a little from their initial setpoint.
     * Example: the servo is set for 0.0, but is knocked into the position of 0.25. If the locked position is 1.0, the
     * servo will still toggle to 1.0 because the current, knocked position is less than the average setpoint, 0.5 (between 0.0 and 1.0).
     */
    double averageSetpoint = (Constants.CLIMBER_LOCK_RATCHET_LOCKED_POSITION - Constants.CLIMBER_LOCK_RATCHET_RELEASED_POSITION) / 2;

    /* Toggles lock ratchet based on if less than or more than/equal to average setpoint. */
    if (lockRatchet.get() > averageSetpoint)
    {
      lockRatchet.set(Constants.CLIMBER_LOCK_RATCHET_RELEASED_POSITION);
    }
    else if (lockRatchet.get() <= averageSetpoint)
    {
      lockRatchet.set(Constants.CLIMBER_LOCK_RATCHET_LOCKED_POSITION);
    }
  }

  /**
   * Method to toggle lock ratchets' positions (released/locked).
   */
  public void toggleLockRatchets()
  {
    this.setLockRatchets(m_lockRatchetLeft);
    this.setLockRatchets(m_lockRatchetRight);
  }
}
  