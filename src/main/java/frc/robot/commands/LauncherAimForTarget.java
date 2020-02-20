package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LauncherAimForTarget extends CommandBase 
{
  double m_targetArea;
  double m_outputMultiplier;
  int m_topWheelOutput;
  int m_bottomWheelOutput;

  double m_angleError;
  double m_pivotOutput;

  public LauncherAimForTarget()
  {
    /**
     * Require launcher to takeover all launcher input.
     * This will allow this command to set the launcher angle and wheel speeds based on vision.
     */
    addRequirements(RobotContainer.m_launcher);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    m_targetArea = 0.0;
    m_outputMultiplier = 0.0;
    m_topWheelOutput = 0;
    m_bottomWheelOutput = 0;
    
    m_angleError = 0.0;
    m_pivotOutput = 0.0;
  }
  
  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute()
  {
    /**
     * Calculating/outputting launcher wheel speeds based on vision.
     */

    /* Target area is used as a gauge to see how far away the robot is from the target. */
    m_targetArea = RobotContainer.m_limelight.getTargetArea();

    /* Multiplier to calculate RPM so when target area is large, the RPM is small (because closer). Vice versa with "ta" being small. */
    m_outputMultiplier = (1 - (m_targetArea / 100));

    /* Calculate RPM based on how far away from the target the robot is (how big the target area is). */
    m_topWheelOutput = (int) (Constants.LAUNCHER_WHEEL_MAX_RPM * m_outputMultiplier); // TODO: Tune this so it actually works properly...
    m_bottomWheelOutput = (int) (Constants.LAUNCHER_WHEEL_MAX_RPM * m_outputMultiplier); // TODO: Tune this.

    /* Give output to launcher wheels. */
    RobotContainer.m_launcher.setTopWheelRPM(m_topWheelOutput);
    RobotContainer.m_launcher.setBottomWheelRPM(m_bottomWheelOutput);

    /**
     * Aiming launcher pivot tilt based on vision.
     */

    /* Angle error is how off the robot is from the target in the "y" direction. */
    m_angleError = RobotContainer.m_limelight.getTargetOffsetY();

    /* Calculate output to launcher angle pivot based on offset from target. */
    if (m_angleError > 0.5)
    {
      m_pivotOutput = Constants.LAUNCHER_AIM_VISION_P * m_angleError - Constants.LAUNCHER_AIM_VISION_MIN;
    }
    else if (m_angleError < -0.5)
    {
      m_pivotOutput = Constants.LAUNCHER_AIM_VISION_P * m_angleError + Constants.LAUNCHER_AIM_VISION_MIN;
    }

    /* Give output to launcher pivot. */
    RobotContainer.m_launcher.setPivotPower(m_pivotOutput);
  }

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished()
  {
    return (m_angleError < 0.5 && m_angleError > -0.5);
  }
}