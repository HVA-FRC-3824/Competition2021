package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LauncherSetPreset extends CommandBase 
{
  public LauncherSetPreset()
  {
    addRequirements(RobotContainer.m_launcher);
  }

  @Override
  public void initialize()
  {
    if (RobotContainer.m_OI.getOperatorController().getRawButton(8))
    {
      RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_INITIATION_LINE_TOP_RPM, 
                                          Constants.LAUNCHER_INITIATION_LINE_BOTTOM_RPM,
                                          Constants.LAUNCHER_INITIATION_LINE_ANGLE);
    }
    else if (RobotContainer.m_OI.getOperatorController().getRawButton(9))
    {
      RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_CLOSE_TRENCH_TOP_RPM, 
                                          Constants.LAUNCHER_CLOSE_TRENCH_BOTTOM_RPM,
                                          Constants.LAUNCHER_CLOSE_TRENCH_ANGLE);
    }
    else
    {
        RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_FAR_TRENCH_TOP_RPM, 
                                            Constants.LAUNCHER_FAR_TRENCH_BOTTOM_RPM,
                                            Constants.LAUNCHER_FAR_TRENCH_ANGLE);
    }
  }
  
  @Override
  public void execute()
  {
  }

  @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }
}