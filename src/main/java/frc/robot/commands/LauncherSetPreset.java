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
    /* Update chassis to ready in launch ready status because vision is not being used. */
    RobotContainer.m_launcher.updateLaunchReadyStatus(3, true);

    /* Based on the position of the switch on operator board, set preset of launcher. */
    // if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OPERATOR_LAUNCHER_PRESET_INIT_BTN_ID))
    // {
    //   RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_INITIATION_LINE_TOP_RPM, 
    //                                       Constants.LAUNCHER_INITIATION_LINE_BOTTOM_RPM,
    //                                       Constants.LAUNCHER_INITIATION_LINE_ANGLE);
    // }
    // else if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OPERATOR_LAUNCHER_PRESET_CLOSE_BTN_ID))
    // {
    //   RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_CLOSE_TRENCH_TOP_RPM, 
    //                                       Constants.LAUNCHER_CLOSE_TRENCH_BOTTOM_RPM,
    //                                       Constants.LAUNCHER_CLOSE_TRENCH_ANGLE);
    // }
    // else
    // {
    //     RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_FAR_TRENCH_TOP_RPM, 
    //                                         Constants.LAUNCHER_FAR_TRENCH_BOTTOM_RPM,
    //                                         Constants.LAUNCHER_FAR_TRENCH_ANGLE);
    // }
    if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OP_LAUNCHER_PRESET_GREEN_BTN_ID))
    {
      RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_GREEN_ZONE_TOP_MOTOR_POWER, Constants.LAUNCHER_GREEN_ZONE_BOTTOM_MOTOR_POWER);
    }
    else if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OP_LAUNCHER_PRESET_YELLOW_BTN_ID))
    {
      RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_YELLOW_ZONE_TOP_MOTOR_POWER, Constants.LAUNCHER_YELLOW_ZONE_BOTTOM_MOTOR_POWER);
    }
    else if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OP_LAUNCHER_PRESET_BLUE_BTN_ID))
    {
      RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_BLUE_ZONE_TOP_MOTOR_POWER, Constants.LAUNCHER_BLUE_ZONE_BOTTOM_MOTOR_POWER);
    }
    else if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OP_LAUNCHER_PRESET_RED_BTN_ID))
    {
      RobotContainer.m_launcher.setPreset(Constants.LAUNCHER_RED_ZONE_TOP_MOTOR_POWER, Constants.LAUNCHER_RED_ZONE_BOTTOM_MOTOR_POWER);
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