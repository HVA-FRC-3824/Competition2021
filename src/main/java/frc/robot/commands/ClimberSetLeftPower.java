package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberSetLeftPower extends CommandBase 
{

  // 1 for extned, -1 for retract
  private int m_type = 1;

  public ClimberSetLeftPower(int type)
  {
    // addRequirements(RobotContainer.m_climber);
    m_type = type;
  }

  @Override
  public void initialize()
  {
  }
  
  @Override
  public void execute()
  {
    /* Based on the position of the switch on operator board, set. */
    if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OPERATOR_CLIMBER_REEL_BTN_ID))
    {
      RobotContainer.m_climber.setLeftReelPower(m_type * Constants.CLIMBER_REEL_EXTEND_POWER);
      RobotContainer.m_climber.setLeftLiftPower(-m_type * Constants.CLIMBER_LIFT_UNREEL_POWER);
    }
    else if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OPERATOR_CLIMBER_LIFT_BTN_ID))
    {
      RobotContainer.m_climber.setLeftLiftPower(m_type * Constants.CLIMBER_LIFT_EXTEND_POWER);
    }
    else
    {
      System.out.println("Whether to set reel or lift power was not specified.");
    }
  }

  @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}