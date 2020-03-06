package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DefenseMode extends CommandBase 
{
  // The subsystem the command runs on

  public DefenseMode()
  {
  }

  @Override
  public void initialize()
  {   
    // Retract intake
    RobotContainer.m_intake.retractExtender();
    
    //Sets isDefending to true
    RobotContainer.m_LEDs.setIsDefending(true);

    //angle launcher straight
    RobotContainer.m_launcher.setAngle(1700);
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