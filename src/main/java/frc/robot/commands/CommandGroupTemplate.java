package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandGroupTemplate extends SequentialCommandGroup
{
  public CommandGroupTemplate()
  {
    addCommands(
        new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
        RobotContainer.m_chassis.generateRamsete("straightForward", true),
        new InstantCommand(() -> RobotContainer.m_limelight.turnOnLED())
    );
  }
}