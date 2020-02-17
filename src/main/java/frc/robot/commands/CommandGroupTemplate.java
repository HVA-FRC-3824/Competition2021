package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CommandGroupTemplate extends SequentialCommandGroup
{
  public CommandGroupTemplate()
  {
    addCommands(
        new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
        new InstantCommand(() -> RobotContainer.m_chassis.reverseTest(false)),
        RobotContainer.m_chassis.generateRamsete("straight", true),
        new InstantCommand(() -> RobotContainer.m_limelight.turnOnLED()),
        new WaitCommand(3.0),
        new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
        new InstantCommand(() -> RobotContainer.m_chassis.reverseTest(false)),
        RobotContainer.m_chassis.generateRamsete("straight3", false),
        new InstantCommand(() -> RobotContainer.m_limelight.turnOnLED())
    );
  }
}