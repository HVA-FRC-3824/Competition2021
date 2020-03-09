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
      // new InstantCommand(() -> RobotContainer.m_limelight.turnOnLED()),
      // new WaitCommand(1.0),
      // new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
      // new InstantCommand(() -> RobotContainer.m_chassis.setReversed(false)),
      // RobotContainer.m_chassis.generateRamsete("straightForward", true, false),
      // new InstantCommand(() -> RobotContainer.m_limelight.turnOnLED()),
      // new WaitCommand(2.0),
      // new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
      // new InstantCommand(() -> RobotContainer.m_chassis.setReversed(true)),
      // RobotContainer.m_chassis.generateRamsete("straightBackward", false, true),
      // new InstantCommand(() -> RobotContainer.m_limelight.turnOffLED())
      
      new WaitCommand(1.0),
      new InstantCommand(() -> RobotContainer.m_chassis.setReversed(false)),
      RobotContainer.m_chassis.generateRamsete("test", true, false),
      new WaitCommand(1.0),
      new InstantCommand(() -> RobotContainer.m_chassis.setReversed(true)),
      RobotContainer.m_chassis.generateRamsete("test2", false, true)
    );
  }
}