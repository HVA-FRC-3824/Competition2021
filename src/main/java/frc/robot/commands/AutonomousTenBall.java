package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousTenBall extends SequentialCommandGroup
{
  public AutonomousTenBall()
  {
    addCommands(
      new PrintCommand("\nStarting Command\n"),
      new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
      new PrintCommand("\nBlinked LED\n"),
      RobotContainer.generateRamsete("tenBall_pathOne"),
      new PrintCommand("\nGenerated first path\n"),
      new ChassisTurnToTarget(),
      new PrintCommand("\nTurned to target\n"),
      new WaitCommand(3.0),
      new PrintCommand("\nwaited\n"),
      new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
      new PrintCommand("\nBlinked LED\n"),
      RobotContainer.generateRamsete("tenBall_pathTwo"),
      new PrintCommand("\nGenerated second path\n"),
      new ChassisTurnToTarget(),
      new PrintCommand("\nTurned to target\n")
      // RobotContainer.m_inlineCommands.m_chassisAutoTurnToTarget
    );
  }
}