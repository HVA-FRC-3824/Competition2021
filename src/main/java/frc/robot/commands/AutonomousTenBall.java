package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousTenBall extends SequentialCommandGroup
{
  public AutonomousTenBall()
  {
    // addCommands(
        // new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
        // RobotContainer.generateRamsete("tenBall_pathOne"),
        // RobotContainer.m_inlineCommands.m_chassisAutoTurnToTarget,
        // new WaitCommand(3.0),
        // new InstantCommand(() -> RobotContainer.m_limelight.blinkLED()),
        // RobotContainer.generateRamsete("tenBall_pathTwo"),
        // RobotContainer.m_inlineCommands.m_chassisAutoTurnToTarget
    // );
  }
}