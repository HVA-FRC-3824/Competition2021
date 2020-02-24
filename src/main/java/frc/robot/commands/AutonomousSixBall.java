package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousSixBall extends SequentialCommandGroup
{
  public AutonomousSixBall()
  {
    addCommands(
      // extend intake
      // align with vision with timeout
      // move elevator up to full chamber worth of position with timeout
      // stop launching
      // spin intake along with base
      // blink LEDs
      // follow path
      // stop blinking LEDs
      // retract intake, stop intake, stop base
      // align with vision with timeout
      // move elevator up to full chamber worth of position with timeout
      // stop launching
    );
  }
}