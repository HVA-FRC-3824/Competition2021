package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * A command that will turn the robot to the specified angle.
 */
public class ChassisTurnAngle extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */
  public ChassisTurnAngle(double targetAngleDegrees) {
    super(
      new PIDController(Constants.K_CHASSIS_TURN_P, Constants.K_CHASSIS_TURN_I, Constants.K_CHASSIS_TURN_D),
      // Close loop on heading
      RobotContainer.m_chassis::getAngle,
      // Set reference to target
      targetAngleDegrees,
      // Pipe output to turn robot
      output -> RobotContainer.m_chassis.drive(0, output),
      // Require the drive
      RobotContainer.m_chassis);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.K_TURN_TOLERANCE_DEG, Constants.K_TURN_RATE_TOLERANCE_DEG_PER_SEC);
  }
  
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}