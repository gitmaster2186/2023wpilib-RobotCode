package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SetInitialStartPosition extends CommandBase {
  private final String startingPosition;
  private RobotContainer m_RobotContainer = new RobotContainer();
  private Robot m_Robot;
  /** Creates a new SetInitialStartPositionCommand. */
  public SetInitialStartPosition(String startingPosition) {
    this.startingPosition = startingPosition;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double initialPosition = 0.0;

    switch (startingPosition) {
        case "Left":
            initialPosition = 0.0;
            break;
        case "Center":
            initialPosition = 0.5;
            break;
        case "Right":
            initialPosition = 1.0;
            break;
        default:
            DriverStation.reportError("Invalid starting position: " + startingPosition, false);
            break;
    }

    m_Robot.setInitialPosition(initialPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
