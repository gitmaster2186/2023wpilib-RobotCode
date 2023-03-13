// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousTime extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on time. This will drive out for a period of time, turn
   * around for time (equivalent to time to turn around) and drive forward again. This should mimic
   * driving out, turning around and driving back.
   *
   * @param drivetrain The drive subsystem on which this command will run
   */
  public AutonomousTime(DrivetrainSubsystem drivetrainsubsystem) {
    addCommands(
        new DriveTime_x(2, 2.0, drivetrainsubsystem),
         new TurnDegrees(2, 90, drivetrainsubsystem),
         new DriveTime_x(2, 2.0, drivetrainsubsystem),
         new TurnDegrees(2, -90, drivetrainsubsystem)
        );
  }
}
