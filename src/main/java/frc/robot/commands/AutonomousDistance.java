// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(DrivetrainSubsystem drivetrainsubsystem) {
    addCommands(
        new DriveDistance(-0.05, 10, drivetrainsubsystem),
        new TurnDegrees(-0.05, 180, drivetrainsubsystem),
        new DriveDistance(-0.05, 10, drivetrainsubsystem),
        new TurnDegrees(0.05, 180, drivetrainsubsystem));
  }
}
