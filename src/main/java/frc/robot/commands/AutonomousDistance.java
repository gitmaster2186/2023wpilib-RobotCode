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
  public AutonomousDistance(DrivetrainSubsystem drivetrainsubsystem, String startingPosition) {
    addCommands(
      //  new DriveDistance(2, 1, drivetrainsubsystem),
      new PlatformDockPidCommand_Pitch(drivetrainsubsystem)
     //   new TurnDegrees(1, 90, drivetrainsubsystem),
     //   new DriveDistance(-2, 1, drivetrainsubsystem)
     //   new TurnDegrees(1, 90, drivetrainsubsystem)
        );
  }
}
