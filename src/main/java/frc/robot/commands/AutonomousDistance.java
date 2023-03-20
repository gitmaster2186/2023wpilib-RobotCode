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
    //drivetrainsubsystem.SwerveDriveOdomertyInitialize();
    System.out.println("Auto Distance Started");
    addCommands(
      // positive x meters is forward
      // Positive y meters is left
      // Drive distance x is not as accurate on trailer floor due to higher speed (>=2)
      new DriveDistance_x(-2, 5, drivetrainsubsystem),
      new DriveDistance_x(2, 3, drivetrainsubsystem)
      // new PlatformDockPidCommand_Pitch(drivetrainsubsystem)
       //new DriveDistance_y(-1, 0.6, drivetrainsubsystem)
       
     // new DriveDistance_y(2, 0.5, drivetrainsubsystem)

    //new TurnDegrees(3, 90,1, drivetrainsubsystem)
     // new DriveDistance_x(-2, 2, drivetrainsubsystem)
     //new DriveDistance_y(-2, 0.5, drivetrainsubsystem)
    //new TurnDegrees(3, 90,-1, drivetrainsubsystem)
        );
  }
}
