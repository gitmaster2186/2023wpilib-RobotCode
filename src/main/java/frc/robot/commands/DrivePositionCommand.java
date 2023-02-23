// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DrivePositionCommand extends CommandBase {
  SwerveDriveOdometry m_subsystemOdometry;
  DrivetrainSubsystem this_DrivetrainSubsystem;
  Pose2d this_desired_pose;


  /** Creates a new AutonomousDriveDistance. */
  public DrivePositionCommand(DrivetrainSubsystem m_DrivetrainSubsystem, Pose2d desired_pose ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this_DrivetrainSubsystem=m_DrivetrainSubsystem;
    this_desired_pose=desired_pose;
    addRequirements(m_DrivetrainSubsystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   this_DrivetrainSubsystem.SwerveDriveOdomertyInitialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Prabhu - Can use a pid here for each axis-maybe overkill
  @Override
  public void execute() {
    //Drive robot
    Pose2d m_currentPose= this_DrivetrainSubsystem.getCurrentPose();
    //this_DrivetrainSubsystem.drive_parameters((this_desired_pose.getX()-m_currentPose.getX())/100,(this_desired_pose.getY()-m_currentPose.getY())/100,this_desired_pose.getRotation().getRadians()-m_currentPose.getRotation().getRadians());
    this_DrivetrainSubsystem.drive_parameters((this_desired_pose.getX()-m_currentPose.getX())/1000.0,(this_desired_pose.getY()-m_currentPose.getY())/1000.0,this_desired_pose.getRotation().getRadians());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //check Pose2d vs what we want
    return (this_DrivetrainSubsystem.getCurrentPose().getX()-  this_desired_pose.getX()< 0.01) &&  (this_DrivetrainSubsystem.getCurrentPose().getY()-  this_desired_pose.getY()< 0.01) ;
    
  }
}
