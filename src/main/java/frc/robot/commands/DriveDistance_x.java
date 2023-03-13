// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveDistance_x extends CommandBase {
  private final DrivetrainSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;
  private double start_pos_x=0.0;
  int dis_fin=0;
  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param meters The number of meters the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance_x(double speed, double meters, DrivetrainSubsystem drive) {
    m_distance = meters;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_drive.drive ( new ChassisSpeeds(0, 0, 0));
    // m_drive.zeroGyroscope();
//   m_drive.SwerveDriveOdomertyInitialize();
   start_pos_x=m_drive.getCurrentPose().getX();
   System.out.println("start_pos_x");
   System.out.println(start_pos_x);
   dis_fin=0;
   SmartDashboard.putNumber("drivex",dis_fin);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drive.drive(new ChassisSpeeds(m_speed,0,0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive (new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    dis_fin=dis_fin+1;
    SmartDashboard.putNumber("drivex",dis_fin);
    return Math.abs(m_drive.getCurrentPose().getX()-start_pos_x) >= m_distance;
    
   //return false;
  }
}
