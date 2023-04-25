// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

<<<<<<< Updated upstream
<<<<<<<< Updated upstream:src/main/java/frc/robot/commands/DriveDistance_y.java
public class DriveDistance_y extends CommandBase {
========
public class DriveDistance_x extends CommandBase {
>>>>>>>> Stashed changes:src/main/java/frc/robot/commands/DriveDistance_x.java
=======
public class DriveDistance_y extends CommandBase {
>>>>>>> Stashed changes
  private final DrivetrainSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;
  private double start_pos_y=0.0;
  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param meters The number of meters the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
<<<<<<< Updated upstream
<<<<<<<< Updated upstream:src/main/java/frc/robot/commands/DriveDistance_y.java
  public DriveDistance_y(double speed, double meters, DrivetrainSubsystem drive) {
========
  public DriveDistance_x(double speed, double meters, DrivetrainSubsystem drive) {
>>>>>>>> Stashed changes:src/main/java/frc/robot/commands/DriveDistance_x.java
=======
  public DriveDistance_y(double speed, double meters, DrivetrainSubsystem drive) {
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
   //m_drive.SwerveDriveOdomertyInitialize();
   start_pos_y=m_drive.getCurrentPose().getY();
   //System.out.println("start_pos_y");
   //System.out.println(start_pos_y);
=======
   m_drive.SwerveDriveOdomertyInitialize();
   start_pos_y=m_drive.getCurrentPose().getY();
   System.out.print(start_pos_y);
>>>>>>> Stashed changes
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(new ChassisSpeeds(0,m_speed,0));
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
<<<<<<< Updated upstream
    return Math.abs(m_drive.getCurrentPose().getY() - start_pos_y) >= m_distance;
=======
    return Math.abs(m_drive.getCurrentPose().getY()-start_pos_y) >= m_distance;
>>>>>>> Stashed changes
   //return false;
  }
}
