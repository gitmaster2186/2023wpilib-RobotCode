// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TurnDegrees extends CommandBase {
  private final DrivetrainSubsystem m_drive;
  private final double m_degrees;
  private final double m_speed;
private double m_start_degree=0;
  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double speed, double degrees, DrivetrainSubsystem drive) {
    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.drive( new ChassisSpeeds());
    // m_drive.zeroGyroscope();
    m_drive.SwerveDriveOdomertyInitialize();
    m_start_degree=m_drive.getCurrentPose().getRotation().getDegrees();
  System.out.print(m_start_degree);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive ( new ChassisSpeeds(0, 0, m_speed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive( new ChassisSpeeds(0, 0, 0 ));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
   // return m_drive.getCurrentPose().getRotation().getDegrees() -m_degrees>=1;
   return false;
  }


}
