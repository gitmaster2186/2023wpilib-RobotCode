// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlatformDockPidCommand_Pitch extends PIDCommand {
  public static AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  DrivetrainSubsystem m_drivetrainSubsystem_local;
  private static double last_pitch=0.0;
  private static int m_count_of_finishied=0;
  private static int m_count_of_in_smooth=0;
  /** Creates a new PlatformDockPidCommand. */
  public PlatformDockPidCommand_Pitch(DrivetrainSubsystem m_drivetrainSubsystem) {
    
    super(
        // The contpitcher that the command will use
        new PIDController(Constants.kP,Constants.kI, Constants.kD),//P,I,D
        // This should return the measurement
        () -> smoothpitch(m_navx.getPitch()),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // this is input to Plant - aka Drive Sub System
            

            m_drivetrainSubsystem.drive_pid_x(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
    // Configure additional PID options by calling `getContpitcher` here.
  
    
  }
 
  private static double smoothpitch(float pitch) {
    m_count_of_in_smooth=m_count_of_in_smooth+1;
    SmartDashboard.putNumber("In smooth pitch", m_count_of_in_smooth);
    double calculatedpitch=pitch;
    SmartDashboard.putNumber("read pitch", pitch);
    if (Math.abs(pitch) < 2.5)
    {
      calculatedpitch=0.00000001;
      last_pitch=0.00000001;

    }
    else if(Math.abs(pitch) < 40)
    {
      calculatedpitch=pitch;
      last_pitch=calculatedpitch;

    }
    else
    {
      //calcuatedpitch=20*pitch/Math.abs(pitch);
      calculatedpitch=last_pitch;
      //calculatedpitch=0.001*pitch/Math.abs(pitch);
    }
    SmartDashboard.putNumber("set pitch", calculatedpitch);
    return calculatedpitch;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
       SmartDashboard.putNumber("Autonomous Balance", m_count_of_finishied);
       m_count_of_finishied=m_count_of_finishied+1;
    return false;
  }
}
