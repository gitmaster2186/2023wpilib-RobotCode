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
  DrivetrainSubsystem m_drivetrainSubsystem_local;
  private static double last_pitch=0.0;
  /** Creates a new PlatformDockPidCommand. */
  public PlatformDockPidCommand_Pitch(DrivetrainSubsystem m_drivetrainSubsystem) {
    
    super(
        // The contpitcher that the command will use
        new PIDController(Constants.kP,Constants.kI, Constants.kD),//P,I,D
        // This should return the measurement
        () -> smoothpitch(m_drivetrainSubsystem.getPitch()),
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
    m_drivetrainSubsystem_local=m_drivetrainSubsystem;
    
  }
 
  private static double smoothpitch(float pitch) {
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
       //FIXME Need to end on some button here
   SmartDashboard.putNumber("getRoll",m_drivetrainSubsystem_local.getPitch() );
   SmartDashboard.putNumber("getPitch",m_drivetrainSubsystem_local.getRoll() );
   SmartDashboard.putNumber("getYaw",m_drivetrainSubsystem_local.getYaw() );
    
    return false;
  }
}
