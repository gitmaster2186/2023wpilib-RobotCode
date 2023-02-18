// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {

  public static final double MAX_VOLTAGE = Constants.MAX_Voltage;
  public static final int 

  public ArmSubsystem() {
    
  }


  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  
  public void drive_pid_x(double pid_output) {
        //System.out.println("getRoll()");
        SmartDashboard.putNumber("getPitch",m_navx.getPitch() );
        SmartDashboard.putNumber("getRoll",m_navx.getRoll() );
        SmartDashboard.putNumber("getYaw",m_navx.getYaw() );
        
        //System.out.println( m_navx.getRoll());
        //System.out.println("pid_output");
        //System.out.println(pid_output);
        SmartDashboard.putNumber("pid_output",pid_output );

        
      //  if((Math.abs(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*pid_output) < 10)) {
                m_chassisSpeeds=new ChassisSpeeds(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*pid_output*-1, 0.000001,0);
                m_chassisSpeeds=new ChassisSpeeds(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*pid_output*-1, -0.000001,0);

        //}
        //Prabhu bring the value in 0 to 1 range
        // modified_pid_output=(Math.abs(pid_output) -180)/180;
        // modified_pid_output=modified_pid_output* pid_output/Math.abs(pid_output);
        // modified_pid_output=-1*modified_pid_output;
        
        //m_chassisSpeeds=ChassisSpeeds.fromFieldRelativeSpeeds(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*modified_pid_output, 0,0,this.getGyroscopeRotation() );
        //m_chassisSpeeds=ChassisSpeeds.fromFieldRelativeSpeeds(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*pid_output, 0,0,new Rotation2d());
      }
  @Override
  public void periodic() {
    
  }

public void zeroRoll() {
        
}
}
