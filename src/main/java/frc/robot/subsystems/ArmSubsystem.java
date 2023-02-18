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

public class ArmSubsystem extends SubsystemBase {

      private static final int ARM_MOTOR_ID = Constants.ARM_MOTOR_ID;
      private CANSparkMax m_armMotor;
      private SparkMaxPIDController m_armPIDController;
      private RelativeEncoder m_armEncoder;

      //Values from documentation here https://github.com/REVrobotics/SPARK-MAX-Examples
      public double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

      public ArmSubsystem() {
            m_armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
            m_armMotor.restoreFactoryDefaults();
            m_armPIDController = m_armMotor.getPIDController();
            m_armEncoder = m_armMotor.getEncoder();

            m_armPIDController.setP(kP);
            m_armPIDController.setI(kI);
            m_armPIDController.setD(kD);
            m_armPIDController.setIZone(kIz);
            m_armPIDController.setFF(kFF);
            m_armPIDController.setOutputRange(kMinOutput, kMaxOutput);

            /*    SmartDashboard.putNumber("P Gain", kP);
            SmartDashboard.putNumber("I Gain", kI);
            SmartDashboard.putNumber("D Gain", kD);
            SmartDashboard.putNumber("I Zone", kIz);
            SmartDashboard.putNumber("Feed Forward", kFF);
            SmartDashboard.putNumber("Max Output", kMaxOutput);
            SmartDashboard.putNumber("Min Output", kMinOutput);
            SmartDashboard.putNumber("Set Rotations", 0);
            */
            


      }


      @Override
      public void periodic() {
            //uncomment this and previous block to set values with smart dashboard for testing

            // double p = SmartDashboard.getNumber("P Gain", 0);
            // double i = SmartDashboard.getNumber("I Gain", 0);
            // double d = SmartDashboard.getNumber("D Gain", 0);
            // double iz = SmartDashboard.getNumber("I Zone", 0);
            // double ff = SmartDashboard.getNumber("Feed Forward", 0);
            // double max = SmartDashboard.getNumber("Max Output", 0);
            // double min = SmartDashboard.getNumber("Min Output", 0);
            
            // 
            // double rotations = SmartDashboard.getNumber("Set Rotations", 0);

            // if((p != kP)) { m_armPIDController.setP(p); kP = p; }
            // if((i != kI)) { m_armPIDController.setI(i); kI = i; }
            // if((d != kD)) { m_armPIDController.setD(d); kD = d; }
            // if((iz != kIz)) { m_armPIDController.setIZone(iz); kIz = iz; }
            // if((ff != kFF)) { m_armPIDController.setFF(ff); kFF = ff; }
            // if((max != kMaxOutput) || (min != kMinOutput)) { 
            //   m_armPIDController.setOutputRange(min, max); 
            //   kMinOutput = min; kMaxOutput = max; 
            // }

      }
      public void setArmPosition(double rotations) {
            m_armPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
      }
}
