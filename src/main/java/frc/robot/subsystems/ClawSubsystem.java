// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase 
{

      private static final int CLAW_MOTOR_ID = Constants.CLAW_MOTOR_ID;
      private CANSparkMax m_clawMotor;
      private SparkMaxPIDController m_clawPIDController;
      private RelativeEncoder m_clawEncoder;


      //Values from documentation here https://github.com/REVrobotics/SPARK-MAX-Examples
      public double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

      public ClawSubsystem() {
            m_clawMotor = new CANSparkMax(CLAW_MOTOR_ID, MotorType.kBrushless);
            m_clawMotor.restoreFactoryDefaults();
            m_clawMotor.setIdleMode(IdleMode.kBrake);
            m_clawPIDController = m_clawMotor.getPIDController();
            m_clawEncoder = m_clawMotor.getEncoder();

            m_clawPIDController.setP(kP);
            m_clawPIDController.setI(kI);
            m_clawPIDController.setD(kD);
            m_clawPIDController.setIZone(kIz);
            m_clawPIDController.setFF(kFF);
            m_clawPIDController.setOutputRange(kMinOutput, kMaxOutput);
            m_clawPIDController.setPositionPIDWrappingMinInput(0.1);
            m_clawPIDController.setPositionPIDWrappingMaxInput(0.9);


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

            // if((p != kP)) { m_clawPIDController.setP(p); kP = p; }
            // if((i != kI)) { m_clawPIDController.setI(i); kI = i; }
            // if((d != kD)) { m_clawPIDController.setD(d); kD = d; }
            // if((iz != kIz)) { m_clawPIDController.setIZone(iz); kIz = iz; }
            // if((ff != kFF)) { m_clawPIDController.setFF(ff); kFF = ff; }
            // if((max != kMaxOutput) || (min != kMinOutput)) { 
            //   m_clawPIDController.setOutputRange(min, max); 
            //   kMinOutput = min; kMaxOutput = max; 
            // }

      }
      public void setArmPosition(double rotations) {
            m_clawPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);

      }
      public void setDummyArmPosition(double rotations) {
            if(m_clawEncoder.getPosition() >= Constants.armEncoderMax)
            {
                  m_clawMotor.set(0.0);
            }
      }
      
      public enum Position{
            
            //FIXME add shuffleboard control for these values
            cone(0.0),
            cube(0.0);
            
            public final double position;

            Position(double position){
                  this.position = position;
            }

      }
}

