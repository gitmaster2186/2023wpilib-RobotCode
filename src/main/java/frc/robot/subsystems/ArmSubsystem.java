// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase 
{

      private static final int ARM_MOTOR_ID = Constants.ARM_MOTOR_ID;
      public static CANSparkMax m_armMotor;
      private SparkMaxPIDController m_armPIDController;
      private RelativeEncoder m_armEncoder;
      private Position currentPosition = Position.ground;
      private double[] rotationMap = {0, 10, 20, 30, 40}; //move to constants eventually
      private double currentRotation = 0;

      //Values from documentation here https://github.com/REVrobotics/SPARK-MAX-Examples
      private double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

      public ArmSubsystem() {
            m_armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
            m_armMotor.restoreFactoryDefaults();
            m_armMotor.setIdleMode(IdleMode.kBrake);
            m_armPIDController = m_armMotor.getPIDController();

            //FIXME use rev 11-1271 bore encoder
            m_armEncoder = m_armMotor.getEncoder();

            m_armPIDController.setP(kP);
            m_armPIDController.setI(kI);
            m_armPIDController.setD(kD);
            m_armPIDController.setIZone(kIz);
            m_armPIDController.setFF(kFF);
            m_armPIDController.setOutputRange(kMinOutput, kMaxOutput);
            m_armPIDController.setPositionPIDWrappingMinInput(0.1);
            m_armPIDController.setPositionPIDWrappingMaxInput(0.9);


            SmartDashboard.putNumber("P Gain", kP);
            SmartDashboard.putNumber("I Gain", kI);
            SmartDashboard.putNumber("D Gain", kD);
            SmartDashboard.putNumber("I Zone", kIz);
            SmartDashboard.putNumber("Feed Forward", kFF);
            SmartDashboard.putNumber("Max Output", kMaxOutput);
            SmartDashboard.putNumber("Min Output", kMinOutput);

            SmartDashboard.putNumberArray("Rotation Map", rotationMap);
            SmartDashboard.putNumber("current Rotation", currentRotation);
            SmartDashboard.putString("current Position", currentPosition.name());




            //MyTab.add("Arm Position", Position.coneLow).withWidget(BuiltInWidgets.kComboBoxChooser);
      }


      @Override
      public void periodic() {
            //uncomment this and previous block to set values with smart dashboard for testing

            double p = SmartDashboard.getNumber("P Gain", kP);
            double i = SmartDashboard.getNumber("I Gain", kI);
            double d = SmartDashboard.getNumber("D Gain", kD);
            double iz = SmartDashboard.getNumber("I Zone", kIz);
            double ff = SmartDashboard.getNumber("Feed Forward", kFF);
            double max = SmartDashboard.getNumber("Max Output", kMaxOutput);
            double min = SmartDashboard.getNumber("Min Output", kMinOutput);

            double[] newRotationMap = SmartDashboard.getNumberArray("Rotation Map", this.rotationMap);
            this.rotationMap = newRotationMap;

            SmartDashboard.putNumber("current Rotation", m_armEncoder.getPosition());
            SmartDashboard.putString("current Position", currentPosition.name());
            
            if((p != kP)) { m_armPIDController.setP(p); kP = p; }
            if((i != kI)) { m_armPIDController.setI(i); kI = i; }
            if((d != kD)) { m_armPIDController.setD(d); kD = d; }
            if((iz != kIz)) { m_armPIDController.setIZone(iz); kIz = iz; }
            if((ff != kFF)) { m_armPIDController.setFF(ff); kFF = ff; }
            if((max != kMaxOutput) || (min != kMinOutput)) { 
                  m_armPIDController.setOutputRange(min, max); 
                  kMinOutput = min; kMaxOutput = max; 
            }
      }
      public void setArmPosition(Position toSet) {
            m_armPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition);
      }
      public Position raiseArmPosition() {

            Position toSet = currentPosition.raise();
            m_armPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition);
            this.currentPosition = toSet;
            return currentPosition;
      }
      public Position lowerArmPosition() {
            Position toSet = currentPosition.lower();
            m_armPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition);
            this.currentPosition = toSet;
            return currentPosition;
      }


      public void setArmSpeed(double joystickInput) {
            //FIXME add limit switches here or encoder max values
            m_armMotor.set(joystickInput);
      }
      
      public enum Position{

            //FIXME add shuffleboard control for these values
            ground(0),
            cubeLow(1),
            coneLow(2),
            cubeHigh(3),
            coneHigh(4);
            
            public final int position;

            Position(int position){
                  this.position = position;
            }

            public Position lower() {
                  if(this.position <= 0) {
                        System.out.println("At lowest position");
                        return this;
                  }
                  return Position.values()[this.position - 1];
            }
            public Position raise() {
                  if(this.position >= Position.values().length - 1) {
                        System.out.println("At max position");
                        return this;
                  }
                  return Position.values()[this.position + 1];
            }
      }
      
}
