// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
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
    private double[] rotationMap = {0, 5, 10, 15, 20}; //move to constants eventually
    private double currentRotation = 0;
    private boolean enableLimitSwitch = false;

    public final double DEADBAND = 0.12;
    private SparkMaxLimitSwitch m_forwardLimit;
    private SparkMaxLimitSwitch m_reverseLimit;
        
    //Values from documentation here https://github.com/REVrobotics/SPARK-MAX-Examples
    private double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
    
    public ArmSubsystem() {
        m_armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        m_armMotor.restoreFactoryDefaults();
        m_armMotor.setIdleMode(IdleMode.kBrake);
        m_armPIDController = m_armMotor.getPIDController();
        
        //FIXME use rev 11-1271 bore encoder
        m_armEncoder = m_armMotor.getEncoder();
        m_forwardLimit = m_armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_reverseLimit = m_armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_forwardLimit.enableLimitSwitch(enableLimitSwitch);
        m_reverseLimit.enableLimitSwitch(enableLimitSwitch);
        
        
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

        SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
        SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isLimitSwitchEnabled());
        /* 
        use SmartDashboard tabs
        m_tab.add("P Gain", kP).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1));
        m_tab.add("I Gain", kI).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1));
        m_tab.add("D Gain", kD).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1));
        m_tab.add("I Zone", kIz).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1));
        m_tab.add("Feed Forward", kFF).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1));
        m_tab.add("Max Output", kMaxOutput).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1));
        m_tab.add("Min Output", kMinOutput).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1));
        
        for (int k = 0; k < rotationMap.length; k++) {
            m_tab.add("Rotation Map of " + k, rotationMap[k]).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -100, "max", 500));
        }
        
        */      
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
        SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
        
        if((p != kP)) { m_armPIDController.setP(p); kP = p; }
        if((i != kI)) { m_armPIDController.setI(i); kI = i; }
        if((d != kD)) { m_armPIDController.setD(d); kD = d; }
        if((iz != kIz)) { m_armPIDController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_armPIDController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            m_armPIDController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }
        for (int r = 0; r < rotationMap.length; r++) {
            rotationMap[r] = SmartDashboard.getNumber("Rotation Map of " + r, rotationMap[r]);
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
    
    public Position refreshArmPosition() {
        m_armPIDController.setReference(rotationMap[currentPosition.position], CANSparkMax.ControlType.kPosition);
        return currentPosition;
    }
    
    
    public void setArmSpeed(double joystickInput) {
        //FIXME add limit switches here or encoder max values
        m_armPIDController.setReference(-joystickInput * Constants.MAX_Voltage, CANSparkMax.ControlType.kVoltage);
    }
    
    public enum Position{
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
