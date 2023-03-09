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
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase 
{
    //declare subsystem variables
    private static final int ARM_MOTOR_ID = Constants.ARM_MOTOR_ID;
    public static CANSparkMax m_armMotor;
    private SparkMaxPIDController m_armPIDController;
    private RelativeEncoder m_armEncoder;
    private double currentRotation;
    private Position currentPosition = Position.ground;

    //create the roation map
    private double[] rotationMap = {-10, -20, -50, -60, -70}; //move to constants eventually

    //create limit configuration variables
    public final double DEADBAND = 0.12;
    private boolean isLimitSwitchEnabled = true;
    private boolean isSoftLimitEnabled = true;
    private float FORWARD_SOFT_LIMIT = 1;
    private float REVERSE_SOFT_LIMIT = -89;
    private SparkMaxLimitSwitch m_forwardLimit;
    private SparkMaxLimitSwitch m_reverseLimit;
    private final int PID_SLOT_ID = 0;
    private double maxAccel = 1000; //in RPM/s
    private double maxVelocity = 2000; //in RPM

        
    //PID values from documentation here https://github.com/REVrobotics/SPARK-MAX-Examples
    private double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
    
    public ArmSubsystem() {
        m_armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        m_armMotor.restoreFactoryDefaults();
        m_armMotor.setIdleMode(IdleMode.kBrake);
        m_armPIDController = m_armMotor.getPIDController();
        m_armEncoder = m_armMotor.getEncoder();
        //m_armMotor.setSmartCurrentLimit(int StallLimit, int FreeLimit); //already enabled by deaflt to 80A and 20A respectively. Test other new code before enabling this. 

        currentRotation = m_armEncoder.getPosition();
        m_armPIDController.setSmartMotionMaxAccel(maxAccel, PID_SLOT_ID);
        m_armPIDController.setSmartMotionMaxVelocity(maxVelocity, PID_SLOT_ID);

        SmartDashboard.putNumber("current Rotation", currentRotation);
        SmartDashboard.putNumber("Max Acceleration", maxAccel);
        SmartDashboard.putNumber("Max Velocity", maxVelocity);




        m_forwardLimit = m_armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_reverseLimit = m_armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_forwardLimit.enableLimitSwitch(isLimitSwitchEnabled);
        m_reverseLimit.enableLimitSwitch(isLimitSwitchEnabled);

        SmartDashboard.putBoolean("Limit Switch Enabled", isLimitSwitchEnabled);



        m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, isSoftLimitEnabled);
        m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, isSoftLimitEnabled);
        m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, FORWARD_SOFT_LIMIT);
        m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, REVERSE_SOFT_LIMIT);

        SmartDashboard.putBoolean("Soft Limit Enabled", isSoftLimitEnabled);
        SmartDashboard.putNumber("Forward Soft Limit", m_armMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
        SmartDashboard.putNumber("Reverse Soft Limit", m_armMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));

        
        
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



        
        //SmartDashboard.putNumberArray("Rotation Map", rotationMap);
        for (int r = 0; r < rotationMap.length; r++) {
            SmartDashboard.putNumber("Rotation Map of " + r, rotationMap[r]);
        }
        SmartDashboard.putString("current Position", currentPosition.name());

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
        
        
        SmartDashboard.putNumber("current Rotation", m_armEncoder.getPosition());
        SmartDashboard.putString("current Position", currentPosition.name());

        SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());

        


        float forwardSoftLimit = (float) SmartDashboard.getNumber("Forward Soft Limit", FORWARD_SOFT_LIMIT);
        float reverseSoftLimit = (float) SmartDashboard.getNumber("Reverse Soft Limit", REVERSE_SOFT_LIMIT);
        if (forwardSoftLimit != FORWARD_SOFT_LIMIT) {
            m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardSoftLimit);
            FORWARD_SOFT_LIMIT = forwardSoftLimit;
        }
        if (reverseSoftLimit != REVERSE_SOFT_LIMIT) {
            m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverseSoftLimit);
            REVERSE_SOFT_LIMIT = reverseSoftLimit;
        }
        //update these
        //m_armPIDController.setSmartMotionMaxAccel(maxAccel, PID_SLOT_ID);
        //m_armPIDController.setSmartMotionMaxVelocity(maxVelocity, PID_SLOT_ID);
        double newMaxAccel = SmartDashboard.getNumber("Max Accel", maxAccel);
        double newMaxVelocity = SmartDashboard.getNumber("Max Velocity", maxVelocity);
        if (newMaxAccel != maxAccel) {
            m_armPIDController.setSmartMotionMaxAccel(newMaxAccel, PID_SLOT_ID);
            maxAccel = newMaxAccel;
        }
        if (newMaxVelocity != maxVelocity) {
            m_armPIDController.setSmartMotionMaxVelocity(newMaxVelocity, PID_SLOT_ID);
            maxVelocity = newMaxVelocity;
        }
        
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
        m_armPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
    }
    public Position raiseArmPosition() {
        
        Position toSet = currentPosition.raise();
        m_armPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
        this.currentPosition = toSet;
        return currentPosition;
    }
    public Position lowerArmPosition() {
        Position toSet = currentPosition.lower();
        m_armPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
        this.currentPosition = toSet;
        return currentPosition;
    }
    
    public Position refreshArmPosition() {
        m_armPIDController.setReference(rotationMap[currentPosition.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
        return currentPosition;
    }
    
    
    public void setArmSpeed(double joystickInput) {
        //FIXME add limit switches here or encoder max values
        m_armPIDController.setReference(-joystickInput * Constants.MAX_Voltage * (0.001), CANSparkMax.ControlType.kVoltage, PID_SLOT_ID);
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
