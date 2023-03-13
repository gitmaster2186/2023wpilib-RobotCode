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


public class ClawSubsystem extends SubsystemBase 
{
    //declare subsystem variables
    private static final int CLAW_MOTOR_ID = Constants.CLAW_MOTOR_ID;
    public static CANSparkMax m_clawMotor;
    private SparkMaxPIDController m_clawPIDController;
    private RelativeEncoder m_clawEncoder;
    private double currentRotation;
    private Object currentPosition = Object.empty;

    private SparkMaxLimitSwitch m_forwardLimit;
    private SparkMaxLimitSwitch m_reverseLimit;


    //create the roation map
    private double[] rotationMap = {0, 10, 20};

    //create limit configuration variables
    public final double DEADBAND = 0.25;
    private boolean isLimitSwitchEnabled = true;
    private boolean isSoftLimitEnabled = false; //tune before enabling
    private float FORWARD_SOFT_LIMIT = 1;
    private float REVERSE_SOFT_LIMIT = -89;
    private final int PID_SLOT_ID = 0;

        
    //PID values from documentation here https://github.com/REVrobotics/SPARK-MAX-Examples
    private double kP = 0.45, kI = 1e-5, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 0.35, kMinOutput = -0.35;
    
    public ClawSubsystem() {
        m_clawMotor = new CANSparkMax(CLAW_MOTOR_ID, MotorType.kBrushless);
        m_clawMotor.restoreFactoryDefaults();
        m_clawMotor.setIdleMode(IdleMode.kBrake);
        m_clawPIDController = m_clawMotor.getPIDController();
        m_clawEncoder = m_clawMotor.getEncoder();
        //m_clawMotor.setSmartCurrentLimit(int StallLimit, int FreeLimit); //already enabled by deaflt to 80A and 20A respectively. Test other new code before enabling this. 

        currentRotation = m_clawEncoder.getPosition();
        //m_clawPIDController.setSmartMotionMaxAccel(maxAccel, PID_SLOT_ID);
        //m_clawPIDController.setSmartMotionMaxVelocity(maxVelocity, PID_SLOT_ID);

        SmartDashboard.putNumber("current Rotation", currentRotation);
        // SmartDashboard.putNumber("Max Acceleration", maxAccel);
        // SmartDashboard.putNumber("Max Velocity", maxVelocity);




        m_forwardLimit = m_clawMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_reverseLimit = m_clawMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_forwardLimit.enableLimitSwitch(isLimitSwitchEnabled);
        m_reverseLimit.enableLimitSwitch(isLimitSwitchEnabled);

        SmartDashboard.putBoolean("Limit Switch Enabled", isLimitSwitchEnabled);



        m_clawMotor.enableSoftLimit(SoftLimitDirection.kForward, isSoftLimitEnabled);
        m_clawMotor.enableSoftLimit(SoftLimitDirection.kReverse, isSoftLimitEnabled);
        m_clawMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, FORWARD_SOFT_LIMIT);
        m_clawMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, REVERSE_SOFT_LIMIT);

        SmartDashboard.putBoolean("Soft Limit Enabled", isSoftLimitEnabled);
        SmartDashboard.putNumber("Forward Soft Limit", m_clawMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
        SmartDashboard.putNumber("Reverse Soft Limit", m_clawMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));

        
        
        m_clawPIDController.setP(kP);
        m_clawPIDController.setI(kI);
        m_clawPIDController.setD(kD);
        m_clawPIDController.setIZone(kIz);
        m_clawPIDController.setFF(kFF);
        m_clawPIDController.setOutputRange(kMinOutput, kMaxOutput);
        m_clawPIDController.setPositionPIDWrappingMinInput(0.1);
        m_clawPIDController.setPositionPIDWrappingMaxInput(0.9);

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
        
        
        SmartDashboard.putNumber("current Rotation", m_clawEncoder.getPosition());
        SmartDashboard.putString("current Position", currentPosition.name());
        SmartDashboard.putNumber("Set Point", rotationMap[currentPosition.position]);


        SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());

        


        float forwardSoftLimit = (float) SmartDashboard.getNumber("Forward Soft Limit", FORWARD_SOFT_LIMIT);
        float reverseSoftLimit = (float) SmartDashboard.getNumber("Reverse Soft Limit", REVERSE_SOFT_LIMIT);
        if (forwardSoftLimit != FORWARD_SOFT_LIMIT) {
            m_clawMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardSoftLimit);
            FORWARD_SOFT_LIMIT = forwardSoftLimit;
        }
        if (reverseSoftLimit != REVERSE_SOFT_LIMIT) {
            m_clawMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverseSoftLimit);
            REVERSE_SOFT_LIMIT = reverseSoftLimit;
        }
        
        if((p != kP)) { m_clawPIDController.setP(p); kP = p; }
        if((i != kI)) { m_clawPIDController.setI(i); kI = i; }
        if((d != kD)) { m_clawPIDController.setD(d); kD = d; }
        if((iz != kIz)) { m_clawPIDController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_clawPIDController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            m_clawPIDController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }
        for (int r = 0; r < rotationMap.length; r++) {
            rotationMap[r] = SmartDashboard.getNumber("Rotation Map of " + r, rotationMap[r]);
        }
    }
    // I don't know if we need this method here. Uncomment if needed
    // public void setClawPosition(Position toSet) {
    //     m_clawPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
    // }
    public Object openClawPosition() {
        
        Object toSet = currentPosition.raise();
        m_clawPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
        this.currentPosition = toSet;
        return currentPosition;
    }
    public Object closeClawPosition() {
        Object toSet = currentPosition.lower();
        m_clawPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
        this.currentPosition = toSet;
        return currentPosition;
    }

    public Object setClawPosition(Object toSet) {
        m_clawPIDController.setReference(rotationMap[toSet.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
        this.currentPosition = toSet;
        return currentPosition;
    }
    
    public void setClawSpeed(double joystickInput) {
        //FIXME add limit switches here or encoder max values
        m_clawPIDController.setReference(-joystickInput * Constants.MAX_Voltage, CANSparkMax.ControlType.kVoltage, PID_SLOT_ID);
    }
    
    public enum Object{
        empty(0),
        cone(1),
        cube(2);
        
        public final int position;
        
        Object(int position){
            this.position = position;
        }
        
        public Object lower() {
            if(this.position <= 0) {
                System.out.println("At lowest position");
                return this;
            }
            return Object.values()[this.position - 1];
        }
        public Object raise() {
            if(this.position >= Object.values().length - 1) {
                System.out.println("At max position");
                return this;
            }
            return Object.values()[this.position + 1];
        }
    }
    
}
