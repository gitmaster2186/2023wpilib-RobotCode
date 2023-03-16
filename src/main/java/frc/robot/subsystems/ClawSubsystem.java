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


public class ClawSubsystem extends SubsystemBase{
    private static final int CLAW_MOTOR_ID = Constants.CLAW_MOTOR_ID;
    public static CANSparkMax m_clawMotor;
    private SparkMaxPIDController m_clawPIDController;
    private RelativeEncoder m_clawEncoder;
    private double[] clawPositionMap = {0, 5, 10}; //move to constants eventually
    private double currentRotation = 0;

    public final double DEADBAND = 0.1;
    private boolean isLimitSwitchEnabled = true;
    private boolean isSoftLimitEnabled = false; //change this one 
    private float FORWARD_SOFT_LIMIT = -0.5f;
    private float REVERSE_SOFT_LIMIT = -97;
    private SparkMaxLimitSwitch m_forwardLimit;
    private SparkMaxLimitSwitch m_reverseLimit;
    private final int PID_SLOT_ID = 0;
    private double kP = 0.45, kI = 1e-5, kD = 1, kIz = 0, kFF = 0, 
    kMaxOutput = 0.6, kMinOutput = -0.6; 

    private double CLAW_MAX_VOLTAGE = 0.5;
    public ClawSubsystem() {
        m_clawMotor = new CANSparkMax(CLAW_MOTOR_ID, MotorType.kBrushless);
        m_clawMotor.restoreFactoryDefaults();
        m_clawMotor.setIdleMode(IdleMode.kBrake);
        m_clawPIDController = m_clawMotor.getPIDController();
        m_clawEncoder = m_clawMotor.getEncoder();
        //m_clawMotor.setSmartCurrentLimit(int StallLimit, int FreeLimit); //already enabled by deaflt to 80A and 20A respectively. Test other new code before enabling this. 

        // !!!SID!!! XXX - this will basically set our max speed!!!
        m_clawMotor.enableVoltageCompensation(CLAW_MAX_VOLTAGE);

        currentRotation = m_clawEncoder.getPosition();
        //m_clawPIDController.setSmartMotionMaxAccel(maxAccel, PID_SLOT_ID);
        //m_clawPIDController.setSmartMotionMaxVelocity(maxVelocity, PID_SLOT_ID);

        //SmartDashboard.putNumber("current Rotation", currentRotation);
        // SmartDashboard.putNumber("Max Acceleration", maxAccel);
        // SmartDashboard.putNumber("Max Velocity", maxVelocity);




        m_forwardLimit = m_clawMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_reverseLimit = m_clawMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_forwardLimit.enableLimitSwitch(isLimitSwitchEnabled);
        m_reverseLimit.enableLimitSwitch(isLimitSwitchEnabled);

        //SmartDashboard.putBoolean("Limit Switch Enabled", isLimitSwitchEnabled);



        m_clawMotor.enableSoftLimit(SoftLimitDirection.kForward, isSoftLimitEnabled);
        m_clawMotor.enableSoftLimit(SoftLimitDirection.kReverse, isSoftLimitEnabled);
        m_clawMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, FORWARD_SOFT_LIMIT);
        m_clawMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, REVERSE_SOFT_LIMIT);

        // SmartDashboard.putBoolean("Soft Limit Enabled", isSoftLimitEnabled);
        // SmartDashboard.putNumber("Forward Soft Limit", m_clawMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
        // SmartDashboard.putNumber("Reverse Soft Limit", m_clawMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));        
        
        m_clawPIDController.setP(kP);
        m_clawPIDController.setI(kI);
        m_clawPIDController.setD(kD);
        m_clawPIDController.setIZone(kIz);
        m_clawPIDController.setFF(kFF);
        m_clawPIDController.setOutputRange(kMinOutput, kMaxOutput);
        m_clawPIDController.setPositionPIDWrappingMinInput(0.1);
        m_clawPIDController.setPositionPIDWrappingMaxInput(0.9); 
    }

    public void setClawSpeed(double joystickInput) {
        //m_clawPIDController.setReference(joystickInput * MAX_VOLTAGE, CANSparkMax.ControlType.kVoltage, PID_SLOT_ID);
        System.out.println("Setting with" + joystickInput);
        m_clawMotor.set(joystickInput);
    }

    // public Grab raiseClawPosition() {
        
    //     Grab toSet = currentPosition.raise();
    //     m_clawPIDController.setReference(clawPositionMap[toSet.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
    //     this.currentPosition = toSet;
    //     return currentPosition;
    // }
    // public Position lowerClawPosition() {
    //     Position toSet = currentPosition.lower();
    //     m_clawPIDController.setReference(clawPositionMapGrab[toSet.position], CANSparkMax.ControlType.kPosition, PID_SLOT_ID);
    //     this.currentPosition = toSet;
    //     return currentPosition;
    // }


    public enum Grab{
        none(0),
        cone(1),
        cube(2);
        
        public final int position;
        
        Grab(int position){
            this.position = position;
        }
        
        public Grab close() {
            if(this.position <= 0) {
                System.out.println("Smallest grab");
                return this;
            }
            return Grab.values()[this.position - 1];
        }
        public Grab open() {
            if(this.position >= Grab.values().length - 1) {
                System.out.println("Biggest Grab");
                return this;
            }
            return Grab.values()[this.position + 1];
        }
    }
        
}
