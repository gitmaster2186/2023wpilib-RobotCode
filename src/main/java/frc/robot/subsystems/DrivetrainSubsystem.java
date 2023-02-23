// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.*;

import java.util.Arrays;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */

  public static final double MAX_VOLTAGE = Constants.MAX_Voltage;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  
//   public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
//           SdsModuleConfigurations.MK4_L2.getDriveReduction() *
//           SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
public static final double MAX_VELOCITY_METERS_PER_SECOND = 4000 / 60.0 *
        SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );



  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
//   private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
 private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final SwerveDriveOdometry m_odometry;
  
  private Pose2d m_pos;
  public SwerveModuleState[] states ;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    m_pos = new Pose2d();

        

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // 
    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            //Prabhu- Use Mk4i - L2 as this is what 2186 Team purchased
            Mk4iSwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    ); 

    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
    
    

    //Declare odometry object
    m_odometry = new SwerveDriveOdometry(
        m_kinematics, getGyroscopeRotation(),
        getModulePositions()); 

  }

  

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
//     m_pigeon.setFusedHeading(0.0);

    // FIXME Uncomment if you are using a NavX
   m_navx.zeroYaw();
  // System.out.println("zeroGyroscope method has been called");

  }

  public Rotation2d getGyroscopeRotation() {
   
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
   
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  
  public void drive_pid_x(double pid_output) {
        SmartDashboard.putNumber("getPitch",m_navx.getPitch() );
        SmartDashboard.putNumber("getRoll",m_navx.getRoll() );
        SmartDashboard.putNumber("getYaw",m_navx.getYaw() );
        
        SmartDashboard.putNumber("pid_output",pid_output );

        
        m_chassisSpeeds=new ChassisSpeeds(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*pid_output*-1, 0.000001,0);
        m_chassisSpeeds=new ChassisSpeeds(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*pid_output*-1, -0.000001,0);

      
      }
  @Override
  public void periodic() {
    SmartDashboard.putString("In Periodic of Drive Subsystem" ,m_chassisSpeeds.toString());
    SmartDashboard.putNumber("x velocity" ,m_chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("y Velocity" ,m_chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Onmega Radians per second" ,m_chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.updateValues();
    states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    
    m_pos=m_odometry.update(getGyroscopeRotation(), getModulePositions() );
    
  }

  private SwerveModulePosition[] getModulePositions()
  {
        
     SwerveModulePosition m_frontLeftModule_position=new SwerveModulePosition(m_frontLeftModule.getDriveDistance(), new Rotation2d(m_frontLeftModule.getSteerAngle()));
     SwerveModulePosition m_frontRightModule_position=new SwerveModulePosition(m_frontRightModule.getDriveDistance(), new Rotation2d(m_frontRightModule.getSteerAngle()));
     SwerveModulePosition m_backLeftModule_position=new SwerveModulePosition(m_backLeftModule.getDriveDistance(), new Rotation2d(m_backLeftModule.getSteerAngle()));
     SwerveModulePosition m_backRightModule_position=new SwerveModulePosition(m_backRightModule.getDriveDistance(), new Rotation2d(m_backRightModule.getSteerAngle()));
     SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];
     Arrays.fill(m_modulePositions, new SwerveModuleState());
     m_modulePositions[0]=m_frontLeftModule_position;
     m_modulePositions[1]=m_frontRightModule_position;
     m_modulePositions[2]=m_backLeftModule_position;
     m_modulePositions[3]=m_backRightModule_position;
     return m_modulePositions;

  }

  public void SwerveDriveOdomertyInitialize()
  {
 //Creating my odometry object from the kinematics object and the initial wheel positions.
    // Here, our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing the opposing alliance wall.
    SwerveModulePosition m_frontLeftModule_position=new SwerveModulePosition();
    SwerveModulePosition m_frontRightModule_position=new SwerveModulePosition();
    SwerveModulePosition m_backLeftModule_position=new SwerveModulePosition();
    SwerveModulePosition m_backRightModule_position=new SwerveModulePosition();

    SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];

    Arrays.fill(m_modulePositions , new SwerveModulePosition());
    m_modulePositions[0]=m_frontLeftModule_position;
    m_modulePositions[1]=m_frontRightModule_position;
    m_modulePositions[2]=m_backLeftModule_position;
    m_modulePositions[3]=m_backRightModule_position;
    m_odometry.resetPosition( getGyroscopeRotation(),m_modulePositions, new Pose2d(new Translation2d(0,0),new Rotation2d(0))); 
  }

  
  public Pose2d getCurrentPose()
  {
 
      return m_pos;
  }

public void zeroRoll() {
        
}

public void drive_parameters(double x_speed, double y_speed, double final_angle) {
        m_chassisSpeeds= new ChassisSpeeds(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*x_speed,DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND*y_speed,final_angle);
}
}
