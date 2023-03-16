// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.ClawSpeedCommand;
import frc.robot.commands.ArmSpeedCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DrivePositionCommand;
import frc.robot.commands.PlatformDockPidCommand_Pitch;


public class RobotContainer {
    final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

    
    final CommandXboxController m_controller = new CommandXboxController(0);
    final CommandXboxController m_subcontroller = new CommandXboxController(1);
    Command zeroJoystickCommand;
    final ShuffleboardTab tab =  Shuffleboard.getTab("Main Tab");
    // Change these to match our plan for each position
    private final Command m_leftPositionCommand =   new AutonomousDistance(m_drivetrainSubsystem);
private final Command m_middlePositionCommand = getAutonomousCommand();
private final Command m_rightPositionCommand = new DrivePositionCommand(m_drivetrainSubsystem, new Pose2d(5.0, 4.9, m_drivetrainSubsystem.getGyroscopeRotation()));

// A chooser for autonomous commands
SendableChooser<Command> m_chooser = new SendableChooser<>();

    
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        m_chooser.setDefaultOption("Right Position", m_rightPositionCommand);
        m_chooser.addOption("Left Position", m_leftPositionCommand);
        m_chooser.addOption("Middle Position", m_middlePositionCommand);
        tab.add(m_chooser);
        //Prabhu Initialize Drive system to forward facing
        m_drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        0,
        new Rotation2d()
        ));
        //Prabhu initialize Gyroscope to 0 on start
        m_drivetrainSubsystem.zeroGyroscope();
        //m_drivetrainSubsystem.zeroRoll();
        
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(m_controller.getLeftY()) * 
        DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * 
        DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getRightX()) * 
        DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));
        
        // Configure the button bindings
        configureButtonBindings();
    }
    
            /**
            * Use this method to define your button->command mappings. Buttons can be created by
            * instantiating a {@link GenericHID} or one of its subclasses ({@link
            * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
            * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
            */
            private void configureButtonBindings() {
                // m_subcontroller.y().onTrue(getAutonomousCommand());
                // m_subcontroller.povDown().onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyroscope(), m_drivetrainSubsystem));

                 m_subcontroller.leftBumper().onTrue(Commands.runOnce(() -> m_armSubsystem.lowerArmPosition(), m_armSubsystem));
                 m_subcontroller.rightBumper().onTrue(Commands.runOnce(() -> m_armSubsystem.raiseArmPosition(), m_armSubsystem));
                 m_subcontroller.axisGreaterThan(XboxController.Axis.kLeftY.value,  m_armSubsystem.DEADBAND).onTrue(new ArmSpeedCommand(() -> m_subcontroller.getLeftY(), m_armSubsystem));
                 m_subcontroller.axisLessThan(   XboxController.Axis.kLeftY.value, -m_armSubsystem.DEADBAND).onTrue(new ArmSpeedCommand(() -> m_subcontroller.getLeftY(), m_armSubsystem));
                
                // !!!SID!!! - added 3/16/23
                m_controller.axisGreaterThan(XboxController.Axis.kRightTrigger.value, m_clawSubsystem.DEADBAND).onTrue(new ClawSpeedCommand(() -> m_controller.getRightTriggerAxis(), m_clawSubsystem));
                m_controller.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, m_clawSubsystem.DEADBAND).onTrue(new ClawSpeedCommand(() -> -m_controller.getLeftTriggerAxis(), m_clawSubsystem));

            }
            
            /**
            * Use this to pass the autonomous command to the main {@link Robot} class.
            *
            * @return the command to run in autonomous
            */
            public Command getAutonomousCommand( ){
                // An ExampleCommand will run in autonomous
                //return new InstantCommand();
               // return new PlatformDockPidCommand_Pitch(m_drivetrainSubsystem);
               m_drivetrainSubsystem.drive ( new ChassisSpeeds(0, 0, 0));
               return new AutonomousDistance(m_drivetrainSubsystem);
            }
            
            private static double deadband(double value, double deadband) {
                if (Math.abs(value) > deadband) {
                    if (value > 0.0) {
                        return (value - deadband) / (1.0 - deadband);
                    } else {
                        return (value + deadband) / (1.0 - deadband);
                    }
                } else {
                    return 0.0;
                }
            }
            
            private static double modifyAxis(double value) {
                // Deadband
                value = deadband(value, 0.05); //sanjay change: Deadband value before 0.05
                
                // Square the axis
                value = Math.copySign(value * value, value);
                
                return value;
            }
        }
        