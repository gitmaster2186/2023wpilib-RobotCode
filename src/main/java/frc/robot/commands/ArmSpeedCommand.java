// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ArmSpeedCommand extends CommandBase {
    ArmSubsystem m_ArmSubsystem;
    DoubleSupplier ySupplier;

    public ArmSpeedCommand(DoubleSupplier armSpeedCommand, ArmSubsystem m_ArmSubsystem) {
        this.m_ArmSubsystem = m_ArmSubsystem;
        this.ySupplier = armSpeedCommand;
        addRequirements(m_ArmSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // // Called every time the scheduler runs while the command is scheduled.
  // // Prabhu - Can use a pid here for each axis-maybe overkill

    @Override
    public void execute() {
        double axisValue = ySupplier.getAsDouble();
        m_ArmSubsystem.setArmSpeed(axisValue);

    }
    
    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setArmSpeed(0);

    }
    @Override
    public boolean isFinished() {
        return Math.abs(ySupplier.getAsDouble()) < m_ArmSubsystem.DEADBAND;
    }

}
