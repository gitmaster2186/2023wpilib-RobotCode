// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;


public class ClawSpeedCommand extends CommandBase {
    ClawSubsystem m_ClawSubsystem;
    DoubleSupplier ySupplier;

    public ClawSpeedCommand(DoubleSupplier axisValue, ClawSubsystem m_ClawSubsystem) {
        this.m_ClawSubsystem = m_ClawSubsystem;
        this.ySupplier = axisValue;
        addRequirements(m_ClawSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // // Called every time the scheduler runs while the command is scheduled.
  // // Prabhu - Can use a pid here for each axis-maybe overkill

    @Override
    public void execute() {
        double axisValue = ySupplier.getAsDouble();
        m_ClawSubsystem.setClawSpeed(axisValue);

    }
    
    @Override
    public void end(boolean interrupted) {
        m_ClawSubsystem.setClawSpeed(0);

    }
    @Override
    public boolean isFinished() {
        return Math.abs(ySupplier.getAsDouble()) < 0.05;
    }

}
