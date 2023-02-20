// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.*;
import frc.robot.ConstantsConfiguration;

// Yaw, pitch, speeds

public class SmartDashboardConfig 
{
    private GenericEntry entries;

    private final double kP = ConstantsConfiguration.kP;
    private final double kI = ConstantsConfiguration.kI;
    private final double kD = ConstantsConfiguration.kD;

    ShuffleboardTab driverTab = Shuffleboard.getTab("PID Control");
    
    GenericEntry kPParam = driverTab.add("kP", kP).withWidget("Number Slider").getEntry();
    GenericEntry kIParam = driverTab.add("kI", kI).withWidget("Number Slider").getEntry();
    GenericEntry kDParam = driverTab.add("kD", kD).withWidget("Number Slider").getEntry();

    public void readWidgetValue(String type){
        if(type == "kP"){
            kPParam.getDouble(0);
        }
        else if(type == "kI"){
            kIParam.getDouble(0);
        }
        else if(type == "kD"){
            kDParam.getDouble(0);
        }
    }
}

