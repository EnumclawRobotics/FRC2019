/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labArm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import common.instrumentation.*;

/**
 * Use for testing the Arm Assembly
 * Use of xboxcontroller left joystick to move arm
 * Use of xboxcontroller right joystick to move wrist
 */
public class Robot extends TimedRobot {
    Telemetry telemetry = new Telemetry("Robot/LabArm");  

    SpeedController armController;
    SpeedController wristController;
    XboxController xboxController;

    @Override
    public void robotInit() {
        xboxController = new XboxController(1);                                 // Operator USB
        armController = new CANSparkMax(1, MotorType.kBrushless);               // CAN Device ID
        wristController = new Spark(0);                                         // PWM port

        // turn on MOTOR BRAKING
        // turn on MOTOR BRAKING

        ((MotorSafety)armController).setSafetyEnabled(true);
        ((MotorSafety)wristController).setSafetyEnabled(true);
    }

    /*
    * The RobotPeriodic function is called every control packet no matter the
    * robot mode.
    */
    @Override
    public void robotPeriodic() {
        putTelemetry();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        armController.set(xboxController.getY(Hand.kLeft));
        wristController.set(xboxController.getY(Hand.kRight));
        
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putDouble("Arm : getY(Left)", xboxController.getY(Hand.kLeft));
        telemetry.putDouble("Wrist : getY(Right)", xboxController.getY(Hand.kRight));
        telemetry.putString("Version", "1.0.0");
    }
}
