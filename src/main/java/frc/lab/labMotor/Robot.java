/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labMotor;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;

import common.instrumentation.*;

/**
 * Use for testing a Motor with a built in Encoder
 * Joystick forward and back should turn it
 */
public class Robot extends TimedRobot {
    Telemetry telemetry = new Telemetry("Robot/LabMotor");  

    SpeedController motorController1;
    SpeedController motorController2;
    XboxController xboxController;

    @Override
    public void robotInit() {
        xboxController = new XboxController(0);                 // USB
        motorController1 = new Spark(0);             // PWM port
        motorController2 = new Spark(1);             // PWM port
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
        motorController1.set(Math.signum(xboxController.getY(Hand.kLeft)) * Math.pow(xboxController.getY(Hand.kLeft),2));
        motorController2.set(Math.signum(xboxController.getY(Hand.kRight)) * Math.pow(xboxController.getY(Hand.kRight),2));
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putDouble("XboxController.getY(left)^2", Math.signum(xboxController.getY(Hand.kLeft)) * Math.pow(xboxController.getY(Hand.kLeft),2));
        telemetry.putDouble("XboxController.getY(right)^2", Math.signum(xboxController.getY(Hand.kRight)) * Math.pow(xboxController.getY(Hand.kRight),2));
        telemetry.putString("Version", "1.0.0");
    }
}
