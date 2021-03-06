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
import edu.wpi.first.wpilibj.VictorSP;
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
    XboxController xboxController;
    double power;

    @Override
    public void robotInit() {
        xboxController = new XboxController(0);                 // USB
        motorController1 = new VictorSP(0);             // PWM port
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
        power = -xboxController.getY(Hand.kLeft);
        motorController1.set(power);
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putDouble("XboxController.getY(left)", power);
        telemetry.putString("Version", "1.0.0");
    }
}
