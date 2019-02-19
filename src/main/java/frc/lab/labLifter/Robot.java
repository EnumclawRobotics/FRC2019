/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labLifter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;

import common.instrumentation.*;

/**
 * Use for testing the Lifter Assembly
 * Behavior
 * Brake to hold in place. Is feed forward required?
 * Use of StartButton to go into Lifter Mode
 * Joystick left trigger for lifting power
 * Forward motor to roll 6 inches forward
 * Use of Right Bumper to retract forward lift
 * Use of Left Bumper to retract back lift
 * Use of Back Button to leave Lifter Mode 
 */
public class Robot extends TimedRobot {
    Telemetry telemetry = new Telemetry("Robot/LabLifter");  

    SpeedController motorControllerFront;
    SpeedController motorControllerBack;
    SpeedController motorControllerMove;
    XboxController xboxController;

    boolean lifterModeEnabled;
    boolean lifting;
    boolean retractingFront;
    boolean retractingBack;

    @Override
    public void robotInit() {
        xboxController = new XboxController(0);         // USB
        motorControllerFront = new Spark(0);            // PWM port
        motorControllerBack = new Spark(1);             // PWM port
        motorControllerMove = new Spark(2);             // PWM port

        // turn on MOTOR BRAKING
        // turn on MOTOR BRAKING

        ((MotorSafety)motorControllerFront).setSafetyEnabled(true);
        ((MotorSafety)motorControllerBack).setSafetyEnabled(true);
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
        lifterModeEnabled = false;
    }

    @Override
    public void teleopPeriodic() {
        if (xboxController.getStartButton()) {
            lifterModeEnabled = true;
            lifting = true;
            retractingFront = false;
            retractingBack = false;
        }

        if (xboxController.getBackButton()) {
            lifterModeEnabled = false;
            lifting = false;
            retractingFront = false;
            retractingBack = false;
        }

        if (lifting) {
            if (xboxController.getAButton()) {
                retractingFront = true;
            }

            if (xboxController.getBButton()) {
                retractingFront = false;
                retractingBack = true;
            }

            if (xboxController.getTriggerAxis(Hand.kLeft) > 0) {
                if (!retractingFront) {
                    motorControllerFront.set(xboxController.getTriggerAxis(Hand.kLeft));
                } else {
                    motorControllerFront.set(-.10);
                }
                if (!retractingBack) {
                    motorControllerBack.set(xboxController.getTriggerAxis(Hand.kLeft));
                } else {
                    motorControllerBack.set(-.10);
                }
            }

            if (!retractingBack) {
                motorControllerMove.set(1d);
            } else {
                motorControllerMove.stopMotor();
            }

            putTelemetry();
        }
    }

    private void putTelemetry() {
        telemetry.putBoolean("Lifting Mode (Start)", lifting);
        telemetry.putDouble("XboxController.getTrigger(Left)", xboxController.getTriggerAxis(Hand.kLeft));
        telemetry.putBoolean("Retracting Front", retractingFront);
        telemetry.putBoolean("Retracting Back", retractingBack);
        telemetry.putString("Version", "1.0.0");
    }
}
