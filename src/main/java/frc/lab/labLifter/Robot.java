/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labLifter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;

import common.instrumentation.*;
import common.util.Geometry;

/**
 * Use for testing the Lifter Assembly
 * Behavior
 * Brake to hold in place. Is feed forward required?
 * Use of StartButton to go into Lifter Mode
 * left pov for lifting power up and down 
 * forward motor to roll 6 inches forward - dampened power on actual wheels
 * Use of X to only do forward lift
 * Use of Y to only do back lift
 * Use of Back Button to leave Lifter Mode 
 */
public class Robot extends TimedRobot {
    Telemetry telemetry = new Telemetry("Robot/LabLifter");  

    SpeedController motorControllerFront;
    SpeedController motorControllerBack;
    SpeedController motorControllerMove;
    XboxController xboxController;

    boolean lifterModeEnabled;
    
    boolean liftingFront;
    double liftingFrontPower;

    boolean liftingBack;
    double liftingBackPower;
    
    double liftGravity = .25d;

    double moverPower = .25d;

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
            liftingFront = true;
            liftingBack = true;
        }

        if (xboxController.getBackButton()) {
            lifterModeEnabled = false;
            liftingFront = false;
            liftingBack = false;
            motorControllerFront.stopMotor();
            motorControllerBack.stopMotor();
            motorControllerMove.stopMotor();
        }

        if (lifterModeEnabled) {
            if (xboxController.getYButton()) {
                liftingFront = true;
                liftingBack = false;
            }

            if (xboxController.getXButton()) {
                liftingFront = false;
                liftingBack = true;
            }

            liftingFrontPower = liftingFront ? Geometry.getYFromAngle(xboxController.getPOV()) : 0;
            liftingBackPower = liftingBack ? Geometry.getYFromAngle(xboxController.getPOV()) : 0;

            motorControllerFront.set(Geometry.clip(liftingFrontPower + liftGravity, -1, 1));
            motorControllerBack.set(Geometry.clip(liftingFrontPower + liftGravity, -1, 1));
            motorControllerMove.set(moverPower);

            // try to roll forward while lifting enabled 
            motorControllerMove.set(1d);
        }

        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putBoolean("Lifting Front", liftingFront);
        telemetry.putDouble("Lifting Front Power", liftingFrontPower);
        telemetry.putBoolean("Lifting Back", liftingBack);
        telemetry.putDouble("Lifting Back Power", liftingBackPower);
        telemetry.putString("Version", "1.0.0");
    }
}
