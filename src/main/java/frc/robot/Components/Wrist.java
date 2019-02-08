package frc.robot.Components;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;
import common.util.*;

/**
 * Wrist component. Often tries to line up horizontally level at end of arm
 *  DESIGNED TO ONLY APPLY CHANGES IN RUN()
 */
public class Wrist {
    // -- setup and cleanup ===
    private Telemetry telemetry = new Telemetry("Robot/Wrist");

    // physical mechanical parent
    private Arm arm;

    // equipment
    private SpeedController controller;                // controller for moving the joint
    private GenericEncoder encoder;                 // counts clicks of rotation for the joint 
    private DigitalInput limitSwitch;               // determines when we're going to turn too far
                                                    
    // start condition
    double baseClicks = 0;                          // where did encoder start at init?  0 is straight up

    // state set target
    private double targetAngle = 0;                 // based on state determistically, or if horizontal to arm, based on arm angle

    // derived but stored for awareness
    double targetClicks = 0;                        // encoder clicks that match angle
    double feedForward = 0;                         // amount of counter gravity force
    double power = 0;                               // power sent to motor

    States state = States.Stopped;

    public enum States {
        Stopped,                // disable stop
        MovingAligned,          // wrist aligned with arm into single bar for grabbing cargo off floor
        MovingFolded,           // held in tight?
        MovingHorizontal,        // keep wrist aligned to horizontal
        MovingManual             // manually adjusting
    }

    // Constructor holds onto motor controller and sensor references
    public Wrist(RobotMap robotMap, Arm arm) {
        controller = robotMap.wristSpeedController;
        ((MotorSafety)controller).setExpiration(RobotMap.safetyExpiration);
        ((MotorSafety)controller).setSafetyEnabled(true);

        encoder = robotMap.wristEncoder;
    }

    // assumes we will not move the wrist after this and before the start of autonomous 
    public void init() {
        // store starting position
        baseClicks = encoder.get();

        // zero out derived fields
        targetClicks = 0;                        
        feedForward = 0;
        baseClicks = 0;
        power = 0;
    }

    // not triggerable by user 
    public void stop() {
        state = States.Stopped;
        power = 0;
        controller.set(power);
    }

    // === Per Period ===

    public States getState() {
        return state;
    }

    public double getAngle() {
        return angleFromClicks(encoder.get());
    }

    public void moveManual(double move) {
        if (state != States.MovingManual) {
            targetAngle = angleFromClicks(encoder.get());    
        }
        targetAngle += (arm.getFacingNormal() ? move : - move);
        state = States.MovingManual; 
    }

    // keep wrist straight aligned with arm
    public void moveAligned() {
        targetAngle = 180;
        state = States.MovingAligned;
    }

    // hold wrist in close 
    public void moveFolded() {
        targetAngle = (arm.getFacingNormal() ? 360 - RobotMap.wristHeldAngle : RobotMap.wristHeldAngle) ;
        state = States.MovingFolded;
    }

    // keep the wrist horizontal compared to the arm angle
    public void moveHorizontal() {
        targetAngle = horizontalAngleFromArm();
        state = States.MovingHorizontal;
    }

    public void run() {
        if (state != States.Stopped) {
            // adjust target based on arm move?
            if (state == States.MovingHorizontal) {
                targetAngle = horizontalAngleFromArm();
            }

            // current angle implies how much force gravity applies
            double angle = getAngle();
            double armAngle = arm.getAngle();
            double gravityAngle = (angle - 180) + armAngle;       // wrist angle is 180 degrees ahead of arm in orientation
            feedForward = Geometry.gravity(gravityAngle) * RobotMap.wristFeedForwardFactor;

            // moving to an angle
            targetClicks = clicksFromAngle(targetAngle);

            // apply a (P)id error correction
            double error = targetClicks - (double)encoder.get();
            double correctionP = error * RobotMap.wristKpFactor;
            power = Geometry.clip(feedForward + correctionP, -1, 1);

            // is limit switch saying we are going too far?
            if (limitSwitch.get() 
                && ((angle > 180 && power > 0)                  // opposite side too far 
                    || (angle < 180 && power < 0))) {           // normal side too far
                power = 0;
            }

            // apply the correction to move towards the target
            controller.set(power);
        }

        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Angle", getAngle());
        telemetry.putDouble("Clicks", encoder.get());
        telemetry.putDouble("Power", power);
        telemetry.putDouble("TargetAngle", targetAngle);
        telemetry.putDouble("TargetClicks", targetClicks);
        telemetry.putDouble("FeedForward", feedForward);
        telemetry.putString("Version", "1.0.0");
    }

    // === Helpers =========

    // given clicks figure out angle 
    public double angleFromClicks(double clicks) {
        return (((double)(clicks - baseClicks)) / RobotMap.wristEncoderClicksPerDegree);
    }
    
    // given a target angle figure out target clicks  
    public double clicksFromAngle(double angle) {
        return (angle * RobotMap.wristEncoderClicksPerDegree) + baseClicks;
    }

    // returns the horizontal correction angle given the arm angle. 
    public double horizontalAngleFromArm() {
        double angle = 0;

        double armAngle = arm.getAngle(); 
        if (armAngle >= 0 && armAngle < 180) {
            // arm 0-180 - wrist is 180 degrees ahead of arm - but want 90 degree align with horizontal
            angle = 270 - armAngle;
        } else if (armAngle >= 180 && armAngle <= 360){
            // arm 180-360 - wrist is 180 degrees ahead of arm - but want 90 degree align with horizontal
            angle = 450 - armAngle ; 
        }

        return angle;
    }
}