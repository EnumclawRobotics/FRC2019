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
    private PID pid;
    private RampSpeedController speedController;    // controller for moving the joint
//    private CANEncoder2 encoder;                    // counts clicks of rotation for the joint 
    private Encoder encoder;                    // counts clicks of rotation for the joint 

//    private DigitalInput limitSwitch;               // determines when we're going to turn too far
                                                    
    // start condition
    private double baseClicks;                          // where did encoder start at init?  0 is straight up

    // set target
    private double armAngle;                        // arm's angle
    private double angle;                           // wrist angle
    private double targetAngle;                     // based on state determistically, or if horizontal to arm, based on arm angle
    private double targetClicks;                        // encoder clicks that match angle

    // derived but stored for awareness
    private double feedForward;                         // amount of counter gravity force
    private double pidPower;
    private double power;                               // power sent to motor

    States state = States.Stopped;

    public enum States {
        Stopped,                // disable stop
        MovingStowed,           // held in tight?
        MovingAligned,          // wrist aligned with arm into single bar for grabbing cargo off floor
        MovingHorizontal,       // wrist aligned to horizontal
        MovingManual            // manually adjusting
    }

    // Constructor holds onto motor controller and sensor references
    public Wrist(RobotMap robotMap) {
        // invert due to mounting direction
        robotMap.wristSpeedController.setInverted(true);

        this.pid = new PID();
        // this.pidController.setZnGainsP(RobotMap.wristPidKu);
        this.pid.setGainsPID(RobotMap.wristPidKp, RobotMap.wristPidKi, RobotMap.wristPidKd);

        this.speedController = new RampSpeedController(robotMap.wristSpeedController, RobotMap.wristRampFactor);
        this.encoder = robotMap.wristEncoder;
//        this.limitSwitch = robotMap.wristLimitSwitch;
    }

    // assumes that arm is Stowed at start of autonomous 
    public void init(Arm arm) {
        // involved parts
        this.arm = arm;

        // store starting position
        baseClicks = encoder.get();

        // reset derived fields
        targetClicks = baseClicks;                        
        feedForward = 0;
        power = 0;
    }

    // not triggerable by user 
    public void stop() {
        state = States.Stopped;
        power = 0;
        speedController.set(power);
    }

    // === Per Period ===

    public States getState() {
        return state;
    }

    public double getClicks() {
        return encoder.get();
    }

    public double getAngle() {
        return angleFromClicks(encoder.get());
    }

    // hold wrist in close 
    public void moveStowed() {
        state = States.MovingStowed;
        targetAngle = -1;
        targetClicks = (arm.getFacingNormal() ? baseClicks : baseClicks + RobotMap.wristEncoderClicksPerDegree * (360 - 2 * RobotMap.wristStowedAngle));        
    }

    // manually move wrist
    public void moveManual(double controlPower) {
        // ignore deadband and defaults where we are not moving joystick
        if (Math.abs(controlPower) > .01) {
            state = States.MovingManual;
            targetAngle = -1;
            targetClicks = getClicks() + (controlPower * RobotMap.wristEncoderClicksPerDegree);
        }
    }

    // nudges wrist by a percent of an angle. 
    // used when switching from open to close and vice versa to aid with grabbing and placing
    public void nudge(double nudgeAngle) {
        // ignore deadband and defaults where we are not moving joystick
        if (Math.abs(nudgeAngle) > .01) {
            targetClicks = targetClicks + (nudgeAngle * RobotMap.wristEncoderClicksPerDegree);
        }
    }
    
    // keep wrist straight aligned with arm
    public void moveAligned() {
        state = States.MovingAligned;
        targetAngle = 180;
        targetClicks = clicksFromAngle(targetAngle);
    }

    // keep the wrist horizontal compared to the arm angle
    public void moveHorizontal() {
        state = States.MovingHorizontal;
        targetAngle = horizontalAngleFromArm();
        targetClicks = clicksFromAngle(targetAngle);
    }

    public void run() {
        if (state != States.Stopped) {
            // adjust target based on current arm move?
            if (state == States.MovingHorizontal) {
                targetAngle = horizontalAngleFromArm();
                targetClicks = clicksFromAngle(targetAngle);
            }

            // current angle implies how much force gravity applies
            angle = getAngle();
            armAngle = arm.getAngle();
            double gravityAngle = (angle - 180) + armAngle;       // wrist angle is 180 degrees ahead of arm in orientation
            feedForward = Functions.gravity(gravityAngle) * RobotMap.wristFeedForwardFactor;

            // get PID output that is best to go towards the target clicks
            pidPower = pid.update(targetClicks - getClicks(), RobotMap.wristPidLocality);

            // add in bias and reduce the power to the allowed range
            // **** be safe for now until we get the settings right ***
            //power = Functions.clip(feedForward + pidPower, -.22d, .22d);
            power = feedForward + pidPower;

            // is limit switch saying we are going too far?
            // if (limitSwitch.get() && 
            //        ((angle > 180 && power > 0)                  // opposite side too far 
            //         || (angle < 180 && power < 0))) {           // normal side too far
            //     power = 0;
            // }

            // apply the correction to move towards the target
            speedController.set(power);
        }

        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
//        telemetry.putBoolean("LimitSwitch.get()", limitSwitch.get());
        telemetry.putDouble("Arm Angle", armAngle);
        telemetry.putDouble("Wrist Angle", angle);
        telemetry.putDouble("Wrist Clicks", encoder.get());
        telemetry.putDouble("TargetAngle", targetAngle);
        telemetry.putDouble("TargetClicks", targetClicks);
        telemetry.putDouble("PIDPower", pidPower);
        telemetry.putDouble("FeedForward", feedForward);
        telemetry.putDouble("Power", power);
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