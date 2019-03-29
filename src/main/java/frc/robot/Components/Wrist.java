package frc.robot.Components;

import com.revrobotics.CANSparkMax.IdleMode;

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
    private CANEncoder2 encoder;                    // counts clicks of rotation for the joint 
//    private Encoder encoder;                    // counts clicks of rotation for the joint 

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

    private boolean isInited = false;

    public enum States {
        Stopped,
        MovingFront,
        MovingBack,
        MovingStowed,
        MovingRocketHatch1, MovingRocketHatch2, 
        MovingDepotCargo, 
        MovingStationCargo,
        MovingRocketCargo1, MovingRocketCargo2,  
        MovingShipCargo, 
        MovingManual,
        HoldingManual,
    }

    // Constructor holds onto motor controller and sensor references
    public Wrist(RobotMap robotMap) {
        // config motor controllers
        // NOTE: zero angle is straight up 
        // wrist uses positive clicks to rotate from stowed at top to straight out to folded under 
        //     and so needs to match reversed right hand side direction from drive. 
        robotMap.wristSpeedController.setIdleMode(IdleMode.kBrake);
        robotMap.wristSpeedController.setInverted(true);

        this.pid = new PID();
        //this.pid.setGainsPID(RobotMap.wristPidKp, RobotMap.wristPidKi, RobotMap.wristPidKd);
        this.pid.setNoOvershootGainsPID(RobotMap.wristPidKp, .3d);

        this.speedController = new RampSpeedController(robotMap.wristSpeedController, RobotMap.wristRampFactor);
        this.encoder = robotMap.wristEncoder;
        this.encoder.setPositionConversionFactor(RobotMap.wristEncoderConversionFactor);
//        this.limitSwitch = robotMap.wristLimitSwitch;
    }

    // assumes that arm is Stowed at start of autonomous 
    public void init(Arm arm) {
        if (!isInited) {
            // involved systems
            this.arm = arm;

            // store starting position
            baseClicks = encoder.get();

            // reset derived fields
            targetClicks = baseClicks;      // start angle is all the way at the end of the rotation 360 - stowed angle   
            feedForward = 0;
            power = 0;

            isInited = true;
        }
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
        targetAngle = arm.getFacingNormal() ? RobotMap.wristAngleStowed : (360 - RobotMap.wristAngleStowed);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // moving to the front side
    public void moveFront() {
        state = States.MovingFront;
        targetAngle = RobotMap.wristStowedAngle;
        targetClicks = clicksFromAngle(targetAngle);
    }

    // moving to the back side 
    public void moveBack() {
        state = States.MovingBack;
        targetAngle = (360 - RobotMap.wristStowedAngle);
        targetClicks = clicksFromAngle(targetAngle);
    }
    
    // go for cargo on floor
    public void moveDepotCargo() {
        state = States.MovingDepotCargo;
        targetAngle = arm.getFacingNormal() ? RobotMap.wristAngleDepotCargo : (360 - RobotMap.wristAngleDepotCargo);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // go for cargo level 1
    public void moveRocketCargo1() {
        state = States.MovingRocketCargo1;
        targetAngle = arm.getFacingNormal() ? RobotMap.wristAngleRocketCargo1 : (360 - RobotMap.wristAngleRocketCargo1);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // go for cargo level 2
    public void moveRocketCargo2() {
        state = States.MovingRocketCargo2;
        targetAngle = arm.getFacingNormal() ? RobotMap.wristAngleRocketCargo2 : (360 - RobotMap.wristAngleRocketCargo2);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // go for cargo station
    public void moveStationCargo() {
        state = States.MovingStationCargo;
        targetAngle = arm.getFacingNormal() ? RobotMap.wristAngleStationCargo : (360 - RobotMap.wristAngleStationCargo);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // go for cargo ship
    public void moveShipCargo() {
        state = States.MovingShipCargo;
        targetAngle = arm.getFacingNormal() ? RobotMap.wristAngleShipCargo : (360 - RobotMap.wristAngleShipCargo);
        targetClicks = clicksFromAngle(targetAngle);
    }
    
    // go for hatch level 1
    public void moveRocketHatch1() {
        state = States.MovingRocketHatch1;
        targetAngle = arm.getFacingNormal() ? RobotMap.wristAngleRocketHatch1 : (360 - RobotMap.wristAngleRocketHatch1);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // go for hatch level 2
    public void moveRocketHatch2() {
        state = States.MovingRocketHatch2;
        targetAngle = arm.getFacingNormal() ? RobotMap.wristAngleRocketHatch2 : (360 - RobotMap.wristAngleRocketHatch2);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // manually move wrist
    public void moveManual(double controlPower) {
        // ignore deadband and defaults where we are not moving joystick
        if (Math.abs(controlPower) > .05d) {
            state = States.MovingManual;
            targetAngle = -1;
            // positive from joystick is decrease in angle from baseline
            targetClicks = getClicks() + (-controlPower * RobotMap.wristPidLocality);
        }
        // were we moving manual and now we hae let up on joystick? try holding
        else if (state == States.MovingManual) {
            state = States.HoldingManual;
            targetClicks = getClicks();
        }
    }

    // // nudges wrist by a percent of an angle. 
    // // used when switching from open to close and vice versa to aid with grabbing and placing
    // public void nudge(double nudgeAngle) {
    //     // ignore deadband and defaults where we are not moving joystick
    //     if (Math.abs(nudgeAngle) > .01) {
    //         targetClicks = targetClicks + (nudgeAngle * RobotMap.wristEncoderClicksPerDegree);
    //     }
    // }
    
    public void run() {
        // has arm moved lower thn expected baseline? then reset baseline
        if (getClicks() < baseClicks) {
            baseClicks = getClicks();
        }

        if (state != States.Stopped) {
            // current angle implies how much force gravity applies
            angle = getAngle();
            armAngle = arm.getAngle();
            double gravityAngle = (angle - 180) + armAngle;       // wrist angle is 180 degrees ahead of arm in orientation
            feedForward = Functions.gravity(gravityAngle) * RobotMap.wristFeedForwardFactor;

            if (state == States.HoldingManual) {
                pidPower = 0;
                power = 0;
            }
            else {
                // get PID output that is best to go towards the target clicks
                pidPower = pid.update(targetClicks - getClicks(), RobotMap.wristPidLocality, RobotMap.wristPowerLimit);

                // add in bias
                power = feedForward + pidPower;

                // is limit switch saying we are going too far?
                // if (limitSwitch.get() && 
                //        ((angle > 180 && power > 0)                  // opposite side too far 
                //         || (angle < 180 && power < 0))) {           // normal side too far
                //     power = 0;
                // }
            }

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
        // baseClicks is 360 - StowedAngle 
        // subtract off from there. 

        return (((double)(clicks - baseClicks)) / RobotMap.wristEncoderClicksPerDegree);
    }
    
    // given a target angle figure out target clicks  
    public double clicksFromAngle(double angle) {
        return (angle * RobotMap.wristEncoderClicksPerDegree) + baseClicks;
    }

    // // returns the horizontal correction angle given the arm angle. 
    // public double horizontalAngleFromArm() {
    //     double angle = 0;

    //     double armAngle = arm.getAngle(); 
    //     if (armAngle >= 0 && armAngle < 180) {
    //         // arm 0-180 - wrist is 180 degrees ahead of arm - but want 90 degree align with horizontal
    //         angle = 270 - armAngle;
    //     } else if (armAngle >= 180 && armAngle <= 360){
    //         // arm 180-360 - wrist is 180 degrees ahead of arm - but want 90 degree align with horizontal
    //         angle = 450 - armAngle ; 
    //     }

    //     return angle;
    // }
}