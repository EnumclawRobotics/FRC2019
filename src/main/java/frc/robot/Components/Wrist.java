package frc.robot.Components;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;

/**
 * Wrist portion tries to line up hrizontally
 *  DESIGNED TO ONLY APPLY CHANGES IN RUN()
 */
public class Wrist {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Wrist");

    PWMSpeedController wrist;             
    Encoder encoder; 
    DigitalInput limitSwitch;
    
    double targetAngle = 0;
    double feedForward = 0;
    double speed = 0;

    boolean facingNormal = true; 

    int inverted = 0;

    States state = States.Stopped;

    public enum States {
        Stopped, Moving, Holding
    }

    public States getState() {
        return state;
    }
    
    public Wrist(RobotMap robotMap) {
        wrist = robotMap.wristSpeedController;
        wrist.setExpiration(RobotMap.safetyExpiration);
        wrist.setSafetyEnabled(true);

        encoder = robotMap.wristEncoder;

        stop();
    }

    // === Executing per Period ===

    public void setFacing(boolean facingNormal) {
        this.facingNormal = facingNormal;
    }

    public double getCurrentAngle() {
        return encoder.getDistance();   // TODO: what is this unit? how to turn into desired unit? degrees? clicks? radians?
    }

    // attempts to set the wrist level compared to the arm angle
    public void setLevel(double shoulderAngle) {
        // TODO:
    }

    public void run() {
        // always ensure that we never use rotation. i.e. both shoulder motors should move as one not try to fight each other
        wrist.set(speed);
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putBoolean("Arm Facing (normal)", facingNormal);
        telemetry.putString("State", state.toString());
        telemetry.putDouble("CurrentAngle", getCurrentAngle());
        telemetry.putDouble("TargetAngle", targetAngle);
        telemetry.putDouble("FeedForward", RobotMap.wristFeedFowardFactor);
        telemetry.putDouble("Speed (forward|back)", speed);
        telemetry.putString("Version", "1.0.0");
    }

    // === User Trigerrable States ===
    public void stop() {
        state = States.Stopped;
        speed = 0;
    }

    // whenever we arent moving use PID controller to hold at desired height
    public void hold() {
        state = States.Holding;
        feedForward = Math.cos(targetAngle) * RobotMap.wristFeedFowardFactor;        // power is a function of angle given everything else
    }

    // === Internally Trigerrable States ===

    // === helpers ===

    // getting a level wrist angle in radians from an arm angle
    public static double wristAngleFromArm(double armAngleRadians) {
        // 90 degrees - arm angle = correction to get a level angle
        return (Math.PI/2) - armAngleRadians;
    }
}