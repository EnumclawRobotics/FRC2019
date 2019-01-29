package frc.robot.Components;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.*;
import frc.robot.HardwareMap;

/**
 * Wrist portion tries to line up hrizontally
 *  DESIGNED TO ONLY APPLY CHANGES IN RUN()
 */
public class Wrist {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Drive");

    PWMSpeedController rotator;             
    Encoder encoder; 
    DigitalInput limitSwitch;
    
    double targetAngle = 0;
    double targetPosition = 0;
    double feedForward = 0;
    double speed = 0;

    boolean facingNormal = true; 

    int inverted = 0;

    States state = States.Stopped;

    public enum States {
        Stopped, 
        MovingCargoFloor, MovingCargo1, MovingCargo2, MovingCargo3, MovingCargoShip, MovingHatch1, MovingHatch2, MovingHatch3,
        Holding
    }

    public States getState() {
        return state;
    }
    
    public Wrist(HardwareMap hardwareMap) {
        rotator = hardwareMap.wristSpeedController;
        rotator.setExpiration(hardwareMap.safetyExpiration);
        rotator.setSafetyEnabled(true);

        encoder = hardwareMap.wristEncoder;
        stop();
    }

    // === Executing per Period ===

    public void setFacing(boolean facingNormal) {
        this.facingNormal = facingNormal;
    }

    public void run() {
        // always ensure that we never use rotation. i.e. both shoulder motors should move as one not try to fight each other
        rotator.set(speed);
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putBoolean("Arm Facing (normal)", facingNormal);
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Angle", speed);
        telemetry.putDouble("FeedForward", speed);
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

        feedForward = Math.cos(targetAngle);        // power is a function of angle given everything else


    }

    // === Internally Trigerrable States ===

}