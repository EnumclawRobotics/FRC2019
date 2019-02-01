package frc.robot.Components;

import common.instrumentation.Telemetry;
import common.i2cSensors.*;
import common.util.*;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;

/**
 * 'Hand' that closes and opens grippers to hold the Cargo or Hatch
 * Assumes a position mechanical stop
 * Timebased opening and closing behavior
 * 
 * DESIGNED TO ONLY APPLY CHANGES IN RUN()
 */
public class Grabber {
    // === setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Grabber");

    SpeedController grabber;
    DigitalInput hatchLimitSwitch;

    SpeedController rollers;
    double rollerSpeed;

    MRColorSensor cargoSensor;
    final int[] cargoColor;
    final double cargoColorThreshold;

    double speed;
    double power;

    States state = States.Stopped;
    double expireCommand;

    public enum States {
        Stopped, Opening, HoldingOpen, Closing, HoldingClose
    }

    public States getState() {
        return state;
    }

    public boolean isHatchHeld() {
        return (hatchLimitSwitch.get());
    }

    public boolean isCargoHeld() {
        return Similarity.isMatch(cargoSensor.getColor(), cargoColor, cargoColorThreshold); 
    }

    public Grabber(RobotMap robotMap) {
        grabber = robotMap.grabberSpeedController;

        hatchLimitSwitch = robotMap.hatchLimitSwitch;
        
        rollers = robotMap.cargoRollerSpeedController;
        cargoSensor = robotMap.cargoColorSensor;
        cargoColor = FieldMap.cargoColor;
        cargoColorThreshold = FieldMap.cargoColorThreshold;

        stop();
    }

    // === Executing per Period ===

    public void run() {
        // update based on time based expiration of command
        switch (state) {
        case Stopped: 
            // nothing to do
            break;
        case Opening:
            if (Timer.getFPGATimestamp() > expireCommand) {
                holdOpen();
            }
            break;
        case HoldingOpen: 
            // nothing to do keep holding
            break;
        case Closing:
            if (Timer.getFPGATimestamp() > expireCommand) {
                holdClose();
            }
            break;
        case HoldingClose: 
            // nothing to do keep holding
            break;
        }

        grabber.set(power);

        rollers.set(speed);

        putTelemetry();
    }

    public void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Speed (intake|expell)", speed);
        telemetry.putDouble("Cargo Held", speed);
        telemetry.putBoolean("Hatch Held", isHatchHeld());
        telemetry.putString("Version", "1.0.0");
    }

    // === User Trigerrable States ===
    public void stop() {
        state = States.Stopped;
        power = 0;
    }

    public void open() {
        if (state != States.Opening) {
            state = States.Opening;
            expireCommand = Timer.getFPGATimestamp() + .5;      // time to complete action
            power = -1d;
        }
    }

    public void close() {
        if (state != States.Closing) {
            state = States.Closing;
            expireCommand = Timer.getFPGATimestamp() + .5;      // time to complete action
            power = 1d;
        }
    }

    // === Internal Triggerable States ====
    public void holdOpen() {
        if (state != States.HoldingOpen) {
            state = States.HoldingOpen;
            expireCommand = Timer.getFPGATimestamp();       // set to now so other commands can go immediately
            power = -.5d;
        }
    }

    public void holdClose() {
        if (state != States.HoldingClose) {
            state = States.HoldingClose;
            expireCommand = Timer.getFPGATimestamp();       // set to now so other commands can go immediately
            power = .5d;
        }
    }

    // full grab. Touch it! Own it!
    public void intake() {
        rollerSpeed = 1d;
    }

    // lazy expell. No shooting just tossing!
    public void expell() {
        rollerSpeed = -.3d;
    }

    // use a tiny bit of force to hang onto ball without grinding it
    public void grip() {
        rollerSpeed = .1d;
    }
}