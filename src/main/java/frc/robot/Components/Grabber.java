package frc.robot.Components;

import common.instrumentation.*;
import edu.wpi.first.wpilibj.*;
import frc.robot.HardwareMap;

/**
 * 'Hand' that closes and opens grippers to hold the Cargo or Hatch
 * Assumes a position mechanical stop
 * Timebased opening and closing behavior
 */
public class Grabber {
    // === setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Grabber");

    SpeedController grabber;
    double power;

    States state = States.Stopped;
    double expireCommand;

    public enum States {
        Stopped, Opening, HoldingOpen, Closing, HoldingClose
    }

    public States getState() {
        return state;
    }

    public Grabber(HardwareMap hardwareMap) {
        grabber = hardwareMap.grabberSpeedController;
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
}