package frc.robot.Components;

import common.instrumentation.Telemetry;
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

    States state = States.Stopped;
    SpeedController grabber;
    double grabberPower;
    double moveExpiration = 0;

    SpeedController rollerMaroonSpeedController;
    SpeedController rollerBlackSpeedController;
    double rollerPower;

    public enum States {
        Stopped, OpeningHatch, OpeningCargo, Closing
    }

    public States getState() {
        return state;
    }

    public Grabber(RobotMap robotMap) {
        grabber = robotMap.grabberSpeedController;

        rollerMaroonSpeedController = robotMap.rollerMaroonSpeedController;          
        rollerBlackSpeedController = robotMap.rollerBlackSpeedController;          

        stop();
    }

    // === Executing per Period ===
    public void openHatch() {
        this.state = States.OpeningHatch;
        grabberPower = .5d;
        moveExpiration = Timer.getFPGATimestamp() + .5;
    }

    public void openCargo() {
        this.state = States.OpeningCargo;
        grabberPower = .5d;
        moveExpiration = Timer.getFPGATimestamp() + .75;
    }

    public void close() {
        state = States.Stopped;
        grabberPower = -.5d;
        moveExpiration = Timer.getFPGATimestamp() + .75;
    }

    public void stop() {
        state = States.Stopped;
        grabberPower = 0d;
        rollerPower = 0d;
    }

    // full grab. Touch it! Own it!
    public void intake() {
        rollerPower = 1d;
    }

    // lazy expell. No shooting just tossing!
    public void expell() {
        rollerPower = -.3d;
    }

    // RUN is only place to set motor values
    public void run() {
        if (state != States.Stopped) {
            if (Timer.getFPGATimestamp() > moveExpiration) {
                grabberPower = .1d;    // holding power
            }

            grabber.set(grabberPower);
            rollerMaroonSpeedController.set(rollerPower);
            rollerBlackSpeedController.set(rollerPower);
            
            putTelemetry();
        }
    }

    public void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Grabber Power", grabberPower);
        telemetry.putDouble("Roller Power (intake|expell)", rollerPower);
        telemetry.putString("Version", "1.0.0");
    }
}