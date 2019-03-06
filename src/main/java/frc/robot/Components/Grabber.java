package frc.robot.Components;

import common.instrumentation.Telemetry;
import common.util.Geometry;
import common.util.PID;
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
    PID grabberPid = new PID(.02, .002, .08);
    SpeedController grabberSpeedController;
    double grabberPower;

    Encoder grabberEncoder;
    double baseClicks;
    double targetClicks;

    SpeedController rollerFrontSpeedController;
    SpeedController rollerBackSpeedController;
    double rollerPower;

    public enum States {
        Stopped, OpeningHatch, OpeningCargo, Closing, Gripping
    }

    public States getState() {
        return state;
    }

    public Grabber(RobotMap robotMap) {
        grabberSpeedController = robotMap.grabberSpeedController;
        grabberEncoder = robotMap.grabberEncoder;

        rollerFrontSpeedController = robotMap.rollerFrontSpeedController;          
        rollerBackSpeedController = robotMap.rollerBackSpeedController;          

        stop();
    }

    public void init() {
        baseClicks = getClicks();
    }

    // === Executing per Period ===

    public double getClicks() {
        return grabberEncoder.get();
    }

    public void openHatch() {
        this.state = States.OpeningHatch;
        targetClicks = baseClicks + (RobotMap.grabberEncoderClicksPerDegree * RobotMap.grabberHatchOpen);
    }

    public void openCargo() {
        this.state = States.OpeningCargo;
        targetClicks = baseClicks + (RobotMap.grabberEncoderClicksPerDegree * RobotMap.grabberCargoOpen);
    }

    public void grip() {
        this.state = States.Gripping;
        targetClicks = baseClicks;
    } 

    public void close() {
        state = States.Closing;
        targetClicks = baseClicks;
    }

    public void stop() {
        state = States.Stopped;
        grabberPower = 0d;
        rollerPower = 0d;

        grabberSpeedController.stopMotor();
        rollerFrontSpeedController.stopMotor();
        rollerBackSpeedController.stopMotor();
    }

    // full grab. Touch it! Own it!
    public void intake() {
        rollerPower = RobotMap.grabberIntake;
    }

    // lazy expell. No shooting just tossing!
    public void expell() {
        rollerPower = RobotMap.grabberExpell;
    }

    // just hold when not intaking or expelling
    public void hold() {
        rollerPower = 0;
    }

    // RUN is only place to set motor values
    public void run() {
        if (state != States.Stopped) {
            grabberPower = grabberPid.update(targetClicks, getClicks());
            grabberPower = Geometry.clip(grabberPower, -.75d, .75d);

            grabberSpeedController.set(grabberPower);
            rollerFrontSpeedController.set(rollerPower);
            rollerBackSpeedController.set(rollerPower);
        }
        putTelemetry();
    }

    public void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Grabber Power", grabberPower);
        telemetry.putDouble("Roller Power (intake|expell)", rollerPower);
        telemetry.putString("Version", "1.0.0");
    }
}