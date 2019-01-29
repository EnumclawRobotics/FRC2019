package frc.robot.Components;

import common.util.*;
import common.i2cSensors.*;
import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.HardwareMap;

public class CargoHandler {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/CargoHandler");
    
    SpeedController rollers;
    MRColorSensor cargoSensor;
    final int[] cargoColor;
    final double cargoColorThreshold;
    double speed;

    States state = States.Stopped;

    public enum States {
        Stopped, 
        Intaking, Expelling,
        Holding
    }

    public States getState() {
        return state;
    }
    
    public CargoHandler(HardwareMap hardwareMap) {
        rollers = hardwareMap.cargoRollerSpeedController;
        cargoSensor = hardwareMap.cargoColorSensor;
        cargoColor = hardwareMap.cargoColor;
        cargoColorThreshold = hardwareMap.cargoColorThreshold;

        stop();
    }

    public void cleanup() {
        stop();
    }

    // === Executing per Period ===
    public void run() {
        rollers.set(speed);
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Speed (intake|expell)", speed);
        telemetry.putDouble("Cargo Held", speed);
        telemetry.putString("Version", "1.0.0");
    }

    public boolean isCargoHeld() {
        return Similarity.isMatch(cargoSensor.getColor(), cargoColor, cargoColorThreshold); 
    }

    // === User Trigerrable States ===
    public void stop() {
        state = States.Stopped;
        speed = 0;
    }

    // full grab. Touch it! Own it!
    public void intake() {
        state = States.Intaking;
        speed = 1;
    }

    // lazy expell. No shooting just tossing!
    public void expell() {
        state = States.Expelling;
        speed = -.3;
    }

    // use a tiny bit of force to hang onto ball without grinding it
    public void hold() {
        state = States.Holding;
        speed = .1d;
    }

    // === Internally Trigerrable States ===
}