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
    Encoder grabberEncoder;
    DigitalInput hatchLimitSwitch;

    SpeedController rollers;
    double rollerPower;

    MRColorSensor cargoSensor;
    final int[] cargoColor;
    final double cargoColorThreshold;

    States state = States.Stopped;
    int baseClicks;                                      // assumes closed at robotStart
    int targetClicks;
    double power;

    public enum States {
        Stopped, OpeningHatch, OpeningCargo, Closing
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
        grabberEncoder = robotMap.grabberEncoder;
        baseClicks = grabberEncoder.get();

        hatchLimitSwitch = robotMap.hatchLimitSwitch;
        
        rollers = robotMap.cargoRollerSpeedController;          
        cargoSensor = robotMap.cargoColorSensor;
        cargoColor = FieldMap.cargoColor;
        cargoColorThreshold = FieldMap.cargoColorThreshold;

        stop();
    }

    // === Executing per Period ===
    private int clicksPerOpeningSize(double opening) {
        double sine = (opening/2)/RobotMap.grabberLength;
        return (int)(Math.toDegrees(Math.asin(sine)) * RobotMap.grabberEncoderClicksPerDegree);
    }
    
    public void openHatch() {
        this.state = States.OpeningHatch;
        targetClicks = baseClicks + clicksPerOpeningSize(FieldMap.hatchHoleDiameter);
    }

    public void openCargo() {
        this.state = States.OpeningCargo;
        targetClicks = baseClicks + clicksPerOpeningSize(FieldMap.cargoDiameter);
    }

    public void close() {
        state = States.Stopped;
        targetClicks = baseClicks;
    }

    public void stop() {
        state = States.Stopped;
        power = 0;
        rollerPower = 0;
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
            int error = grabberEncoder.get() - targetClicks;
            power = Geometry.clip(error * .5d, -1, 1) ;          // PID  

            grabber.set(power);
            rollers.set(rollerPower);
            putTelemetry();
        }
    }

    public void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Base Clicks", baseClicks);
        telemetry.putDouble("Target Clicks", targetClicks);
        telemetry.putDouble("Grabber Clicks", grabberEncoder.get());
        telemetry.putDouble("Grabber Power", power);
        telemetry.putDouble("Roller Power (intake|expell)", rollerPower);
        telemetry.putBoolean("Cargo Held", isCargoHeld());
        telemetry.putBoolean("Hatch Held", isHatchHeld());
        telemetry.putString("Version", "1.0.0");
    }
}