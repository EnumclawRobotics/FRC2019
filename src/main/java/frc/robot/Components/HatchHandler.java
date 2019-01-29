package frc.robot.Components;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.HardwareMap;

public class HatchHandler {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/HatchHandler");

    Grabber grabber;
    DigitalInput hatchALimitSwitch;
    DigitalInput hatchBLimitSwitch;

    States state = States.Stopped;

    public enum States {
        Stopped, 
        Readying, Intaking, Holding, Placing, Placed
    }

    public States getState() {
        return state;
    }
    
    public HatchHandler(HardwareMap hardwareMap, Grabber grabber) {
        this.grabber = grabber;
        hatchALimitSwitch = hardwareMap.hatchALimitSwitch;
        hatchBLimitSwitch = hardwareMap.hatchBLimitSwitch;
        stop();
    }

    // === Executing per Period ===
    public void run() {
        switch (state) {
        case Stopped: 
            break;
        case Readying:
            break;
        case Intaking:
            if (isHatchHeld() && grabber.state == Grabber.States.HoldingOpen) {
                hold();
            }
            break;
        case Holding:
            break;
        case Placing:
            if (!isHatchHeld() && grabber.state == Grabber.States.HoldingClose) {
                place();
            }
            break;
        case Placed:
            break;
        }

        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putBoolean("Hatch Held", isHatchHeld());
        telemetry.putString("Version", "1.0.0");
    }

    public boolean isHatchHeld() {
        return (hatchALimitSwitch.get() || hatchBLimitSwitch.get()); 
    }

    // === User Trigerrable States ===
    public void stop() {
        state = States.Stopped;
    }

    // close grabber to get ready for taking hatch
    public void ready() {
        state = States.Readying;
        if (grabber.getState() != Grabber.States.Closing && grabber.getState() != Grabber.States.HoldingClose) {
            grabber.close();
        }
    }

    // open grabber to take hatch - hopefully limit switch says we are successful
    public void intake() {
        state = States.Intaking;
        if (grabber.getState() != Grabber.States.Opening && grabber.getState() != Grabber.States.HoldingOpen) {
            grabber.open();  // TODO: Do we need a softer open than for cargo?
        }
    }

    // use a tiny bit of open force to hang onto hatch without grinding it
    public void hold() {
        state = States.Holding;
    }

    // lazy expell. No shooting just tossing!
    public void place() {
        state = States.Placing;
    }

    // TODO: Combine grabber and handlers?


    // === Internally Trigerrable States ===
}