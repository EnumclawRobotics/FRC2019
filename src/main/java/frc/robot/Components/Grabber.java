package frc.robot.Components;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import common.instrumentation.Telemetry;
import common.util.Functions;
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

    // involved
    private Wrist wrist;

    private States state = States.Stopped;
    private PID grabberPid;
    private RampSpeedController grabberSpeedController;
    private double grabberPower;

    private Encoder grabberEncoder;
    private double baseClicks;
    private double targetClicks;
    private double stateExpiration;

    RampSpeedController rollerFrontSpeedController;
    RampSpeedController rollerBackSpeedController;
    double rollerPower;

    public enum States {
        Stopped,
        Opening, 
        OpeningHatch, 
        OpeningCargo, 
        Closing,
        Gripping
    }

    public States getState() {
        return state;
    }

    public Grabber(RobotMap robotMap) {
        // when opening the encoder values should go up - requires inverting motor direction
        robotMap.grabberSpeedController.setNeutralMode(NeutralMode.Brake);
        robotMap.grabberSpeedController.setInverted(true);
        robotMap.grabberEncoder.setReverseDirection(true);

        // set intake brake mode and ensure they rotate counter to each other
        robotMap.rollerBackSpeedController.setNeutralMode(NeutralMode.Brake);
        robotMap.rollerFrontSpeedController.setNeutralMode(NeutralMode.Brake);
        robotMap.rollerFrontSpeedController.setInverted(true);

        // grabber        
        grabberSpeedController = new RampSpeedController(robotMap.grabberSpeedController, RobotMap.grabberRampFactor);
        grabberEncoder = robotMap.grabberEncoder;

        // for holding at tight positioning
        grabberPid = new PID();
        grabberPid.setGainsPID(RobotMap.grabberPidKp, RobotMap.grabberPidKi, RobotMap.grabberPidKd);

        // intake
        rollerFrontSpeedController = new RampSpeedController(robotMap.rollerFrontSpeedController, RobotMap.rollerRampFactor);          
        rollerBackSpeedController = new RampSpeedController(robotMap.rollerBackSpeedController, RobotMap.rollerRampFactor);          

        stop();
    }

    public void init(Wrist wrist) {
        // involved
        this.wrist = wrist;

        // assumes closed at start of autonomous
        baseClicks = getClicks();
    }

    // === Executing per Period ===

    public double getClicks() {
        return grabberEncoder.get();
    }

    public void close() {
        if (state != States.Closing) {
            state = States.Closing;
            grabberPower = -RobotMap.grabberPowerLimit;
        }

        // find minimum encoder reading
        if (getClicks() < baseClicks) {
            baseClicks = getClicks();
        }
    }

    public void open() {
        if (state != States.Opening) {
            state = States.Opening;
            grabberPower = RobotMap.grabberPowerLimit;
        }
    }

    // hatch adjustment(s) to help grab and release
    // wrist.nudge(.25);

    // open to cargo-grab size 
    public void openCargo() {
        if (state != States.OpeningCargo) {
            state = States.OpeningCargo;
            targetClicks = baseClicks + (RobotMap.grabberEncoderClicksPerDegree * RobotMap.grabberCargoOpen);
            grabberPower = grabberPid.update(targetClicks - grabberEncoder.get(), RobotMap.grabberLocality, RobotMap.grabberPowerLimit);
        }
    }

    // hold at current position using brake
    public void grip() {
        if (state != States.Gripping) {
            state = States.Gripping;
            grabberPower = 0;
        }
    }

    // // close by time so that we can reset baseline for opening
    // public void close() {
    //     if (state != States.Closing) {
    //         state = States.Closing;
    //         grabberPower = -.5d;
    //         stateExpiration = Timer.getFPGATimestamp() + 1d;
    //         wrist.nudge(-.25);
    //     }
    // }

    // // open by target clicks
    // public void openHatch() {
    //     if (state != States.OpeningHatch) {
    //         state = States.OpeningHatch;
    //         targetClicks = baseClicks + (RobotMap.grabberEncoderClicksPerDegree * RobotMap.grabberHatchOpen);
    //         wrist.nudge(.25);
    //     }
    // }

    // // open by target clicks
    // public void openCargo() {
    //     if (state != States.OpeningCargo) {
    //         state = States.OpeningCargo;
    //         targetClicks = baseClicks + (RobotMap.grabberEncoderClicksPerDegree * RobotMap.grabberCargoOpen);
    //         wrist.nudge(.25);
    //     }
    // }

    // // stay at current position 
    // public void grip() {
    //     if (state != States.Gripping) {
    //         state = States.Gripping;
    //         targetClicks = grabberEncoder.get();
    //     }
    // }

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
        rollerPower = RobotMap.rollerIntake;
    }

    // lazy expell. No shooting just tossing!
    public void expell() {
        rollerPower = RobotMap.rollerExpell;
    }

    // just hold when not intaking or expelling
    public void hold() {
        rollerPower = 0;
    }

    // RUN is only place to set motor values
    public void run() {
        if (state != States.Stopped) {
            grabberSpeedController.set(grabberPower);
            rollerFrontSpeedController.set(rollerPower);
            rollerBackSpeedController.set(rollerPower);
        }
        putTelemetry();
    }

    public void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Grabber Power", grabberPower);
        telemetry.putDouble("Base Clicks", baseClicks);
        telemetry.putDouble("Encoder Clicks", grabberEncoder.get());
        telemetry.putDouble("Target Clicks", targetClicks);
        telemetry.putDouble("Roller Power (intake|expell)", rollerPower);
        telemetry.putString("Version", "1.0.0");
    }
}