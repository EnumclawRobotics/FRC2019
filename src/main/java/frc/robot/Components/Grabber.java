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
    private SpeedController grabberSpeedController;
    private double grabberPower;

    private Encoder grabberEncoder;
    private double baseClicks;
    private double targetClicks;
    private double stateStart;
    private double stateExpiration;

    SpeedController rollerFrontSpeedController;
    SpeedController rollerBackSpeedController;
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
        // when opening encoder values should go up - requires inverting motor direction
        robotMap.grabberSpeedController.setInverted(true);
        robotMap.grabberEncoder.setReverseDirection(true);

        // set intake brake mode
        robotMap.rollerFrontSpeedController.setNeutralMode(NeutralMode.Brake);
        robotMap.rollerFrontSpeedController.setInverted(true);
        robotMap.rollerBackSpeedController.setNeutralMode(NeutralMode.Brake);

        // grabber        
        grabberSpeedController = robotMap.grabberSpeedController;
        grabberEncoder = robotMap.grabberEncoder;

        grabberPid = new PID();
        grabberPid.setGainsPID(RobotMap.grabberPidKp, RobotMap.grabberPidKi, RobotMap.grabberPidKd);

        // intake
        rollerFrontSpeedController = robotMap.rollerFrontSpeedController;          
        rollerBackSpeedController = robotMap.rollerBackSpeedController;          

        stop();
    }

    public void init(Wrist wrist) {
        // involved
        this.wrist = wrist;

        // assumes all the way closed
        baseClicks = getClicks();
    }

    // === Executing per Period ===

    public double getClicks() {
        return grabberEncoder.get();
    }

    // public void close() {
    //     if (state != States.Closing) {
    //         state = States.Closing;
    //         grabberPower = -.5d;
    //     }
    // }

    // public void open() {
    //     if (state != States.Opening) {
    //         state = States.Opening;
    //         grabberPower = .5d;
    //     }
    // }

    // public void grip() {
    //     if (state != States.Gripping) {
    //         state = States.Gripping;
    //         grabberPower = .1d;
    //     }
    // }

    // close by time so that we can reset baseline for opening
    public void close() {
        if (state != States.Closing) {
            state = States.Closing;
            grabberPower = -.5d;
            stateExpiration = Timer.getFPGATimestamp() + 1d;
            wrist.nudge(-.25);
        }
    }

    // open by target clicks
    public void openHatch() {
        if (state != States.OpeningHatch) {
            state = States.OpeningHatch;
            targetClicks = baseClicks + (RobotMap.grabberEncoderClicksPerDegree * RobotMap.grabberHatchOpen);
            wrist.nudge(.25);
        }
    }

    // open by target clicks
    public void openCargo() {
        if (state != States.OpeningCargo) {
            state = States.OpeningCargo;
            targetClicks = baseClicks + (RobotMap.grabberEncoderClicksPerDegree * RobotMap.grabberCargoOpen);
            wrist.nudge(.25);
        }
    }

    // stay at current position 
    public void grip() {
        if (state != States.Gripping) {
            state = States.Gripping;
            targetClicks = grabberEncoder.get();
        }
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
            if (state == States.Closing) {
                // close using time. reset baseline after closing and move to gripping
                if (Timer.getFPGATimestamp() > stateExpiration) {
                    baseClicks = grabberEncoder.get();
                    grip();
                }
            }
            else if (state == States.OpeningHatch || state == States.OpeningCargo) {
                grabberPower = grabberPid.update(targetClicks - getClicks(), RobotMap.grabberLocality);
                grabberPower = Functions.clip(grabberPower, -.5d, .5d);
            }

            grabberSpeedController.set(grabberPower);
            rollerFrontSpeedController.set(rollerPower);
            rollerBackSpeedController.set(rollerPower);
        }
        putTelemetry();
    }

    public void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Grabber Power", grabberPower);
        telemetry.putDouble("Encoder Clicks", grabberEncoder.get());
        telemetry.putDouble("Target Clicks", targetClicks);
        telemetry.putDouble("Base Clicks", baseClicks);
        telemetry.putDouble("Roller Power (intake|expell)", rollerPower);
        telemetry.putString("Version", "1.0.0");
    }
}