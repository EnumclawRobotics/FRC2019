package frc.robot.Components;

import com.revrobotics.CANSparkMax.IdleMode;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;
import common.util.PID;

public class Lifter {
    private Telemetry telemetry = new Telemetry("Robot/Lifter");

    private SpeedController frontSpeedController;
    private SpeedController backSpeedController;
    private SpeedController moverSpeedController;

    private CANEncoder2 frontEncoder;
    private CANEncoder2 backEncoder;

    private PID frontPid;
    private PID backPid;

    private double frontTouchdownClicks;
    private double backTouchdownClicks;
    private double frontTargetClicks;
    private double backTargetClicks;

    private double frontPower;
    private double backPower;
    private double moverPower;
    private double stateExpiration;

    private States state = States.Stopped;

    public Lifter(RobotMap robotMap) {
        robotMap.liftFrontSpeedController.setIdleMode(IdleMode.kBrake);
        robotMap.liftBackSpeedController.setIdleMode(IdleMode.kBrake);

//        robotMap.liftFrontSpeedController.setInverted(true);
//        robotMap.liftBackSpeedController.setInverted(true);

        frontSpeedController = robotMap.liftFrontSpeedController;
        backSpeedController = robotMap.liftBackSpeedController;
        moverSpeedController = robotMap.liftMoverSpeedController;

        frontEncoder = robotMap.liftFrontEncoder;
        backEncoder = robotMap.liftBackEncoder;

        frontPid = new PID();
        frontPid.setGainsPID(RobotMap.liftPidKp, RobotMap.liftPidKi, RobotMap.liftPidKd);
        backPid = new PID();
        backPid.setGainsPID(RobotMap.liftPidKp, RobotMap.liftPidKi, RobotMap.liftPidKd);
    }

    public void init() {
        stow();
    }

    public enum States {
        Stopped,
        Stowing, 
        TouchingDown,
        Extending,
        RetractingFront,
        RetractingBack,
        Retracting
    }

    public States getState() {
        return state;
    }

    private void brake() {
        frontPower = 0;
        backPower = 0;
    }

    // goes into stowed mode where the lift retracts with a holding force enough to hold the lift columns up
    public void stow() {
        // was stopped so store lift legs
        if (state == States.Stopped) {
            frontPower = RobotMap.liftStow;     // gently continuously retract
            backPower = RobotMap.liftStow;
            moverPower = 0;
            stateExpiration = Timer.getFPGATimestamp() + .5d;
        } 
        // was extending so actually brake 
        else if (state == States.Extending || state == States.RetractingBack || state == States.RetractingFront) {
            brake();
            stateExpiration = Timer.getFPGATimestamp() + 1d;
        } 
        // 
        else if (state == States.Stowing) {
            if (Timer.getFPGATimestamp() > stateExpiration) {
                brake();
            }
        }
        state = States.Stowing;
    }

    // gently touch down without lifting
    public void touchdown() {
        if (state == States.Stowing) {
            state = States.TouchingDown;
            frontPower = RobotMap.liftTouchdown;     // gently extend without lifting
            backPower = RobotMap.liftTouchdown;
            stateExpiration = Timer.getFPGATimestamp() + 1d;
        }
    }

    // extend the lift elements
    public void extend() {
        if (state == States.TouchingDown) {
            frontTouchdownClicks = frontEncoder.get();
            backTouchdownClicks = backEncoder.get();
            frontTargetClicks = frontTouchdownClicks;
            backTargetClicks = frontTouchdownClicks;
            state = States.Extending;
        } 
        else if (state == States.Extending || state == States.Retracting) {
            // are we close together on front and back? if not dont move else pick another goal 
            if (Math.abs((frontEncoder.get() - frontTouchdownClicks) - (backEncoder.get() - backTouchdownClicks)) < RobotMap.liftLocality) {
                frontTargetClicks += RobotMap.liftLocality;
                backTargetClicks += RobotMap.liftLocality;
                moverPower = RobotMap.liftMoverPower;
            }
        }
    }

    // retracting the front side
    public void retract() {
        if (state == States.Extending || state == States.Retracting) {
            // are we close together on front and back? if not dont move else pick another goal 
            if (Math.abs((frontEncoder.get() - frontTouchdownClicks) - (backEncoder.get() - backTouchdownClicks)) < RobotMap.liftLocality) {
                frontTargetClicks -= RobotMap.liftLocality;
                backTargetClicks -= RobotMap.liftLocality;
                moverPower = 0;
            }
        }
    }

    // retracting the front side
    public void retractFront() {
        if (state == States.Extending || state == States.Retracting) {
            state = States.RetractingFront;
            frontPower = RobotMap.liftStow;         // gently and continuously retract
            moverPower = RobotMap.liftMoverPower;
            stateExpiration = Timer.getFPGATimestamp() + 1.5d;
        }
    }

    // retracting the back side
    public void retractBack() {
        if (state == States.RetractingFront) {
            state = States.RetractingBack;
            frontPower = 0;
            backPower = RobotMap.liftStow;
            moverPower = 0;
            stateExpiration = Timer.getFPGATimestamp() + 1.5d;
        }
    }

    public void run() {
        if (state != States.Stopped) {
            // adjustments per cycle
            if (state == States.Stowing) {
                if (Timer.getFPGATimestamp() > stateExpiration) {
                    brake();
                }
            }
            else if (state == States.TouchingDown) {
                if (Timer.getFPGATimestamp() > stateExpiration) {
                    brake();
                    frontTouchdownClicks = frontEncoder.get();
                    backTouchdownClicks = backEncoder.get();
                    extend();
                }
            }
            else if (state == States.Extending || state == States.Retracting) {
                // lift/retract
                frontPower = RobotMap.liftPower + frontPid.update(frontTargetClicks - frontEncoder.get(), RobotMap.liftLocality);
                backPower = RobotMap.liftPower + backPid.update(backTargetClicks - backEncoder.get(), RobotMap.liftLocality);

                // try and keep the lift relatively in parrallel
                double liftCorrection = ((frontEncoder.get() - frontTargetClicks) - (backEncoder.get() - backTargetClicks)) * RobotMap.liftPidKp;
                frontPower = frontPower - liftCorrection;
                backPower = backPower + liftCorrection;  
            }
            else if (state == States.RetractingFront) {
                backPower = RobotMap.liftPower + backPid.update(backTargetClicks - backEncoder.get(), RobotMap.liftLocality);
            }
            
            frontSpeedController.set(frontPower);        
            backSpeedController.set(backPower);
            moverSpeedController.set(moverPower);
        }

        putTelemetry();
    }

    public void stop() {
        state = States.Stopped;
        frontSpeedController.stopMotor();
        backSpeedController.stopMotor();
        moverSpeedController.stopMotor();
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Front Touchdown Clicks", frontTouchdownClicks);
        telemetry.putDouble("Front Clicks", frontEncoder.get());
        telemetry.putDouble("Front Target Clicks", frontTargetClicks);
        telemetry.putDouble("Front Power", frontPower);
        telemetry.putDouble("Back Touchdown Clicks", backTouchdownClicks);
        telemetry.putDouble("Back Clicks", backEncoder.get());
        telemetry.putDouble("Back Target Clicks", backTargetClicks);
        telemetry.putDouble("Back Power", backPower);
        telemetry.putDouble("Move Power", moverPower);
        telemetry.putString("Version", "1.0.0");
    }
}