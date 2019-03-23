package frc.robot.Components;

import com.revrobotics.CANSparkMax.IdleMode;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;
import common.util.PID;

public class Lifter {
    private Telemetry telemetry = new Telemetry("Robot/Lifter");

    public enum States {
        Stopped,
        Stowing, 
        StowingFront,
        StowingBack,
        Moving,
        Holding,
        Rolling //NEW!
    }

    //NEW!
    public enum ClimbingStates {
        Inactive,
        Teleop,
        Raising,
        UseLiftWheels,
        RaiseFrontLift,
        DriveWheels1,
        RaiseBackLift,
        DriveWheels2
    }

    private RampSpeedController frontSpeedController;
    private RampSpeedController backSpeedController;
    private RampSpeedController moverSpeedController;

    private CANEncoder2 frontEncoder;
    private CANEncoder2 backEncoder;

    private PID frontPid;
    private PID backPid;

    private double frontTargetClicks;
    private double backTargetClicks;

    private double frontPower;
    private double backPower;
    private double moverPower;
    private double stateExpiration;

    private States state = States.Stopped;
    private ClimbingStates climbingState; //NEW!

    public Lifter(RobotMap robotMap) {
        robotMap.liftFrontSpeedController.setIdleMode(IdleMode.kBrake);
        robotMap.liftBackSpeedController.setIdleMode(IdleMode.kBrake);

//        robotMap.liftFrontSpeedController.setInverted(true);
//        robotMap.liftBackSpeedController.setInverted(true);

        frontSpeedController = new RampSpeedController(robotMap.liftFrontSpeedController, RobotMap.liftRamp);
        backSpeedController = new RampSpeedController(robotMap.liftBackSpeedController, RobotMap.liftRamp);
        moverSpeedController = new RampSpeedController(robotMap.liftMoverSpeedController, RobotMap.liftRamp);

        frontEncoder = robotMap.liftFrontEncoder;
        backEncoder = robotMap.liftBackEncoder;

        frontPid = new PID();
        frontPid.setGainsPID(RobotMap.liftPidKp, RobotMap.liftPidKi, RobotMap.liftPidKd);
        backPid = new PID();
        backPid.setGainsPID(RobotMap.liftPidKp, RobotMap.liftPidKi, RobotMap.liftPidKd);
    }

    public void init() {
        frontTargetClicks = frontEncoder.get();
        backTargetClicks = backEncoder.get();
        stow();
    }

    public States getState() {
        return state;
    }

    //NEW!
    public ClimbingStates getClimbingState() {
        return climbingstate;
    }

    // hold at position
    private void brake() {
        frontPower = 0;
        backPower = 0;
        frontTargetClicks = frontEncoder.get();
        backTargetClicks = backEncoder.get();
    }

    // stores both lifts
    public void stow() {
        frontPower = RobotMap.liftStow;     // gently retract for a period
        backPower = RobotMap.liftStow;
        moverPower = 0;
        stateExpiration = Timer.getFPGATimestamp() + 1d;
        state = States.Stowing;
    }

    // stores front lift 
    public void stowFront() {
        frontPower = RobotMap.liftStow;                 // gently retract
        frontTargetClicks = frontEncoder.get();
        state = States.StowingFront;
    }

    // stores back lift
    public void stowBack() {
        backPower = RobotMap.liftStow;     // gently retract
        state = States.StowingBack;
    }
    
    // raising / lowering
    public void move(double controlPower) {
        frontTargetClicks = frontTargetClicks + (controlPower * RobotMap.liftLocality);
        backTargetClicks = backTargetClicks + (controlPower * RobotMap.liftLocality);
        state = States.Moving;
    }

    // holding position
    public void holding() {
        if (state != States.Holding) {
            frontTargetClicks = frontEncoder.get();
            backTargetClicks = backEncoder.get();
            state = States.Holding;
        }
    }

    public void run() {
        // adjustments per cycle
            if (state != States.Stopped) {
                if (state == States.Stowing) {
                    if (Timer.getFPGATimestamp() > stateExpiration) {
                        brake();
                    }
                }
                else if (state == States.StowingFront) {
                    backPower = backPid.update(backTargetClicks - backEncoder.get(), RobotMap.liftLocality);
                }
                else if (state == States.Moving || state == States.Holding) {
                    frontPower = frontPid.update(frontTargetClicks - frontEncoder.get(), RobotMap.liftLocality);
                    backPower = backPid.update(backTargetClicks - backEncoder.get(), RobotMap.liftLocality);
                }
                
                moverPower = (state == States.Rolling) ? 1.0f : 0.0f; //NEW!

                frontSpeedController.set(frontPower);        
                backSpeedController.set(backPower);
                moverSpeedController.set(moverPower);
            }

        switch (climbingState)
        {
            case Inactive:
            //Nothing
            break;
            case Teleop:
            //Nothing
            break;

            case Raising:
            //move(/*power*/);
            if (frontEncoder.get >= 000000000000) {
                climbingState = ClimbingStates.UseLiftWheels;
                Timer rollingStateDuration = new Timer();
            }
            break;

            case UseLiftWheels:
            state = States.Rolling;
            if (rollingStateDuration.getFPGATimestamp() >= 2.0)
            {
                climbingState = ClimbingStates.RaiseFrontLift;
            }

            break;
            case RaiseFrontLift:
            //...
            break;
            case DriveWheels1:
            //...
            break;
            case RaiseBackLift:
            //...
            break;
            case DriveWheels2:
            //...
            break;
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
        telemetry.putDouble("Front Clicks", frontEncoder.get());
        telemetry.putDouble("Front Target Clicks", frontTargetClicks);
        telemetry.putDouble("Front Power", frontPower);
        telemetry.putDouble("Back Clicks", backEncoder.get());
        telemetry.putDouble("Back Target Clicks", backTargetClicks);
        telemetry.putDouble("Back Power", backPower);
        telemetry.putDouble("Move Power", moverPower);
        telemetry.putString("Version", "1.0.0");
    }
}