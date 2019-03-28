package frc.robot.Components;

import com.revrobotics.CANSparkMax.IdleMode;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;
import common.util.PID;
import common.util.Functions;

public class Lifter {
    private Telemetry telemetry = new Telemetry("Robot/Lifter");

    private boolean rolling = false;

    public enum States {
        Stopped,
        Stowing, 
        StowingFront,
        StowingBack,
        Moving,
        Holding
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

    private double frontBaseClicks;
    private double backBaseClicks;

    private double frontTargetClicks;
    private double backTargetClicks;

    private double frontError;
    private double backError;

    private double balancePower;
    private double frontPower;
    private double backPower;

    private double moverPower;
    private double stateExpiration;

    private States state = States.Stopped;
    private ClimbingStates climbingState = ClimbingStates.Inactive;

    private double climbingStateStartTime;

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
        frontBaseClicks = getFrontClicks();
        backBaseClicks = getBackClicks();
        frontTargetClicks = frontBaseClicks;
        backTargetClicks = backBaseClicks;
        stow();
    }

    public States getState() {
        return state;
    }

    public double getFrontClicks() {
        return frontEncoder.getPosition();
    }

    public double getBackClicks() {
        return backEncoder.getPosition();
    }

    //NEW!
    public ClimbingStates getClimbingState() {
        return climbingState;
    }
    public double getFrontError() {
        frontError = frontTargetClicks - getFrontClicks();
        return frontError;
    }

    public double getBackError() {
        backError = backTargetClicks - getBackClicks();
        return backError;
    }

    // hold at position
    private void brake() {
        frontPower = 0;
        backPower = 0;
        frontTargetClicks = getFrontClicks();
        backTargetClicks = getBackClicks();
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
        frontPower = RobotMap.liftStow;                 // gently retract front
        state = States.StowingFront;
    }

    // stores back lift
    public void stowBack() {
        backPower = RobotMap.liftStow;                  // gently retract back
        state = States.StowingBack;
    }
    
    // raising / lowering
    public void moveAll(double controlPower) {
        moveFront(controlPower);
        moveBack(controlPower);
        state = States.Moving;
    }

    // raising / lowering
    public void moveFront(double controlPower) {
        frontTargetClicks = frontTargetClicks + (controlPower * RobotMap.liftLocality);
    }

    // raising / lowering
    public void moveBack(double controlPower) {
        backTargetClicks = backTargetClicks + (controlPower * RobotMap.liftLocality);
    }

    // raising / lowering
    public void moveAllClicks(double clicks) {
        moveFrontClicks(clicks);
        moveBackClicks(clicks);
        state = States.Moving;
    }

    // raising / lowering
    public void moveFrontClicks(double clicks) {
        frontTargetClicks = frontBaseClicks + clicks;
    }

    // raising / lowering
    public void moveBackClicks(double clicks) {
        backTargetClicks = backBaseClicks + clicks;
    }

    // holding position
    public void holding() {
        if (state != States.Holding) {
            frontTargetClicks = getFrontClicks();
            backTargetClicks = getBackClicks();
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
                backPower = backPid.update(getBackError(), RobotMap.liftLocality, RobotMap.liftPower);
            }
            else if (state == States.Moving || state == States.Holding) {
                frontError = getFrontError();
                backError = getBackError();

                // try and keep the front and back in synch
                if (frontError > backError) {
                    frontPower = frontPid.update(frontError, RobotMap.liftLocality, RobotMap.liftPower) + .05d;
                    backPower = backPid.update(backError, RobotMap.liftLocality, RobotMap.liftPower);
                } else if (backError > frontError) {
                    frontPower = frontPid.update(frontError, RobotMap.liftLocality, RobotMap.liftPower);
                    backPower = backPid.update(backError, RobotMap.liftLocality, RobotMap.liftPower) + .05d;
                } else {
                    frontPower = frontPid.update(frontError, RobotMap.liftLocality, RobotMap.liftPower);
                    backPower = backPid.update(backError, RobotMap.liftLocality, RobotMap.liftPower);
                }

                balancePower = (((frontError - backError)/2d) / RobotMap.liftLocality) * RobotMap.liftPidKp;
                
                frontPower = frontPid.update(frontError, RobotMap.liftLocality, RobotMap.liftPower) - balancePower;
                backPower = backPid.update(backError, RobotMap.liftLocality, RobotMap.liftPower) + balancePower;
            }
            moverPower = (rolling && state == States.Holding) ? RobotMap.liftMoverPower : 0.0f; //NEW!

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
                int clicksToHab = RobotMap.liftEncoderClicksPerInch * (RobotMap.liftStartingError + FieldMap.heightHabLevel2);
                frontTargetClicks = clicksToHab;
                backTargetClicks = clicksToHab;
                state = State.Moving;

                if (getFrontClicks() >= clicksToHab) {
                    climbingState = ClimbingStates.UseLiftWheels;
                    climbingStateStartTime = Timer.getFPGATimestamp();
                }
            break;

            case UseLiftWheels:
                state = State.Holding;
                rolling = true;

                if (Timer.getFPGATimestamp() >= climbingStateStartTime + 2.0f) {
                    rolling = false;
                    climbingState = ClimbingStates.RaiseFrontLift;
                }
            break;
            case RaiseFrontLift:
                stowFront();
                if (getFrontClicks() <= 0) { //If front hab lift has fully retracted
                    climbingState = ClimbingStates.DriveWheels1;
                    climbingStateStartTime = Timer.getFPGATimestamp();
                }
            break;
            case DriveWheels1:
                drive.move(0.05f, 0.0f, false);
                if (Timer.getFPGATimestamp() >= climbingStateStartTime + 2.0f) {
                    climbingState = ClimbingStates.RaiseBackLift;
                }
            break;
            case RaiseBackLift:
                stowBack();
                if (getBackClicks() <= 0) { //If back hab lift has fully retracted
                    climbingState = ClimbingStates.DriveWheels2;
                    climbingStateStartTime = Timer.getFPGATimestamp();
                }
            break;
            case DriveWheels2:
                drive.move(0.05f, 0.0f, false);
                if (Timer.getFPGATimestamp() >= climbingStateStartTime + 2.0f) {
                    climbingState = ClimbingStates.Inactive;
                }
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
        telemetry.putDouble("Balance Power", moverPower);
        telemetry.putDouble("Move Power", moverPower);
        telemetry.putString("Version", "1.0.0");
    }
}