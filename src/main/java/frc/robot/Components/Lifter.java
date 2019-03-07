package frc.robot.Components;

import com.revrobotics.CANSparkMax.IdleMode;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;
import common.util.PID;

public class Lifter {
    private Telemetry telemetry = new Telemetry("Robot/Lifter");

    private RampSpeedController frontSpeedController;
    private RampSpeedController backSpeedController;
    private SpeedController moverSpeedController;

    private CANEncoder2 frontEncoder;
    private CANEncoder2 backEncoder;

    private double frontBaseClicks;
    private double backBaseClicks;
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

        frontSpeedController = new RampSpeedController(robotMap.liftFrontSpeedController, RobotMap.liftRamp) ;
        backSpeedController = new RampSpeedController(robotMap.liftBackSpeedController, RobotMap.liftRamp);
        moverSpeedController = robotMap.liftMoverSpeedController;

        frontEncoder = robotMap.liftFrontEncoder;
        backEncoder = robotMap.liftBackEncoder;

        stow();
    }

    public enum States {
        Stopped,
        Stowing, 
        TouchingDown,
        Extending,
        RetractingFront,
        RetractingBack,
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
        state = States.Stowing;
        frontPower = RobotMap.liftStow;     // gently continuously retract
        backPower = RobotMap.liftStow;
        moverPower = 0;

        stateExpiration = Timer.getFPGATimestamp() + .5d;
    }

    // gently touch down without lifting
    public void touchdown() {
        state = States.TouchingDown;
        frontPower = RobotMap.liftTouchdown;     // gently extend without lifting
        backPower = RobotMap.liftTouchdown;
        
        stateExpiration = Timer.getFPGATimestamp() + .5d;
    }

    // extend the lift elements
    public void extend() {
        state = States.Extending;
        frontTargetClicks = frontTargetClicks + 10;
        backTargetClicks = backTargetClicks + 10;
        moverPower = RobotMap.liftMoverPower;
    }

    // retracting the front side
    public void retractFront() {
        if (state == States.Extending) {
            state = States.RetractingFront;
            frontPower = RobotMap.liftStow;         // gently and continuously retract
            backPower = 0;                          // brake
            moverPower = RobotMap.liftMoverPower;

            stateExpiration = Timer.getFPGATimestamp() + 1.5d;
        }
    }

    // retracting the black side
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
        if (state == States.Stowing) {
            if (Timer.getFPGATimestamp() > stateExpiration) {
                brake();
            }
        }
        if (state == States.TouchingDown) {
            if (Timer.getFPGATimestamp() > stateExpiration) {
                frontBaseClicks = frontEncoder.get();
                backBaseClicks = backEncoder.get();
                frontTargetClicks = frontBaseClicks;
                backTargetClicks = backBaseClicks;
                extend();
            }
        }
        if (state == States.Extending) {
            double correction = ((backEncoder.get() - backTargetClicks) - (frontEncoder.get() - frontTargetClicks)) * RobotMap.liftKpFactor;
            frontPower = RobotMap.liftExtend + correction;
            backPower = RobotMap.liftExtend - correction;  
        }

        frontSpeedController.set(frontPower);        
        backSpeedController.set(backPower);
        moverSpeedController.set(moverPower);

        putTelemetry();
    }

    public void stop() {
        state = States.Stopped;
        frontSpeedController.stopMotor();
        backSpeedController.stopMotor();
        moverSpeedController.stopMotor();
    }

    private void putTelemetry() {
        telemetry.putDouble("Front Power", frontPower);
        telemetry.putDouble("Back Power", backPower);
        telemetry.putDouble("Move Power", moverPower);
        telemetry.putString("Version", "1.0.0");
    }
}