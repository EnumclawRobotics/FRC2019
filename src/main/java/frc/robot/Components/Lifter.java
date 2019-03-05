package frc.robot.Components;

import com.revrobotics.CANSparkMax.IdleMode;

import common.instrumentation.Telemetry;
import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;

public class Lifter {
    private Telemetry telemetry = new Telemetry("Robot/Lifter");

    private SpeedController maroonSpeedController;
    private SpeedController blackSpeedController;
    private SpeedController moverSpeedController;

    private double stowedPower;
    private double maroonPower;
    private double blackPower;
    private double moverPower;
    private double stateExpiration;

    private States state = States.Stopped;

    public Lifter(RobotMap robotMap) {
        robotMap.liftMaroonSpeedController.setIdleMode(IdleMode.kBrake);
        robotMap.liftBlackSpeedController.setIdleMode(IdleMode.kBrake);
        // TODO: Doublecheck that motor is inverted?
        robotMap.liftBlackSpeedController.setInverted(true);

        maroonSpeedController = robotMap.liftMaroonSpeedController;
        blackSpeedController = robotMap.liftBlackSpeedController;
        moverSpeedController = robotMap.liftMoverSpeedController;

        setStowed();
    }

    public enum States {
        Stowed, 
        Touchdown,
        Extending,
        RetractingMaroon,
        RetractingBlack,
        Stopped
    }

    public States getState() {
        return state;
    }

    // goes into stowed mode where the lift retracts with a holding force enough to hold the lift columns up
    public void setStowed() {
        state = States.Stowed;
        maroonPower = RobotMap.liftStowedPower;     // gently continuously retract
        blackPower = RobotMap.liftStowedPower;
    }

    // starts a lift the whole bot cycle
    public void lift() {
        if (state == States.Stowed) {
            touchdown();
        }
    }

    // gently touch down without lifting
    private void touchdown() {
        state = States.Touchdown;
        maroonPower = -RobotMap.liftStowedPower;     // gently extend without lifting
        blackPower = -RobotMap.liftStowedPower;
        stateExpiration = Timer.getFPGATimestamp() + .5d;
    }

    // extend the lift elements
    private void extend() {
        state = States.Extending;
        maroonPower = RobotMap.liftExtend;           // forcefully continuously extend at just over what is required
        blackPower = RobotMap.liftExtend;
        moverPower = RobotMap.liftMoverPower;
    }

    // retracting the maroon side
    public void retractMaroon() {
        if (state == States.Extending) {
            state = States.RetractingMaroon;
            maroonPower = RobotMap.liftStowedPower;     // gently and continuously retract
        }
    }

    // retracting the black side
    // TODO: Make sure black side is where roller is installed
    public void retractBlack() {
        if (state == States.RetractingMaroon) {
            state = States.RetractingBlack;
            blackPower = RobotMap.liftStowedPower;     // gently and continuously retract
            moverPower = 0;
        }
    }

    public void run() {
        if (state == States.Touchdown) {
            if (Timer.getFPGATimestamp() > stateExpiration) {
                extend();
            }
        }

        maroonSpeedController.set(maroonPower);        
        blackSpeedController.set(blackPower);
        moverSpeedController.set(moverPower);

        putTelemetry();
    }

    public void stop() {
        state = States.Stopped;
        maroonSpeedController.stopMotor();
        blackSpeedController.stopMotor();
        moverSpeedController.stopMotor();
    }

    private void putTelemetry() {
        telemetry.putDouble("Maroon Power", maroonPower);
        telemetry.putDouble("Gold Power", blackPower);
        telemetry.putDouble("Move Power", moverPower);
        telemetry.putString("Version", "1.0.0");
    }
}