package frc.robot.Components;

import common.instrumentation.Telemetry;
import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;

public class Lifter {
    private Telemetry telemetry = new Telemetry("Robot/Lifter");

    private SpeedController maroonSpeedController;
    private SpeedController blackSpeedController;
    private SpeedController moverSpeedController;

    private boolean liftingActive;
    private boolean maroonActive;
    private boolean blackActive;

    private double maroonPower;
    private double blackPower;
    private double moverPower;

    private double gravity;

    public Lifter(RobotMap robotMap) {
        maroonSpeedController = robotMap.liftMaroonSpeedController;
        blackSpeedController = robotMap.liftBlackSpeedController;
        moverSpeedController = robotMap.liftMoverSpeedController;

        // ((MotorSafety)maroonSpeedController).setExpiration(RobotMap.safetyExpiration);
        // ((MotorSafety)maroonSpeedController).setSafetyEnabled(true);

        // ((MotorSafety)blackSpeedController).setExpiration(RobotMap.safetyExpiration);
        // ((MotorSafety)blackSpeedController).setSafetyEnabled(true);

        gravity = robotMap.liftGravity;
        moverPower = robotMap.liftMoverSpeed;
    }

    public boolean getLiftingActive() {
        return liftingActive;
    }

    public void setLiftingActive() {
        liftingActive = true;
        maroonActive = true;
        blackActive = true;
    }

    public void setMaroonActive() {
        liftingActive = true;
        maroonActive = true;
        blackActive = false;
    }

    public void setGoldActive() {
        liftingActive = true;
        maroonActive = false;
        blackActive = true;
    }

    public void move(double speed) {
        maroonPower = (maroonActive ? speed : 0);
        blackPower = (blackActive ? speed : 0);
    }

    public void run() {
        if (liftingActive) {
            maroonSpeedController.set(Geometry.clip(maroonPower + gravity , -1, 1));        
            blackSpeedController.set(Geometry.clip(blackPower + gravity , -1, 1));
            moverSpeedController.set(moverPower);
        }
        else {
            stop();
        }

        putTelemetry();
    }

    public void stop() {
        liftingActive = false;
        maroonActive = false;
        blackActive = false;

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