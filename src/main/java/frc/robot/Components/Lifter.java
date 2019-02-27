package frc.robot.Components;

import common.instrumentation.Telemetry;
import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;

public class Lifter {
    private Telemetry telemetry = new Telemetry("Robot/Lifter");

    private SpeedController maroonSpeedController;
    private SpeedController goldSpeedController;
    private SpeedController moverSpeedController;

    private boolean liftingActive;
    private boolean maroonActive;
    private boolean goldActive;

    private double maroonSpeed;
    private double goldSpeed;
    private double moverSpeed;

    private double gravity;

    public Lifter(RobotMap robotMap) {
        maroonSpeedController = robotMap.liftMaroonSpeedController;
        goldSpeedController = robotMap.liftGoldSpeedController;
        moverSpeedController = robotMap.liftMoverSpeedController;

        ((MotorSafety)maroonSpeedController).setExpiration(RobotMap.safetyExpiration);
        ((MotorSafety)maroonSpeedController).setSafetyEnabled(true);

        ((MotorSafety)goldSpeedController).setExpiration(RobotMap.safetyExpiration);
        ((MotorSafety)goldSpeedController).setSafetyEnabled(true);

        gravity = robotMap.liftGravity;
        moverSpeed = robotMap.liftMoverSpeed;
    }

    public boolean getLiftingActive() {
        return liftingActive;
    }

    public void setLiftingActive() {
        liftingActive = true;
        maroonActive = true;
        goldActive = true;
    }

    public void setMaroonActive() {
        liftingActive = true;
        maroonActive = true;
        goldActive = false;
    }

    public void setGoldActive() {
        liftingActive = true;
        maroonActive = false;
        goldActive = true;
    }

    public void move(double speed) {
        maroonSpeed = (maroonActive ? speed : 0);
        goldSpeed = (goldActive ? speed : 0);
    }

    public void run() {
        if (liftingActive) {
            maroonSpeedController.set(Geometry.clip(maroonSpeed + gravity , -1, 1));        
            goldSpeedController.set(Geometry.clip(goldSpeed + gravity , -1, 1));
            moverSpeedController.set(moverSpeed);
        }
        else {
            stop();
        }

        putTelemetry();
    }

    public void stop() {
        liftingActive = false;
        maroonActive = false;
        goldActive = false;

        maroonSpeedController.stopMotor();
        goldSpeedController.stopMotor();
        moverSpeedController.stopMotor();
    }

    private void putTelemetry() {
        telemetry.putDouble("Maroon Power", maroonSpeed);
        telemetry.putDouble("Gold Power", goldSpeed);
        telemetry.putDouble("Move Power", moverSpeed);
        telemetry.putString("Version", "1.0.0");
    }
}