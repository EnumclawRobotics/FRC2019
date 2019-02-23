package frc.robot.Components;

import common.instrumentation.Telemetry;
import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;

public class Lifter {
    private Telemetry telemetry = new Telemetry("Robot/Lifter");

    private SpeedController frontSpeedController;
    private SpeedController backSpeedController;
    private SpeedController moverSpeedController;

    private boolean liftingActive;
    private boolean frontActive;
    private boolean backActive;

    private double frontSpeed;
    private double backSpeed;
    private double moverSpeed;

    private double feedForward;

    public Lifter(RobotMap robotMap) {
        frontSpeedController = robotMap.liftFrontSpeedController;
        backSpeedController = robotMap.liftBackSpeedController;
        moverSpeedController = robotMap.liftMoverSpeedController;

        ((MotorSafety)frontSpeedController).setExpiration(RobotMap.safetyExpiration);
        ((MotorSafety)frontSpeedController).setSafetyEnabled(true);

        ((MotorSafety)backSpeedController).setExpiration(RobotMap.safetyExpiration);
        ((MotorSafety)backSpeedController).setSafetyEnabled(true);

        feedForward = robotMap.liftFeedForward;
        moverSpeed = robotMap.liftMoverSpeed;
    }

    public boolean getLiftingActive() {
        return liftingActive;
    }

    public void setLiftingActive() {
        liftingActive = true;
        frontActive = true;
        backActive = true;
    }

    public void setFrontActive() {
        liftingActive = true;
        frontActive = true;
        backActive = false;
    }

    public void setBackActive() {
        liftingActive = true;
        frontActive = false;
        backActive = true;
    }

    public void move(double speed) {
        frontSpeed = (frontActive ? speed : 0);
        backSpeed = (backActive ? speed : 0);
    }

    public void run() {
        if (liftingActive) {
            frontSpeedController.set(Geometry.clip(frontSpeed + feedForward , -1, 1));        
            backSpeedController.set(Geometry.clip(backSpeed + feedForward , -1, 1));
            moverSpeedController.set(moverSpeed);
        }
        else {
            stop();
        }

        putTelemetry();
    }

    public void stop() {
        liftingActive = false;
        frontActive = false;
        backActive = false;

        frontSpeedController.stopMotor();
        backSpeedController.stopMotor();
        moverSpeedController.stopMotor();
    }

    private void putTelemetry() {
        telemetry.putDouble("Front Power", frontSpeed);
        telemetry.putDouble("Back Power", backSpeed);
        telemetry.putDouble("Move Power", moverSpeed);
        telemetry.putString("Version", "1.0.0");
    }
}