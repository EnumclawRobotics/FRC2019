package frc.robot.Components;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;

public class Lifter {
    private Telemetry telemetry = new Telemetry("Robot/Lifter");

    private SpeedController liftFrontSpeedController;
    private SpeedController liftBackSpeedController;
    private SpeedController liftMoverSpeedController;

//    private Encoder liftFrontEncoder;
//    private Encoder liftBackEncoder;

    private double liftFrontSpeed;
    private double liftBackSpeed;
    private double liftMoverSpeed;

    public Lifter(RobotMap robotMap) {
        liftFrontSpeedController = robotMap.liftFrontSpeedController;
        liftBackSpeedController = robotMap.liftBackSpeedController;
        liftMoverSpeedController = robotMap.liftMoverSpeedController;

        ((MotorSafety)liftFrontSpeedController).setExpiration(RobotMap.safetyExpiration);
        ((MotorSafety)liftFrontSpeedController).setSafetyEnabled(true);

        ((MotorSafety)liftBackSpeedController).setExpiration(RobotMap.safetyExpiration);
        ((MotorSafety)liftBackSpeedController).setSafetyEnabled(true);
    }

    // public double getBackClicks() {
    //     return liftBackEncoder.get();
    // }

    // public double getFrontClicks() {
    //     return liftFrontEncoder.get();
    // }

    public void moveFrontLift(double speed) {
        liftFrontSpeed = speed;
    }

    public void moveBackLift(double speed) {
        liftBackSpeed = speed;
    }

    public void move(double speed) {
        liftMoverSpeed = speed;
    }

    public void run() {
        liftFrontSpeedController.set(liftFrontSpeed);        
        liftBackSpeedController.set(liftBackSpeed);
        liftMoverSpeedController.set(liftMoverSpeed);
    }

    public void stop() {
        liftFrontSpeedController.stopMotor();
        liftBackSpeedController.stopMotor();
        liftMoverSpeedController.stopMotor();
    }

    private void putTelemetry() {
        telemetry.putDouble("Front Power", liftFrontSpeed);
//        telemetry.putDouble("Front Clicks", getFrontClicks());
        telemetry.putDouble("Back Power", liftBackSpeed);
//        telemetry.putDouble("Back Clicks", getBackClicks());
        telemetry.putString("Version", "1.0.0");
    }

}