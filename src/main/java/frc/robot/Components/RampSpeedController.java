package frc.robot.Components;

import edu.wpi.first.wpilibj.SpeedController;
import common.util.Geometry;

public class RampSpeedController implements SpeedController {
    private SpeedController speedController;
    private double rampIncrement; 

    public RampSpeedController(SpeedController speedController, double rampIncrement) {
        this.speedController = speedController;
        this.rampIncrement = rampIncrement;
    }

    public void set(double targetPower) {
        double currentPower = speedController.get();
        double power = 0;

        // close? then just snap to it, else progress in correct diraction
        if (Math.abs(targetPower - currentPower) < rampIncrement) {
            power = targetPower;
        } else if (targetPower > currentPower) {
            power = currentPower + rampIncrement;
        } else if (targetPower < currentPower) {
            power = currentPower - rampIncrement;
        }

        Geometry.clip(power, -1d, 1d);

        speedController.set(power);
    }

    public double get() {
        return speedController.get();
    }

    public void pidWrite(double targetPower) {
        speedController.set(targetPower);
    }

    public void stopMotor() {
        speedController.stopMotor();
    }

    public void disable() {
        speedController.disable();
    }

    public void setInverted(boolean isInverted) {
        speedController.setInverted(isInverted);
    }

    public boolean getInverted() {
        return speedController.getInverted();
    }

}