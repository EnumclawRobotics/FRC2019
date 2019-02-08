package frc.robot.Components;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.*;

public class GenericEncoder implements CounterBase {
    Encoder encoder = null;
    CANEncoder canEncoder = null;

    public GenericEncoder(Encoder encoder) {
        this.encoder = encoder;
    }

    public GenericEncoder(CANEncoder canEncoder) {
        this.canEncoder = canEncoder;
    }

    public void setMaxPeriod(double period) {
        if (encoder != null) {
            encoder.setMaxPeriod(period);
        }
    }

    public void reset() {
        if (encoder != null) {
            encoder.reset();
        }
    }

    public boolean getStopped() {
        boolean result;
        if (encoder != null) {
            result = encoder.getStopped();
        } else {
            result = canEncoder.getVelocity() == 0;
        }
        return result;
    }

    public int get() {
        int result;
        if (encoder != null) {
            result = encoder.get();
        } else {
            result = (int)Math.round(canEncoder.getPosition());
        }
        return result;
    }

    public boolean getDirection() {
        boolean result = true;
        if (encoder != null) {
            result = encoder.getDirection();
        }
        return result;
    }

    public double getPeriod() {
        double result;
        if (encoder != null) {
            result = encoder.getRate();
        } else {
            result = canEncoder.getVelocity();
        }
        return result;
    }
}