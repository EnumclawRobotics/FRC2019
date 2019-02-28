package frc.robot.Components;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.*;

public class CANEncoder2 extends CANEncoder implements CounterBase {
    public CANEncoder2(CANSparkMax sparkMax) {
        super(sparkMax);
    }

    public void setMaxPeriod(double period) {
    }

    public void reset() {
    }

    public boolean getStopped() {
        return super.getVelocity() == 0;
    }

    public int get() {
        return (int)Math.round(super.getPosition());
    }

    public boolean getDirection() {
        boolean result = true;
        return result;
    }

    public double getPeriod() {
        return super.getVelocity();
    }
}