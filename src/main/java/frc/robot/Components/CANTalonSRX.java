package frc.robot.Components;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.*;

public class CANTalonSRX extends TalonSRX implements SpeedController {
    public CANTalonSRX(int canId) {
        super(canId);
    }

    public void set(double power) {
        super.set(ControlMode.PercentOutput, power);
    }

    public double get() { 
        return super.getMotorOutputPercent(); 
    }

    public void pidWrite(double power) {}

    public void disable() { 
        stopMotor(); 
    }

    public void stopMotor() {
        set(0);
    }
}