
package frc.robot.Components;

import common.util.*;
import common.i2cSensors.*;
import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.HardwareMap;

public class Operator {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Operator");
    
    public Joystick speedJoystick;
    public Joystick turnJoystick;
    public Joystick heightJoystick; 

    public Operator(HardwareMap hardwareMap) {
        speedJoystick = hardwareMap.speedJoystick;
        turnJoystick = hardwareMap.turnJoystick;
        heightJoystick = hardwareMap.heightJoystick;
    }

    public void putTelemetry() {
        telemetry.putDouble("Speed (forward|back)", speedJoystick.getY());
        telemetry.putDouble("Turn (left|right)", turnJoystick.getX());
        telemetry.putDouble("Height (position)", heightJoystick.getY());
        telemetry.putString("Version", "1.0.0");
    }
}
