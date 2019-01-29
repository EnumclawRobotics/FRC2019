
package frc.robot.Components;

import common.instrumentation.Telemetry;
import common.oiHelpers.ToggleButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.HardwareMap;

public class Operator {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Operator");
    
    public Joystick speedJoystick;
    public Joystick turnJoystick;
    public ToggleButton driveFacing;
    public ToggleButton armFacing;
    
    public Joystick heightJoystick; 

    public Operator(HardwareMap hardwareMap) {
        driveFacing = new ToggleButton(speedJoystick, 2);        // thumb reverses direction
        speedJoystick = hardwareMap.speedJoystick;
        turnJoystick = hardwareMap.turnJoystick;
        
        armFacing = new ToggleButton(heightJoystick, 2);         // thumb reverses direction
        heightJoystick = hardwareMap.heightJoystick;
    }

    public void putTelemetry() {
        telemetry.putBoolean("Drive Facing (toggle)", driveFacing.toggleOn());
        telemetry.putDouble("Speed (forward|back)", speedJoystick.getY());
        telemetry.putDouble("Turn (left|right)", turnJoystick.getX());
        telemetry.putBoolean("Arm Facing (toggle)", driveFacing.toggleOn());
        telemetry.putDouble("Height (position)", heightJoystick.getY());
        telemetry.putString("Version", "1.0.0");
    }
}
