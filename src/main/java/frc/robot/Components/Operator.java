
package frc.robot.Components;

import common.instrumentation.Telemetry;
import common.oiHelpers.ToggleButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.*;

public class Operator {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Operator");
    
    public XboxController xboxController;

    public Joystick speedJoystick;
    public Joystick turnJoystick;
    public ToggleButton driveFacing;
    public ToggleButton armFacing;
    
    public Joystick armJoystick; 

    public Operator(RobotMap robotMap) {

        xboxController = robotMap.xboxController;

        driveFacing = new ToggleButton(speedJoystick, 2);        // thumb reverses direction
        speedJoystick = robotMap.speedJoystick;
        turnJoystick = robotMap.turnJoystick;
        
        armFacing = new ToggleButton(armJoystick, 2);         // thumb reverses direction
        armJoystick = robotMap.heightJoystick;
    }

    public void putTelemetry() {
        telemetry.putBoolean("Drive Facing (toggle)", driveFacing.toggleOn());
        telemetry.putDouble("Speed (forward|back)", speedJoystick.getY());
        telemetry.putDouble("Turn (left|right)", turnJoystick.getX());
        telemetry.putBoolean("Arm Facing (toggle)", armFacing.toggleOn());
        telemetry.putString("Version", "1.0.0");
    }
}
