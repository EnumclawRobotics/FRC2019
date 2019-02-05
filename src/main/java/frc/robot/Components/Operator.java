
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
    
    public Joystick armJoystick; 
    public ToggleButton armFacing;

    public Operator(RobotMap robotMap) {

        xboxController = robotMap.xboxController;

        speedJoystick = robotMap.speedJoystick;
        turnJoystick = robotMap.turnJoystick;
        driveFacing = new ToggleButton(speedJoystick, 2);        // thumb reverses direction
        
        armJoystick = robotMap.heightJoystick;
        armFacing = new ToggleButton(armJoystick, 2);         // thumb reverses direction
    }

    public void putTelemetry() {
        telemetry.putBoolean("Drive Facing (toggle)", driveFacing.toggleOn());
        telemetry.putDouble("Speed (forward|back)", speedJoystick.getY());
        telemetry.putDouble("Turn (left|right)", turnJoystick.getX());
        telemetry.putBoolean("Arm Facing (toggle)", armFacing.toggleOn());
        telemetry.putString("Version", "1.0.0");
    }


}
