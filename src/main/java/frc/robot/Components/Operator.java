
package frc.robot.Components;

import common.instrumentation.Telemetry;
//import common.oiHelpers.JoystickButton;
import common.oiHelpers.ToggleButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.*;

public class Operator {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Operator");
    
    public XboxController xboxController;
    public ToggleButton driveFacing;
    
    public Joystick armJoystick; 
    public Joystick armButtons; 

    private final int floorCargoButtonNumber = 1;
    private final int rocketCargo1ButtonNumber = 2;
    private final int rocketCargo2ButtonNumber = 3;
    private final int rocketCargo3ButtonNumber = 4;
    private final int shipCargoButtonNumber = 5; 
    private final int stationCargoButtonNumber = 6; 

    private final int rocketHatch1ButtonNumber = 7;
    private final int rocketHatch2ButtonNumber = 8;
    private final int rocketHatch3ButtonNumber = 9;
    private final int shipHatchButtonNumber = 10; 
    private final int stationHatchButtonNumber = 11; 

    private final int normalFacingButtonNumber = 11;
    private final int invertedFacingButtonNumber = 12;

    public JoystickButton floorCargoButton;
    public JoystickButton rocketCargo1Button;
    public JoystickButton rocketCargo2Button;
    public JoystickButton rocketCargo3Button;
    public JoystickButton shipCargoButton; 
    public JoystickButton stationCargoButton; 

    public JoystickButton rocketHatch1Button;
    public JoystickButton rocketHatch2Button;
    public JoystickButton rocketHatch3Button;
    public JoystickButton shipHatchButton; 
    public JoystickButton stationHatchButton;
    
    public JoystickButton normalFacingButton;
    public JoystickButton invertedFacingButton;

    public Operator(RobotMap robotMap) {

        xboxController = robotMap.xboxController;

//        driveFacing = new ToggleButton(xboxController., 2);        // thumb reverses direction
        
        armJoystick = robotMap.armJoystick;
        armButtons = robotMap.armButtons;
        
        floorCargoButton = new JoystickButton(armButtons, floorCargoButtonNumber);
        rocketCargo1Button = new JoystickButton(armButtons, rocketCargo1ButtonNumber);
        rocketCargo2Button = new JoystickButton(armButtons, rocketCargo2ButtonNumber);
        rocketCargo3Button = new JoystickButton(armButtons, rocketCargo3ButtonNumber);
        shipCargoButton = new JoystickButton(armButtons, shipCargoButtonNumber);
        stationCargoButton = new JoystickButton(armButtons, stationCargoButtonNumber);

        rocketHatch1Button = new JoystickButton(armButtons, rocketHatch1ButtonNumber);
        rocketHatch2Button = new JoystickButton(armButtons, rocketHatch2ButtonNumber);
        rocketHatch3Button = new JoystickButton(armButtons, rocketHatch3ButtonNumber);
        shipHatchButton = new JoystickButton(armButtons, shipHatchButtonNumber);
        stationHatchButton = new JoystickButton(armButtons, stationHatchButtonNumber);
        
        normalFacingButton = new JoystickButton(armButtons, normalFacingButtonNumber);
        invertedFacingButton = new JoystickButton(armButtons, invertedFacingButtonNumber);
    }

    public void putTelemetry() {
        telemetry.putBoolean("Drive Facing (toggle)", driveFacing.toggleOn());
        telemetry.putDouble("Speed (forward|back)", xboxController.getY());
        telemetry.putDouble("Turn (left|right)", xboxController.getX());
        telemetry.putBoolean("Cargo Floor", floorCargoButton.get());
        telemetry.putBoolean("Cargo Rocket1", rocketCargo1Button.get());
        telemetry.putBoolean("Cargo Rocket2", rocketCargo2Button.get());
        telemetry.putBoolean("Cargo Rocket3", rocketCargo3Button.get());
        telemetry.putBoolean("Cargo Ship", shipCargoButton.get());
        telemetry.putBoolean("Cargo Station", stationCargoButton.get());
        telemetry.putBoolean("Hatch Rocket1", rocketHatch1Button.get());
        telemetry.putBoolean("Hatch Rocket2", rocketHatch2Button.get());
        telemetry.putBoolean("Hatch Rocket3", rocketHatch3Button.get());
        telemetry.putBoolean("Hatch Ship", shipHatchButton.get());
        telemetry.putBoolean("Hatch Station", stationHatchButton.get());
        telemetry.putString("Version", "1.0.0");
    }
}
