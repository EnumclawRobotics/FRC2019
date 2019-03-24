
package frc.robot.Components;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.*;

public class Operator {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Operator");
    
    public XboxController driveXboxController;
    public XboxController armXboxController; 
    public Joystick armButtons; 

    private final int armFrontButtonNumber = 6;
    private final int armBackButtonNumber = 2;

    private final int cargoRocket1ButtonNumber = 4;
    private final int cargoRocket2ButtonNumber = 5;
    private final int cargoDepotButtonNumber = 7;
    private final int cargoShipButtonNumber = 8; 
    private final int cargoStationButtonNumber = 9; 

    private final int hatchRocket1ButtonNumber = 1;
    private final int hatchRocket2ButtonNumber = 3;

    public JoystickButton armFrontButton;
    public JoystickButton armBackButton;

    public JoystickButton cargoDepotButton;
    public JoystickButton cargoRocket1Button;
    public JoystickButton cargoRocket2Button;
    public JoystickButton cargoShipButton; 
    public JoystickButton cargoStationButton; 

    public JoystickButton hatchRocket1Button;
    public JoystickButton hatchRocket2Button;
    
    public Operator(RobotMap robotMap) {
        driveXboxController = robotMap.driveXboxController;        
        armXboxController = robotMap.armXboxController;
        armButtons = robotMap.armButtons;

        armFrontButton = new JoystickButton(armButtons, armFrontButtonNumber);
        armBackButton = new JoystickButton(armButtons, armBackButtonNumber);

        cargoDepotButton = new JoystickButton(armButtons, cargoDepotButtonNumber);
        cargoRocket1Button = new JoystickButton(armButtons, cargoRocket1ButtonNumber);
        cargoRocket2Button = new JoystickButton(armButtons, cargoRocket2ButtonNumber);
        cargoShipButton = new JoystickButton(armButtons, cargoShipButtonNumber);
        cargoStationButton = new JoystickButton(armButtons, cargoStationButtonNumber);

        hatchRocket1Button = new JoystickButton(armButtons, hatchRocket1ButtonNumber);
        hatchRocket2Button = new JoystickButton(armButtons, hatchRocket2ButtonNumber);
    }

    public void run() {
        putTelemetry();
    }

    public void putTelemetry() {
        telemetry.putDouble("Speed (forward|back)", driveXboxController.getY(Hand.kLeft));
        telemetry.putDouble("Turn (left|right)", driveXboxController.getX(Hand.kRight));
        telemetry.putDouble("Arm", armXboxController.getY(Hand.kLeft));
        telemetry.putDouble("Wrist", armXboxController.getY(Hand.kRight));
        telemetry.putBoolean("Arm Front", armFrontButton.get());
        telemetry.putBoolean("Arm Back", armBackButton.get());
        telemetry.putBoolean("Cargo Floor", cargoDepotButton.get());
        telemetry.putBoolean("Cargo Rocket1", cargoRocket1Button.get());
        telemetry.putBoolean("Cargo Rocket2", cargoRocket2Button.get());
        telemetry.putBoolean("Cargo Ship", cargoShipButton.get());
        telemetry.putBoolean("Cargo Station", cargoStationButton.get());
        telemetry.putBoolean("Hatch Rocket1 | Hatch Ship | Hatch Station", hatchRocket1Button.get());
        telemetry.putBoolean("Hatch Rocket2", hatchRocket2Button.get());
        telemetry.putString("Version", "1.0.0");
    }
}
