package common.oiHelpers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class XboxToggleButton {
	XboxController xboxController;			        // joystick reference
	String buttonName;						        // raw button number
	double debouncePeriod = .25;			        // seconds to wait before recognizing a subsequent press

	boolean toggleOn = false;
	boolean buttonBeingHeld = false;
	double debounceUntil = 0;

	// holds joystick reference and button number
	public XboxToggleButton(XboxController xboxController, String buttonName) {
		this.xboxController = xboxController;
		this.buttonName = buttonName;
	}
	
	// holds joystick reference and button number and time to ignore subsequent accidental button presses
	public XboxToggleButton(XboxController xboxController, String buttonName, double debouncePeriod) {
		this.xboxController = xboxController;
		this.buttonName = buttonName;
		this.debouncePeriod = debouncePeriod;
	}

	// updates button and returns whether is currently toggled on
	public boolean updateToggle() {
		double now = Timer.getFPGATimestamp();
		if (now > debounceUntil) {
			if(isButtonPressed()) {
				if(!buttonBeingHeld) {
					toggleOn = !toggleOn;
					buttonBeingHeld = true;
					debounceUntil = now + debouncePeriod;
				}
			}else{
				buttonBeingHeld = false;
			}
		}
		return toggleOn();		
	}

	// returns whether toggled on
	public boolean toggleOn() {
		return toggleOn;
    }
    
    private boolean isButtonPressed() {
        boolean result;

        switch (buttonName.toUpperCase()) {
            case "A":
                result = xboxController.getAButton();
                break;
            case "B":
                result = xboxController.getBButton();
                break;
            case "X":
                result = xboxController.getXButton();
                break;
            case "Y":
                result = xboxController.getYButton();
                break;
            case "START":
                result = xboxController.getStartButton();
                break;
            case "BACK":
                result = xboxController.getBButton();
                break;
            default: 
                result = false;
                break;
        }
        return result;        
    }

}