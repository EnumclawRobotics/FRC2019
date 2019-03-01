package common.oiHelpers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

public class JoystickToggleButton {
	Joystick joystick;						// joystick reference
	int buttonNumber;						// raw button number
	double debouncePeriod = .25;			// seconds to wait before recognizing a subsequent press

	boolean toggleOn = false;
	boolean buttonBeingHeld = false;
	double debounceUntil = 0;

	// holds joystick reference and button number
	public JoystickToggleButton(Joystick joystick, int buttonNumber) {
		this.joystick = joystick;
		this.buttonNumber = buttonNumber;
	}
	
	// holds joystick reference and button number and time to ignore subsequent accidental button presses
	public JoystickToggleButton(Joystick joystick, int buttonNumber, double debouncePeriod) {
		this.joystick = joystick;
		this.buttonNumber = buttonNumber;
		this.debouncePeriod = debouncePeriod;
	}

	// updates button and returns whether is currently toggled on
	public boolean updateToggle() {
		double now = Timer.getFPGATimestamp();
		if (now > debounceUntil) {
			if(joystick.getRawButton(buttonNumber)) {
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
}