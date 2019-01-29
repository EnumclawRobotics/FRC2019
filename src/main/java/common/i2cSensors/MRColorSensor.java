package common.i2cSensors;

public class MRColorSensor extends I2CSensor {

	// default address
	public MRColorSensor() {
		super(0x1E);   // address is 7bits for RoboRio code (8bit is 3C)
	}
	
	// alternate address
	public MRColorSensor(int i2cAddress) {
		super(i2cAddress);	// address is 7bits for RoboRio code
	}

    // Enable Active Mode (turns on led) use to detect color of an object
    public void setActiveModeEnabled() {
        writeByte(0x93, 0x00);
    }

    // Enable Passive Mode (no led) use to detect color of a light source
    public void setPassiveModeEnabled() {
        writeByte(0x93, 0x01);
    }

	// ColorNumber 0-16
	public int getColorNumber() {
		return readByte(0x04);
    }
    
    // Red Normalized 0-16
    public int getRed() {
        return readLsbMsb(0x0E);
    }

    // Green Normalized 0-16
    public int getGreen() {
        return readLsbMsb(0x10);
    }

    // Blue Normalized 0-16
    public int getBlue() {
        return readLsbMsb(0x12);
    }

    // White 0-16
    public int getWhite() {
        return readLsbMsb(0x08);
    }
 
    // gets array of color features R,G,B,W
    public int[] getColor() {
        return createColor(getRed(), getBlue(), getGreen(), getWhite()); 
    }

    // creates array of color features
    public int[] createColor(int red, int green, int blue, int white) {
        return new int[] { getRed(), getBlue(), getGreen(), getWhite() };
    }

}