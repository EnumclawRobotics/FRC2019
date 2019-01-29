package common.i2cSensors;

public class MRGyro extends I2CSensor {

	// default address 
	public MRGyro() {
		super(0x10);   // address is 7bits for RoboRio code (8bit is 0x20)
	}
	
	// alternate address. use corediscoverymodule to find and change.
	public MRGyro(int i2cAddress) {
		super(i2cAddress);	// address is 7bits for RoboRio code
	}

	/*
	* Resets the z-integrator to 0
	*/
	public void reset() {
		writeByte(0x03, 0x52);
	} 

	/*
	*  May take up to 3 seconds. Only do in robot init or robot constructor.
	*  TODO: Maybe add callback code to check whether done and update status. 
	*  NOTE: 'Register 0x3 goes back to 0 when done calibrating'.
	*/
	public void calibrate() {
		writeByte(0x03, 0x4E);
	}

	/*
	*  Calibrating may take up to 3 seconds. Only do in robot init or robot constructor.
	*/
	public boolean isDoneCalibrating() {
		return (readByte(0x03) == 0);
	}

	// get heading
	public int getIntegratedZ() {
		return readLsbMsb(0x06);
	}
	
	// get X
	public int getX() {
		return readLsbMsb(0x08);
	}

	// get Y
	public int getY() {
		return readLsbMsb(0x0A);
	}
}