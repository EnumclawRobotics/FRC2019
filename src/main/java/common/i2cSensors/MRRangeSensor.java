package common.i2cSensors;

public class MRRangeSensor extends I2CSensor {

	// default address 
	public MRRangeSensor() {
		super(0x14);            // address is 7bits for RoboRio code (8bit is 0x28)
	}
	
	// alternate address. use corediscoverymodule to find and change.
	public MRRangeSensor(int i2cAddress) {
		super(i2cAddress);	    // address is 7bits for RoboRio code
	}

	/*
	* reads the ultrasonic distance in cm. 5cm-255cm
    */
	public double getDistance() {
		return readByte(0x04);
		//return readFsbLsb(0x04);
	}
}