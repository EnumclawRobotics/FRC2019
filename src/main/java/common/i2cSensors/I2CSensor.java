package common.i2cSensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;

public class I2CSensor {

	I2C i2cSensor;
	
	// default address
	public I2CSensor() {
		i2cSensor = new I2C(I2C.Port.kOnboard, 0x00);	// address is 7bits for RoboRio code
	}
	
	// additional address
	public I2CSensor(int i2cAddress) {
		i2cSensor = new I2C(I2C.Port.kOnboard, i2cAddress);	// address is 7bits for RoboRio code
	}

	protected int readByte(int register) {
		byte[] data = new byte[1];
		i2cSensor.read(register, 1, data);
		
		return Byte.toUnsignedInt(data[0]);			// alternate is using && 0xFF
	}

	// getting lsb/msb from 16 bits
	protected int readLsbMsb(int register) {
		byte[] data = new byte[2];
		i2cSensor.read(register, 2, data);
		
		ByteBuffer buffer = ByteBuffer.wrap(data);
		buffer.order(ByteOrder.LITTLE_ENDIAN);		// uses 2 bytes in reverse order
		return buffer.getShort();
	}

	// getting 0.0-255.0 from 16 bits
	protected double readFsbLsb(int register) {
		byte[] data = new byte[2];
		i2cSensor.read(register, 2, data);

		ByteBuffer buffer = ByteBuffer.wrap(data);
		buffer.order(ByteOrder.BIG_ENDIAN);			// uses 2 bytes in normal order
		return buffer.getShort()/256d;				// divide by 256 for fsb/lsb
	}

	protected void writeByte(int register, int value) {
		i2cSensor.write(register, value);
	}
}
