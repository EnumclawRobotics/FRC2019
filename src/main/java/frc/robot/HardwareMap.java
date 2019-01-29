
package frc.robot;

import edu.wpi.first.wpilibj.*;
import common.i2cSensors.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// important numbers and hardware specifics. 
// NOTE: Components will use super classes to hide the hardware specifics.
//       One exception is CAN versus PWM speedcontrollers
public class HardwareMap {

  // === ADDRESSES/PLUGINS ============================
  
  // PWM Ports
  final int leftDrivePwmPort = 0;            // pwm group - use Y split to signal both front and back motor controller
  final int rightDrivePwmPort = 1;           // pwm group - use Y split to signal both front and back motor controller

  final int elbowPwmPort = 4;
  final int wristPwmPort = 5;

  final int grabberPwmPort = 6;
  final int cargoRollerPwmPort = 7;          // depending on motor size can Y split the power to both motors 

  // CAN Device IDs
  final int roboRioCanId = 0;                 // one end of bus
  final int powerDistributionModuleCanId = 1; // other end of bus - remember to set CAN jumper
  final int leftShoulderCanId = 2;           // if using SparkMax can only use CAN version with NEO
  final int rightShoulderCanId = 3;          // if using SparkMax can only use CAN version with NEO

  // DIO Ports
  final int shoulderLimitSwitchDioPort = 0;
  final int elbowLimitSwitchDioPort = 1;
  final int wristLimitSwitchDioPort = 2;

  final int wristEncoderADioPort = 5;
  final int wristEncoderBDioPort = 6;

  final int hatchALimitSwitchDioPort = 7;
  final int hatchBLimitSwitchDioPort = 8;

  // USB Ports (driver station)
  final int speedJoystickUsb = 0;
  final int turnJoystickUsb = 1;
  final int heightJoystickUsb = 2;

  // === REFERENCES ======================

  // SpeedControllers
  public Spark leftDriveSpeedController;
  public Spark rightDriveSpeedController;
  
  public CANSparkMax leftShoulderSpeedController; 
  public CANSparkMax rightShoulderSpeedController; 
  public Spark wristSpeedController;
  
  public Spark grabberSpeedController;
  public Spark cargoRollerSpeedController;

  // sensors
  public ADXRS450_Gyro driveGyro;
  
  public CANEncoder shoulderEncoder;
  public Encoder elbowEncoder;
  public Encoder wristEncoder;
  
  public DigitalInput shoulderLimitSwitch;
  public DigitalInput elbowLimitSwitch;
  public DigitalInput wristLimitSwitch;
  
  public DigitalInput hatchALimitSwitch;
  public DigitalInput hatchBLimitSwitch;

  public MRColorSensor cargoColorSensor;

  // usb
  public Joystick speedJoystick;  
  public Joystick turnJoystick;  
  public Joystick heightJoystick;  

  // safety
  public double safetyExpiration = .25;

  // colors / constants
  public int[] cargoColor = new int[] { 0, 0,0,0 };     // TODO: Fill in accurate color values 
  public double cargoColorThreshold = .1d;
  
  // setup subsystems
  public HardwareMap() {
    // operator
    speedJoystick = new Joystick(speedJoystickUsb);
    turnJoystick = new Joystick(turnJoystickUsb);
    heightJoystick = new Joystick(heightJoystickUsb);

    // drive 
    leftDriveSpeedController = new Spark(leftDrivePwmPort);
    rightDriveSpeedController = new Spark(rightDrivePwmPort);
    driveGyro = new ADXRS450_Gyro();

    // arm
    leftShoulderSpeedController = new CANSparkMax(leftShoulderCanId, MotorType.kBrushless);
    rightShoulderSpeedController = new CANSparkMax(rightShoulderCanId, MotorType.kBrushless);

    shoulderEncoder = new CANEncoder(leftShoulderSpeedController);

    wristSpeedController = new Spark(wristPwmPort); 
    wristEncoder = new Encoder(wristEncoderADioPort, wristEncoderBDioPort);

    // grabber
    grabberSpeedController = new Spark(grabberPwmPort);

    // cargo handler
    cargoRollerSpeedController = new Spark(cargoRollerPwmPort);
    cargoColorSensor = new MRColorSensor();

    // hatch handler
    hatchALimitSwitch = new DigitalInput(hatchALimitSwitchDioPort);
    hatchBLimitSwitch = new DigitalInput(hatchBLimitSwitchDioPort);
  }
}
