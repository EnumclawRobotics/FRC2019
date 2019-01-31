
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

  final int wristPwmPort = 3;
  final int grabberPwmPort = 4;
  final int cargoRollerPwmPort = 5;          // depending on motor size can Y split the power to both motors 

  // CAN Device IDs
  final int leftShoulderCanId = 2;            // SparkMax-Neo requires CAN otherwise buggy
  final int rightShoulderCanId = 3;           // SparkMax-Neo requires CAN otherwise buggy

  // DIO Ports
  final int shoulderLimitSwitchDioPort = 0;
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
  public int[] cargoColor = new int[] { 0, 0, 0, 0 };     // TODO: Fill in accurate color values 
  public double cargoColorThreshold = .1d;

  // arm geometries (in inches)
  public double lengthArm = 41;                   // TODO: Confirm this
  public double heightArmPivot = 46;              // TODO: Confirm this

  // field geometries
  public double whiteLineLength = 18;

  public double heightHatchRocket1 = 19;
  public double heightHatchRocket2 = 47;
  public double heightHatchRocket3 = 75;
  
  public double heightCargoRocket1 = 27.5;
  public double heightCargoRocket2 = 55.5;
  public double heightCargoRocket3 = 83.5;
  
  public double heightCargoFloor = 9.250;             // requires wrist to be straight with arm instead of held level
  public double heightHatchStation = 19;
  public double heightCargoStation = 44.125;
  
  public double heightCargoShip = 38.75;
  public double heightHatchShip = 19;
  
  // arm 
  public double armFeedFowardFactor = .5;         // use to multiply cos(arm angle) to find holding power
  public double wristFeedFowardFactor = .5;       // use to multiply cos(wrist angle) to find holding power

  // hab geometries
  public double heightHabLevel1 = 3; 
  public double heightHabLevel2 = 6;
  public double heightHabLevel3 = 19;

  // game piece geometries
  public double hatchDiameter = 19;
  public double hatchHoleDiameter = 6;
  public double cargoDiameter = 13;

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
