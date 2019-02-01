
package frc.robot;

import edu.wpi.first.wpilibj.*;
import common.i2cSensors.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// important numbers and hardware specifics. 
// NOTE: Components will use super classes to hide the hardware specifics.
//       One exception is CAN versus PWM speedcontrollers
public class FieldMap {

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


  // === CONSTANTS =====================================

  // safety
  public static double safetyExpiration = .25;

  // colors / constants
  public static int[] cargoColor = new int[] { 0, 0, 0, 0 };     // TODO: Fill in accurate color values 
  public static double cargoColorThreshold = .1d;

  // arm geometries (in inches)
  public static double lengthArm = 38; 
  public static double heightArmPivot = 45;

  // field geometries
  public static double whiteLineLength = 18;

  public static double heightHatchRocket1 = 19;
  public static double heightHatchRocket2 = 47;
  public static double heightHatchRocket3 = 75;
  
  public static double heightCargoRocket1 = 27.5;
  public static double heightCargoRocket2 = 55.5;
  public static double heightCargoRocket3 = 83.5;
  
  public static double heightCargoFloor = 9.250;             // requires wrist to be straight with arm instead of held level
  public static double heightHatchStation = 19;
  public static double heightCargoStation = 44.125;
  
  public static double heightCargoShip = 38.75;
  public static double heightHatchShip = 19;
  
  // arm 
  public static double armFeedFowardFactor = .5;         // use to multiply cos(arm angle) to find holding power
  public static double wristFeedFowardFactor = .5;       // use to multiply cos(wrist angle) to find holding power
  public static double armEncoderCicksPerRev = 4200;     // NEO gearbox output shaft include gear reduction  

  // wrist
  public static double wristEncoderClicksPerRev = 1316;    // PG188 output shaft include gear reduction

  // hab geometries
  public static double heightHabLevel1 = 3; 
  public static double heightHabLevel2 = 6;
  public static double heightHabLevel3 = 19;

  // game piece geometries
  public static double hatchDiameter = 19;
  public static double hatchHoleDiameter = 6;
  public static double cargoDiameter = 13;


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

  // setup subsystems
  public FieldMap() {
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
