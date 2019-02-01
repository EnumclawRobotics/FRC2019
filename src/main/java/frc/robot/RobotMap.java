
package frc.robot;

import edu.wpi.first.wpilibj.*;
import common.i2cSensors.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Hardware specifics 
// NOTE: Components will use super classes to hide the hardware specifics.
//       One exception is CAN versus PWM speedcontrollers
// TODO: Fill in Javadocs for all calls
public class RobotMap {

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

  final int hatchLimitSwitchDioPort = 7;
  
  // USB Ports (driver station)
  final int speedJoystickUsb = 0;
  final int turnJoystickUsb = 1;
  final int heightJoystickUsb = 2;


  // TODO: Add buttons to control height


  // === CONSTANTS =====================================

  // safety
  public final static double safetyExpiration = .25;

  // arm geometries (in inches)
  public final static double armLength = 38; 
  public final static double heightArmPivot = 45;

  // arm 
  public final static double armFeedForwardFactor = .5;         // use to multiply cos(arm angle) to find holding power
  public final static double wristFeedForwardFactor = .5;       // use to multiply cos(wrist angle) to find holding power
  public final static double armEncoderCicksPerRev = 4200;     // NEO gearbox output shaft include gear reduction  

  // wrist
  public final static double wristEncoderClicksPerRev = 1316;    // PG188 output shaft include gear reduction

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
  public Encoder wristEncoder;
  
  public DigitalInput shoulderLimitSwitch;
  public DigitalInput wristLimitSwitch;
  
  public DigitalInput hatchLimitSwitch;     // TODO: Use two limitswitches on physical bot just linked together
  
  public MRColorSensor cargoColorSensor;

  // usb
  public Joystick speedJoystick;  
  public Joystick turnJoystick;  
  public Joystick heightJoystick;  

  // setup subsystems
  public RobotMap() {
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
    hatchLimitSwitch = new DigitalInput(hatchLimitSwitchDioPort);
  }
}
