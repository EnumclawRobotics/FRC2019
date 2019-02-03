
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
  final int leftDrivePwmPort = 0;            // 40AMP x2 pwm group - use Y split to signal both front and back motor controller
  final int rightDrivePwmPort = 1;           // 40AMP x2 pwm group - use Y split to signal both front and back motor controller

  final int wristPwmPort = 3;                // 30AMP
  final int grabberPwmPort = 4;              // 30AMP
  final int cargoRollerPwmPort = 5;          // 30AMP depending on motor size can Y split the power to both motors 

  // CAN Device IDs
  final int leftArmCanId = 2;                 // 40AMP SparkMax-Neo requires CAN otherwise buggy
  final int rightArmCanId = 3;                // 40AMP SparkMax-Neo requires CAN otherwise buggy

  // DIO Ports
  final int armLimitSwitchDioPort = 0;
 
  final int wristLimitSwitchDioPort = 1;
  final int wristEncoderADioPort = 2;
  final int wristEncoderBDioPort = 3;

  final int grabberLimitSwitchDioPort = 4;
  final int grabberEncoderADioPort = 5;
  final int grabberEncoderBDioPort = 6;

  final int hatchLimitSwitchDioPort = 7;      // tie the two limit switches together so that either pressed causes trigger 
  
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
  public final static double armFeedForwardFactor = .5;                   // use to multiply cos(arm angle) to find holding power
  public final static double armEncoderCicksPerDegree = 4200/360;         // NEO gearbox output shaft include gear reduction  

  // wrist
  public final static double wristLength = 12;                            // TODO: Confirm 
  public final static double wristEncoderClicksPerDegree = 1316/360;      // PG188 output shaft include gear reduction
  public final static double wristFeedForwardFactor = .5;                 // use to multiply cos(wrist angle) to find holding power

  // grabber
  public final static double grabberLength = 7; 
  public final static double grabberEncoderClicksPerDegree = 1316/360;    // PG188 output shaft include gear reduction


  // === REFERENCES ======================

  // SpeedControllers
  public Spark leftDriveSpeedController;
  public Spark rightDriveSpeedController;
  
  public CANSparkMax leftArmSpeedController; 
  public CANSparkMax rightArmSpeedController; 
  public Spark wristSpeedController;

  public Spark grabberSpeedController;
  public Spark cargoRollerSpeedController;                    // TODO: Set brake mode on roller controller

  // sensors
  public ADXRS450_Gyro driveGyro;
  
  public CANEncoder armEncoder;
  public DigitalInput armLimitSwitch;

  public Encoder wristEncoder;
  public DigitalInput wristLimitSwitch;

  public Encoder grabberEncoder;
  public DigitalInput grabberLimitSwitch;
  
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
    leftArmSpeedController = new CANSparkMax(leftArmCanId, MotorType.kBrushless);
    rightArmSpeedController = new CANSparkMax(rightArmCanId, MotorType.kBrushless);
    armEncoder = new CANEncoder(leftArmSpeedController);
    armLimitSwitch = new DigitalInput(armLimitSwitchDioPort);

    // wrist
    wristSpeedController = new Spark(wristPwmPort); 
    wristEncoder = new Encoder(wristEncoderADioPort, wristEncoderBDioPort);
    wristLimitSwitch = new DigitalInput(wristLimitSwitchDioPort);

    // grabber
    grabberSpeedController = new Spark(grabberPwmPort);
    grabberEncoder = new Encoder(grabberEncoderADioPort, grabberEncoderBDioPort);
    grabberLimitSwitch = new DigitalInput(grabberLimitSwitchDioPort);

    // cargo handler
    cargoRollerSpeedController = new Spark(cargoRollerPwmPort);
    cargoColorSensor = new MRColorSensor();

    // hatch handler
    hatchLimitSwitch = new DigitalInput(hatchLimitSwitchDioPort);
  }
}
