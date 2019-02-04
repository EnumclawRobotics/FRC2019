
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
  final int leftDrivePwm = 0;            // 40AMP x2 pwm group - use Y split to signal both front and back motor controller
  final int rightDrivePwm = 1;           // 40AMP x2 pwm group - use Y split to signal both front and back motor controller

  final int wristPwm = 3;                // 30AMP
  final int grabberPwm = 4;              // 30AMP
  final int cargoRollerPwm = 5;          // 30AMP depending on motor size can Y split the power to both motors 

  // CAN Device IDs
  final int leftArmCan = 2;                 // 40AMP SparkMax-Neo requires CAN otherwise buggy
  final int rightArmCan = 3;                // 40AMP SparkMax-Neo requires CAN otherwise buggy

  // DIO Ports
  final int armLimitSwitchDio = 0;
 
  final int wristLimitSwitchDio = 1;
  final int wristEncoderADio = 2;
  final int wristEncoderBDio = 3;

  final int grabberLimitSwitchDio = 4;
  final int grabberEncoderADio = 5;
  final int grabberEncoderBDio = 6;

  final int hatchLimitSwitchDio = 7;      // tie the two limit switches together so that either pressed causes trigger 
  
  // USB Ports (driver station)
  final int xboxControllerUsb = 0;
  final int speedJoystickUsb = 1;
  final int turnJoystickUsb = 2;
  final int heightJoystickUsb = 3;


  // TODO: Add buttons to control height


  // === CONSTANTS =====================================

  // safety
  public final static double safetyExpiration = .25d;

  // arm geometries (in inches)
  public final static double armLength = 38d; 
  public final static double heightArmPivot = 45d;
  public final static double armFeedForwardFactor = .5d;                     // hold at horizontal power. find by testing
  public final static double armKpFactor = .1d;                               // PID kP correction factor 
  public final static double armEncoderClicksPerDegree = 4200d/360d;          // NEO gearbox output shaft include gear reduction  

  // wrist
  public final static double wristLength = 7.75d; 
  public final static double wristEncoderClicksPerDegree = 1316d/360d;        // PG188 output shaft include gear reduction
  public final static double wristFeedForwardFactor = .5d;                  // hold at horizontal power. find by testing
  public final static double wristKpFactor = .5d;                               // PID kP correction factor
  public final static double wristHeldAngle = 10d;                           // angle to fold back the grabber for protection

  // grabber
  public final static double grabberLength = 10d;
  public final static double grabberEncoderClicksPerDegree = 1316d/360d;    // PG188 output shaft include gear reduction


  // === REFERENCES ======================

  // SpeedControllers
  public Spark leftDriveSpeedController;
  public Spark rightDriveSpeedController;
  public CANSparkMax leftArmSpeedController; 
  public CANSparkMax rightArmSpeedController; 
  public PWMVictorSPX wristSpeedController;
  public PWMVictorSPX grabberSpeedController;
  public PWMVictorSPX cargoRollerSpeedController;                    // TODO: Set brake mode on roller controller

  // sensors
  public ADXRS450_Gyro driveGyro;
  
  public CANEncoder armEncoder;
  public DigitalInput armLimitSwitch;

  public Encoder wristEncoder;
  public DigitalInput wristLimitSwitch;

  public Encoder grabberEncoder;
  public DigitalInput grabberLimitSwitch;
  
  public DigitalInput hatchLimitSwitch;                 // TODO: Use two limit switches on physical bot just linked together
  
  public MRColorSensor cargoColorSensor;

  // usb
  public XboxController xboxController;
  public Joystick speedJoystick;  
  public Joystick turnJoystick;  
  public Joystick heightJoystick;  

  // setup subsystems
  public RobotMap() {
    // operator
    xboxController = new XboxController(xboxControllerUsb);
    speedJoystick = new Joystick(speedJoystickUsb);
    turnJoystick = new Joystick(turnJoystickUsb);
    heightJoystick = new Joystick(heightJoystickUsb);

    // drive 
    leftDriveSpeedController = new Spark(leftDrivePwm);
    rightDriveSpeedController = new Spark(rightDrivePwm);
    driveGyro = new ADXRS450_Gyro();

    // arm
    leftArmSpeedController = new CANSparkMax(leftArmCan, MotorType.kBrushless);
    rightArmSpeedController = new CANSparkMax(rightArmCan, MotorType.kBrushless);
    armEncoder = new CANEncoder(leftArmSpeedController);
    armLimitSwitch = new DigitalInput(armLimitSwitchDio);

    // wrist
    wristSpeedController = new PWMVictorSPX(wristPwm); 
    wristEncoder = new Encoder(wristEncoderADio, wristEncoderBDio);
    wristLimitSwitch = new DigitalInput(wristLimitSwitchDio);

    // grabber
    grabberSpeedController = new PWMVictorSPX(grabberPwm);
    grabberEncoder = new Encoder(grabberEncoderADio, grabberEncoderBDio);
    grabberLimitSwitch = new DigitalInput(grabberLimitSwitchDio);

    // cargo handler
    cargoRollerSpeedController = new PWMVictorSPX(cargoRollerPwm);
    cargoColorSensor = new MRColorSensor();

    // hatch handler
    hatchLimitSwitch = new DigitalInput(hatchLimitSwitchDio);
  }
}
