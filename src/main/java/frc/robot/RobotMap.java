
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
    wristSpeedController = new PWMVictorSPX(wristPwmPort); 
    wristEncoder = new Encoder(wristEncoderADioPort, wristEncoderBDioPort);
    wristLimitSwitch = new DigitalInput(wristLimitSwitchDioPort);

    // grabber
    grabberSpeedController = new PWMVictorSPX(grabberPwmPort);
    grabberEncoder = new Encoder(grabberEncoderADioPort, grabberEncoderBDioPort);
    grabberLimitSwitch = new DigitalInput(grabberLimitSwitchDioPort);

    // cargo handler
    cargoRollerSpeedController = new PWMVictorSPX(cargoRollerPwmPort);
    cargoColorSensor = new MRColorSensor();

    // hatch handler
    hatchLimitSwitch = new DigitalInput(hatchLimitSwitchDioPort);
  }
}
