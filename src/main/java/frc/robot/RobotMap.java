package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.cameraserver.*;
import edu.wpi.cscore.UsbCamera;

import frc.robot.Components.*;
import common.i2cSensors.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import common.pixy2Api.*;
import common.pixy2Api.Pixy2;

// Hardware specifics 
// NOTE: Components will use super classes to hide the hardware specifics.
//       One exception is CAN versus PWM speedcontrollers
// TODO: Fill in Javadocs for all calls
public class RobotMap {

    // === ADDRESSES/PLUGINS ============================

    // PWM Ports
    final int driveLeftPwm = 0;            // 40amp x2 pwm group - use Y split to signal both front and back motor controller
    final int driveRightPwm = 1;           // 40amp x2 pwm group - use Y split to signal both front and back motor controller

    final int wristPwm = 3;                // 30amp
    final int grabberPwm = 4;              // 30amp
    final int cargoRollerPwm = 5;          // 30amp depending on motor size can Y split the power to both motors 

    final int liftFrontPwm = 7;             // 30amp
    final int liftBackPwm = 8;              // 30amp
    final int liftMoverPwm = 9;             // 30amp 

    // CAN Device IDs
    final int armLeftCan = 2;               // 40AMP SparkMax-Neo requires CAN otherwise buggy
    final int armRightCan = 3;              // 40AMP SparkMax-Neo requires CAN otherwise buggy

    // DIO Ports
    final int armLimitSwitchDio = 0;

    final int wristLimitSwitchDio = 1;
    final int wristEncoderADio = 2;
    final int wristEncoderBDio = 3;

    final int grabberLimitSwitchDio = 4;
    final int grabberEncoderADio = 5;
    final int grabberEncoderBDio = 6;

    final int hatchLimitSwitchDio = 7;      // tie the two limit switches together so that either pressed causes trigger 

    // I2C Addresses
    final int pixy2NormalI2C = 0x53;
    final int pixy2InvertedI2C = 0x63; 

    // USB Ports RoboRio
    final int cameraNormalUsb = 0;
    final int cameraInvertedUsb = 1;

    // USB Ports (driver station)
    final int driveXboxControllerUsb = 0;
    final int armXboxControllerUsb = 1;
    final int armButtonsUsb = 2;

    // TODO: Add buttons to control height
 

    // === TUNING CONSTANTS =====================================

    // safety
    public final static double safetyExpiration = .25d;

    // arm geometries (in inches)
    public final static double armLength = 38d; 
    public final static double armPivotHeight = 45d;
    public final static double armFeedForwardFactor = .5d;                    // hold at horizontal power. find by testing
    public final static double armKpFactor = .1d;                             // PID kP correction factor 
    public final static double armEncoderClicksPerDegree = 4200d/360d;        // NEO gearbox output shaft include gear reduction  
    public final static double armStartDegree = 5d;                           // starting angle for arm  

    // wrist
    public final static double wristLength = 7.75d; 
    public final static double wristEncoderClicksPerDegree = 1316d/360d;      // PG188 output shaft include gear reduction
    public final static double wristFeedForwardFactor = .5d;                  // hold at horizontal power. find by testing
    public final static double wristKpFactor = .5d;                           // PID kP correction factor
    public final static double wristHeldAngle = 10d;                          // angle to fold back the grabber for protection
    public final static double wristStartDegree = 5d;                         // starting angle for wrist

    // grabber
    public final static double grabberLength = 10d;
    public final static double grabberEncoderClicksPerDegree = 1316d/360d;    // PG188 output shaft include gear reduction

    // camera
    public double cameraElevation = 46d;           // Camera height from floor;
    public double cameraAngle = 30d;               // Angle below camera and vertical facing in degrees;
    public double cameraFovX = 60d;                // Field of view in degrees (FOV)
    public double cameraFovY = 40d;                // Field of view in degrees (FOV)
    public double cameraMaxX = 80d;                // Max X resolution
    public double cameraMaxY = 60d;                // Max Y resolution

    // lift 
    public double liftHeight = 20d;                 // inches to raise bot
    public double liftMoverSpeed = 1d;              // power to roll forward
    public double liftFeedForward = .25d;           // station keeping lift power

    // === REFERENCES ======================

    // SpeedControllers
    public SpeedController driveLeftSpeedController;
    public SpeedController driveRightSpeedController;
    public SpeedController armLeftSpeedController; 
    public SpeedController armRightSpeedController; 
    public SpeedController wristSpeedController;
    public SpeedController grabberSpeedController;
    public SpeedController cargoRollerSpeedController;        // TODO: Set brake mode on roller controller
    public SpeedController liftFrontSpeedController;
    public SpeedController liftBackSpeedController;
    public SpeedController liftMoverSpeedController;

    // sensors
    public ADXRS450_Gyro driveGyro;

    public GenericEncoder armEncoder;
    public DigitalInput armLimitSwitch;

    public GenericEncoder wristEncoder;
    public DigitalInput wristLimitSwitch;

    public GenericEncoder grabberEncoder;
    public DigitalInput grabberLimitSwitch;

    public DigitalInput hatchLimitSwitch;                     // TODO: Use two limit switches on physical bot just linked together

    public MRColorSensor cargoColorSensor;

    public Pixy2 pixy2Normal;
    public Pixy2 pixy2Inverted;

    public UsbCamera cameraNormal;
    public UsbCamera cameraInverted;

    // usb driver station
    public XboxController driveXboxController;
    public XboxController armXboxController;  
    public Joystick armButtons;  

    // setup subsystems
    public RobotMap() {
        // operator
        driveXboxController = new XboxController(driveXboxControllerUsb);
        armXboxController = new XboxController(armXboxControllerUsb);
        armButtons = new Joystick(armButtonsUsb);

        // drive 
        driveLeftSpeedController = new Spark(driveLeftPwm);
        driveRightSpeedController = new Spark(driveRightPwm);
        driveGyro = new ADXRS450_Gyro();

        // arm
        armLeftSpeedController = new CANSparkMax(armLeftCan, MotorType.kBrushless);
        armRightSpeedController = new CANSparkMax(armRightCan, MotorType.kBrushless);
        armEncoder = new GenericEncoder(new CANEncoder((CANSparkMax)armLeftSpeedController));
        armLimitSwitch = new DigitalInput(armLimitSwitchDio);

        // wrist
        wristSpeedController = new PWMVictorSPX(wristPwm); 
        wristEncoder = new GenericEncoder(new Encoder(wristEncoderADio, wristEncoderBDio));
        wristLimitSwitch = new DigitalInput(wristLimitSwitchDio);

        // grabber
        grabberSpeedController = new PWMVictorSPX(grabberPwm);
        grabberEncoder = new GenericEncoder(new Encoder(grabberEncoderADio, grabberEncoderBDio));
        grabberLimitSwitch = new DigitalInput(grabberLimitSwitchDio);

        // cargo handler
        cargoRollerSpeedController = new PWMVictorSPX(cargoRollerPwm);
        cargoColorSensor = new MRColorSensor();

        // hatch handler
        hatchLimitSwitch = new DigitalInput(hatchLimitSwitchDio);

        // lift
        liftFrontSpeedController = new PWMVictorSPX(liftFrontPwm);
        liftBackSpeedController = new PWMVictorSPX(liftBackPwm);
        liftMoverSpeedController = new PWMVictorSPX(liftMoverPwm);

        // cameras
        cameraNormal = new UsbCamera("USB Camera 0", cameraNormalUsb);
        cameraInverted = new UsbCamera("USB Camera 1", cameraInvertedUsb);

        // Pixy2
        pixy2Normal = Pixy2.createInstance(new I2CLink());
        pixy2Normal.init(2);
        // pixy2Inverted = Pixy2.createInstance(new I2CLink(2));
        // pixy2Inverted.init();
    }
}
