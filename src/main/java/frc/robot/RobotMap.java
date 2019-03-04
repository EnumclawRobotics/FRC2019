package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.VictorSP;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.cameraserver.*;
import edu.wpi.cscore.UsbCamera;

import frc.robot.Components.*;
import common.i2cSensors.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import common.pixy2Api.*;
import common.pixy2Api.Pixy2;

/**
 * Robot's Hardware Specifics
 * @see FieldMap
 */
public class RobotMap {

    // **** PRACTICE BOT SETTINGS ****

    // === ADDRESSES/PLUGINS ============================

    // PWM Ports
    final int liftMoverPwm = 0;                     // 30amp / VictorSP / Neverest 40 

    // CAN Device IDs
    final int driveLeftMaroonCan = 1;               // 40amp / VictorSPX / CIM / ToughBox 
    final int driveLeftBlackCan = 2;                 // 40amp / VictorSPX / CIM / ToughBox
    final int driveRightMaroonCan = 3;              // 40amp / VictorSPX / CIM / ToughBox
    final int driveRightBlackCan = 4;                // 40amp / VictorSPX / CIM / ToughBox 

    final int armCan = 6;                           // 40amp / SparkMax / NEO / CimSport 100:1
    final int wristCan = 1;                         // 40amp / SparkMax / NEO / CimSport 100:1
    final int liftMaroonCan = 7;                    // 40amp / SparkMax / NEO / CimSport 12:1
    final int liftBlackCan = 6;                      // 40amp / SparkMax / NEO / CimSport 12:1

    final int grabberCan = 1;                       // 30amp / TalonSRX / 775 / CimSport 12:1
    final int rollerMaroonCan = 2;                  // 30amp / TalonSRX / 775 / CimSport 4:1
    final int rollerBlackCan = 3;                   // 30amp / TalonSRX / 775 / CimSport 4:1

    // DIO Ports
    final int armLimitSwitchDio = 0;                // Rev Magnetic limit switch
    final int wristLimitSwitchDio = 1;              // Rev Magnetic limit switch

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
    public final static double armPivotHeight = 37.25d;                     // pivot above ground
    public final static double armLength = 19d;                             // pivot to pivot
    public final static double armRampFactor = .04;                         // ramp change power limit per cycle (50hz) 
    public final static double armFeedForwardFactor = 0d;                   // hold at horizontal power. find by testing
    public final static double armKpFactor = 0;                             // PID kP correction factor 
    public final static double armKiFactor = 0;                             // PID kI correction factor 
    public final static double armKdFactor = 0;                             // PID kD correction factor 
    public final static double armEncoderClicksPerDegree = 4200d/360d;      // NEO gearbox output shaft include gear reduction  
    public final static double armStowedAngle = 5d;                         // starting angle for arm  

    // wrist
    public final static double wristLength = 7.75d; 
    public final static double wristRampFactor = .04d;                      // ramp change power limit per cycle (50hz)
    public final static double wristFeedForwardFactor = 0d;                 // hold at horizontal power. find by testing
    public final static double wristKpFactor = 0;                           // PID kP correction factor
    public final static double wristKiFactor = 0;                           // PID kI correction factor
    public final static double wristKdFactor = 0;                           // PID kD correction factor
    public final static double wristEncoderClicksPerDegree = 4200d/360d;    // NEO gearbox output shaft include gear reduction
    public final static double wristStowedAngle = 5d;                       // angle to fold back the grabber for protection

    // grabber
    public final static double grabberLength = 10d;
//    public final static double grabberEncoderClicksPerDegree = 1316d/360d;    // PG188 output shaft include gear reduction

    // camera
    public double cameraElevation = 35d;           // Camera height from floor;
    public double cameraAngle = 30d;               // Angle between vertical and camera facing in degrees;
    public double cameraFovX = 60d;                // Field of view in degrees (FOV) Pixy2
    public double cameraFovY = 40d;                // Field of view in degrees (FOV) Pixy2
    public double cameraMaxX = 80d;                // Max X resolution Pixy2
    public double cameraMaxY = 60d;                // Max Y resolution Pixy2

    // lift 
    public double liftHeight = 20d;                 // inches to raise bot
    public double liftMoverSpeed = 1d;              // power to roll forward
    public double liftGravity = .25d;               // station keeping lift power

    // === REFERENCES ======================

    // SpeedControllers
    public CANVictorSPX driveLeftMaroonSpeedController;            // set coast
    public CANVictorSPX driveLeftBlackSpeedController;              // set coast
    public CANVictorSPX driveRightMaroonSpeedController;           // set coast
    public CANVictorSPX driveRightBlackSpeedController;             // set coast

    public CANSparkMax armSpeedController;                      // set brake 
    public CANSparkMax wristSpeedController;                    // set brake

    public CANTalonSRX grabberSpeedController;                     // set brake
    public CANTalonSRX rollerMaroonSpeedController;                // set brake
    public CANTalonSRX rollerBlackSpeedController;                  // set brake
    
    public CANSparkMax liftMaroonSpeedController;               // set brake
    public CANSparkMax liftBlackSpeedController;                 // set brake
    public VictorSP liftMoverSpeedController;                   // set brake

    // sensors
    //    public ADXRS450_Gyro driveGyro;                           // ROBORIO mounted wrong for this

    public CANEncoder2 armEncoder;
    public DigitalInput armLimitSwitch;

    public CANEncoder2 wristEncoder;
    public DigitalInput wristLimitSwitch;
//    public GenericEncoder grabberEncoder;
//    public DigitalInput grabberLimitSwitch;

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
        driveLeftMaroonSpeedController = new CANVictorSPX(driveLeftMaroonCan);
        driveLeftBlackSpeedController = new CANVictorSPX(driveLeftBlackCan);
        driveRightMaroonSpeedController = new CANVictorSPX(driveRightMaroonCan);
        driveRightBlackSpeedController = new CANVictorSPX(driveRightBlackCan);

        // driveGyro = new ADXRS450_Gyro();

        // arm
        armSpeedController = new CANSparkMax(armCan, MotorType.kBrushless);
        armEncoder = new CANEncoder2(armSpeedController);
        armLimitSwitch = new DigitalInput(armLimitSwitchDio);

        // wrist
        wristSpeedController =  new CANSparkMax(wristCan, MotorType.kBrushless);
        wristEncoder = new CANEncoder2(wristSpeedController);
        wristLimitSwitch = new DigitalInput(wristLimitSwitchDio);

        // grabber
        grabberSpeedController = new CANTalonSRX(grabberCan);
//        grabberEncoder = new GenericEncoder(new Encoder(grabberEncoderADio, grabberEncoderBDio));
//        grabberLimitSwitch = new DigitalInput(grabberLimitSwitchDio);

        // cargo handler
        rollerMaroonSpeedController = new CANTalonSRX(rollerMaroonCan);
        rollerBlackSpeedController = new CANTalonSRX(rollerBlackCan);
//        cargoColorSensor = new MRColorSensor();

        // hatch handler
//        hatchLimitSwitch = new DigitalInput(hatchLimitSwitchDio);

        // lift
//        liftMaroonSpeedController = new CANSparkMax(liftMaroonCan, MotorType.kBrushless);
//        liftBlackSpeedController = new CANSparkMax(liftMaroonCan, MotorType.kBrushless);
//        liftMoverSpeedController = new VictorSP(liftMoverPwm);

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
