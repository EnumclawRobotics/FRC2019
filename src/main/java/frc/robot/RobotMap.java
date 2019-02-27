package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
//import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.cameraserver.*;
import edu.wpi.cscore.UsbCamera;

import frc.robot.Components.*;
import common.i2cSensors.*;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.*;
import com.revrobotics.CANEncoder;
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
    final int driveLeftGoldCan = 2;                 // 40amp / VictorSPX / CIM / ToughBox
    final int driveRightMaroonCan = 3;              // 40amp / VictorSPX / CIM / ToughBox
    final int driveRightGoldCan = 4;                // 40amp / VictorSPX / CIM / ToughBox 

    final int armCan = 1;                           // 40amp / SparkMax / NEO / CimSport 100:1
    final int wristCan = 6;                         // 40amp / SparkMax / NEO / CimSport 100:1
    final int liftMaroonCan = 7;                    // 40amp / SparkMax / NEO / CimSport 12:1
    final int liftGoldCan = 6;                      // 40amp / SparkMax / NEO / CimSport 12:1

    final int grabberCan = 1;                       // 30amp / TalonSRX / 775 / CimSport 12:1
    final int rollerMaroonCan = 2;                  // 30amp / TalonSRX / 775 / CimSport 4:1
    final int rollerGoldCan = 3;                    // 30amp / TalonSRX / 775 / CimSport 4:1

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
    public final static double armLength = 19d; 
    public final static double armPivotHeight = 37.25d;
    public final static double armFeedForwardFactor = 0d;                     // hold at horizontal power. find by testing
    public final static double armKpFactor = .1d;                             // PID kP correction factor 
    public final static double armEncoderClicksPerDegree = 4200d/360d;        // NEO gearbox output shaft include gear reduction  
    public final static double armStartDegree = 5d;                           // starting angle for arm  

    // wrist
    public final static double wristLength = 7.75d; 
    public final static double wristEncoderClicksPerDegree = 4200d/360d;      // NEO gearbox output shaft include gear reduction
    public final static double wristFeedForwardFactor = 0d;                   // hold at horizontal power. find by testing
    public final static double wristKpFactor = .5d;                           // PID kP correction factor
    public final static double wristHeldAngle = 10d;                          // angle to fold back the grabber for protection
    public final static double wristStartDegree = 5d;                         // starting angle for wrist

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
    public double liftGravity = .25d;           // station keeping lift power

    // === REFERENCES ======================

    // SpeedControllers
    public SpeedController driveLeftMaroonSpeedController;      // set coast
    public SpeedController driveLeftGoldSpeedController;        // set coast
    public SpeedController driveRightMaroonSpeedController;     // set coast
    public SpeedController driveRightGoldSpeedController;       // set coast

    public SpeedController armSpeedController;                  // set brake 
    public SpeedController wristSpeedController;                // set brake
    public SpeedController grabberSpeedController;              // set brake
    public SpeedController rollerMaroonSpeedController;         // set brake
    public SpeedController rollerGoldSpeedController;           // set brake
    
    public SpeedController liftMaroonSpeedController;           // set brake
    public SpeedController liftGoldSpeedController;             // set brake
    public SpeedController liftMoverSpeedController;            // set brake

    // sensors
    //    public ADXRS450_Gyro driveGyro;                           // ROBORIO mounted wrong for this

    public GenericEncoder armEncoder;
    public DigitalInput armLimitSwitch;

    public GenericEncoder wristEncoder;
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
        driveLeftMaroonSpeedController = new CANSparkMax(driveLeftMaroonCan, MotorType.kBrushless);
        driveLeftGoldSpeedController = new CANSparkMax(driveLeftGoldCan, MotorType.kBrushless);
        driveRightMaroonSpeedController = new CANSparkMax(driveRightMaroonCan, MotorType.kBrushless);
        driveRightGoldSpeedController = new CANSparkMax(driveRightGoldCan, MotorType.kBrushless);

        // driveGyro = new ADXRS450_Gyro();

        // arm
        armSpeedController = new CANSparkMax(armCan, MotorType.kBrushless);
        armEncoder = new GenericEncoder(new CANEncoder((CANSparkMax)armSpeedController));
        armLimitSwitch = new DigitalInput(armLimitSwitchDio);

        // wrist
        wristSpeedController =  new CANSparkMax(wristCan, MotorType.kBrushless);
        wristEncoder = new GenericEncoder(new CANEncoder((CANSparkMax)wristSpeedController));
 //       wristLimitSwitch = new DigitalInput(wristLimitSwitchDio);

        // grabber
        grabberSpeedController = (SpeedController)new TalonSRX(grabberCan);
//        grabberEncoder = new GenericEncoder(new Encoder(grabberEncoderADio, grabberEncoderBDio));
//        grabberLimitSwitch = new DigitalInput(grabberLimitSwitchDio);

        // cargo handler
        rollerMaroonSpeedController = (SpeedController)new VictorSPX(rollerMaroonCan);
        rollerGoldSpeedController = (SpeedController)new VictorSPX(rollerGoldCan);
//        cargoColorSensor = new MRColorSensor();

        // hatch handler
//        hatchLimitSwitch = new DigitalInput(hatchLimitSwitchDio);

        // lift
        liftMaroonSpeedController = new CANSparkMax(liftMaroonCan, MotorType.kBrushless);
        liftGoldSpeedController = new CANSparkMax(liftMaroonCan, MotorType.kBrushless);
        liftMoverSpeedController = new VictorSP(liftMoverPwm);

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
