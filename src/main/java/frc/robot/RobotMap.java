package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.cameraserver.*;

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
    final int driveLeftFrontCan = 1;               // 40amp / VictorSPX / CIM / ToughBox 
    final int driveLeftBackCan = 2;                 // 40amp / VictorSPX / CIM / ToughBox
    final int driveRightFrontCan = 3;              // 40amp / VictorSPX / CIM / ToughBox
    final int driveRightBackCan = 4;                // 40amp / VictorSPX / CIM / ToughBox 

    final int armLeftCan = 1;                     // 40amp / SparkMax / NEO / CimSport 100:1
    final int armRightCan = 6;                      // 40amp / SparkMax / NEO / CimSport 100:1
    final int wristCan = 8;                         // 30amp / SparkMax / NEO / CimSport 64:1

    final int grabberCan = 5;                      // 30amp / VictorSPX / 775 / PG188 188:1
    final int rollerFrontCan = 6;                  // 30amp / VictorSPX / 775 / CimSport 4:1
    final int rollerBackCan = 7;                   // 30amp / VictorSPX / 775 / CimSport 4:1

    final int liftFrontCan = 7;                    // 40amp / SparkMax / NEO / CimSport 12:1
    final int liftBackCan = 9;                     // 40amp / SparkMax / NEO / CimSport 12:1

    // DIO Ports
    final int grabberEncoderADio = 0;               // PG188 encoder
    final int grabberEncoderBDio = 1;               // PG188 encoder

    final int armLimitSwitchDio = 5;                // Rev Magnetic limit switch
    final int wristLimitSwitchDio = 6;              // Rev Magnetic limit switch

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

    public final static double driveSpeedLimiter = .50d;                 // used to limit output when not afterburning
    public final static double driveRotationLimiter = .40d;                 // used to limit output when not afterburning

    // safety
    public final static double safetyExpiration = .25d;

    // arm geometries (in inches)
    public final static double armPivotHeight = 37.25d;                         // pivot above ground
    public final static double armLength = 19d;                                 // pivot to pivot
    public final static double armStowedAngle = 5d;                             // starting angle for arm  
    public final static double armEncoderClicksPerDegree = (42d*100d)/360d;     // NEO gearbox output shaft include gear reduction  
    public final static double armPidLocality = armEncoderClicksPerDegree * 5d;   // area around setpoint to use PID with 
    public final static double armPidKu = .18d;                                 // Proportional oscillation 
    public final static double armPidKp = .0171d;                               // PID kP correction factor 
    public final static double armPidKi = 0d;                                   // PID kI correction factor 
    public final static double armPidKd = 0d;                                   // PID kD correction factor 
    public final static double armFeedForwardFactor = .12d;                     // hold at horizontal power. find by testing
    public final static double armRampFactor = .04d;                            // ramp change power limit per cycle (60hz) 

    // wrist
    public final static double wristLength = 7.75d; 
    public final static double wristStowedAngle = 5d;                           // angle to fold back the grabber for protection
    public final static double wristEncoderClicksPerDegree = (42d*64d)/360d;    // NEO gearbox output shaft include gear reduction
    public final static double wristPidLocality = wristEncoderClicksPerDegree * 5d;   // area around setpoint to use PID with 
    public final static double wristPidKu = .12d;                                   // Proportional oscillation
    public final static double wristPidKp = .0268d;                             // PID kP correction factor
    public final static double wristPidKi = 0d;                                 // PID kI correction factor
    public final static double wristPidKd = 0d;                                 // PID kD correction factor
    public final static double wristFeedForwardFactor = .12d;                   // hold at horizontal power. find by testing
    public final static double wristRampFactor = .04d;                          // ramp change power limit per cycle (60hz)

    // grabber
    public final static double grabberLength = 10d;
    public final static double grabberEncoderClicksPerDegree = 1316d/360d;      // PG188 output shaft include gear reduction
    public final static double grabberLocality = grabberEncoderClicksPerDegree * 5d;   // area around setpoint to use PID with
    public final static double grabberPidKp = .0547d;                           // PID kP correction factor
    public final static double grabberPidKi = 0d;                               // PID kI correction factor
    public final static double grabberPidKd = 0d;                               // PID kD correction factor
    public final static double grabberHatchOpen = 30d;                          // degrees to open
    public final static double grabberCargoOpen = 60d;                          // degrees to open
    public final static double grabberIntake = .3d;                             // speed of intake
    public final static double grabberExpell = -.3d;                            // speed of expelling
    public final static double grabberRampFactor = .04d;                        // ramp change power limit per cycle (60hz)

    // lift 
    public final static double liftHeight = 20d;                                // inches to raise bot
    public final static double liftMoverPower = 1d;                             // power to roll forward
    public final static double liftEncoderClicksPerDegree = 42d*(12d * 24d/18d)/360d;      // 12:1 output shaft with 24/18 sprocket reduction
    public final static double liftLocality = liftEncoderClicksPerDegree * 60d;  // area around setpoint to use PID with
    public final static double liftPidKp = .00892d;                             // PID kP correction factor
    public final static double liftPidKi = .0d;                                 // PID kI correction factor
    public final static double liftPidKd = .0d;                                 // PID kD correction factor

    public final static double liftStow = -.05d;                                // retraction power 
    public final static double liftTouchdown = .05d;                            // touching down
    public final static double liftExtend = .40d;                               // extend lift power limit

    public final static double liftRamp = .1d;                                  // acceleration

    // camera
    public final static double cameraElevation = 35d;           // Camera height from floor;
    public final static double cameraAngle = 30d;               // Angle between vertical and camera facing in degrees;
    public final static double cameraFovX = 60d;                // Field of view in degrees (FOV) Pixy2
    public final static double cameraFovY = 40d;                // Field of view in degrees (FOV) Pixy2
    public final static double cameraMaxX = 80d;                // Max X resolution Pixy2
    public final static double cameraMaxY = 60d;                // Max Y resolution Pixy2

    
    // === REFERENCES ======================

    // SpeedControllers
    public CANVictorSPX driveLeftFrontSpeedController;          // set coast
    public CANVictorSPX driveLeftBackSpeedController;           // set coast
    public CANVictorSPX driveRightFrontSpeedController;         // set coast
    public CANVictorSPX driveRightBackSpeedController;          // set coast

    public CANSparkMax armLeftSpeedController;                  // set brake 
    public CANSparkMax armRightSpeedController;                 // set brake 
    public CANSparkMax wristSpeedController;                    // set brake

    public CANVictorSPX grabberSpeedController;                 // set brake
    public CANVictorSPX rollerFrontSpeedController;             // set brake
    public CANVictorSPX rollerBackSpeedController;              // set brake
    
    public CANSparkMax liftFrontSpeedController;                // set brake
    public CANSparkMax liftBackSpeedController;                 // set brake
    public VictorSP liftMoverSpeedController;                   // set brake

    // sensors
    //    public ADXRS450_Gyro driveGyro;                           // ROBORIO is mounted wrong for this

    public CANEncoder2 armEncoder;
    public DigitalInput armLimitSwitch;

    public CANEncoder2 wristEncoder;
    public DigitalInput wristLimitSwitch;

    public Encoder grabberEncoder;
    //    public DigitalInput grabberLimitSwitch;
    public DigitalInput hatchLimitSwitch;               // TODO: Use two limit switches on physical bot just linked together
    public MRColorSensor cargoColorSensor;

    public CANEncoder2 liftFrontEncoder;
    public CANEncoder2 liftBackEncoder;

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
        driveLeftFrontSpeedController = new CANVictorSPX(driveLeftFrontCan);
        driveLeftBackSpeedController = new CANVictorSPX(driveLeftBackCan);
        driveRightFrontSpeedController = new CANVictorSPX(driveRightFrontCan);
        driveRightBackSpeedController = new CANVictorSPX(driveRightBackCan);

        // driveGyro = new ADXRS450_Gyro();

        // arm
        armLeftSpeedController = new CANSparkMax(armLeftCan, MotorType.kBrushless);
        armRightSpeedController = new CANSparkMax(armRightCan, MotorType.kBrushless);
        armEncoder = new CANEncoder2(armRightSpeedController);
        armLimitSwitch = new DigitalInput(armLimitSwitchDio);

        // wrist
        wristSpeedController =  new CANSparkMax(wristCan, MotorType.kBrushless);
        wristEncoder = new CANEncoder2(wristSpeedController);
        wristLimitSwitch = new DigitalInput(wristLimitSwitchDio);

        // grabber
        grabberSpeedController = new CANVictorSPX(grabberCan);
        grabberEncoder = new Encoder(grabberEncoderADio, grabberEncoderBDio);
//        grabberLimitSwitch = new DigitalInput(grabberLimitSwitchDio);

        // cargo handler
        rollerFrontSpeedController = new CANVictorSPX(rollerFrontCan);
        rollerBackSpeedController = new CANVictorSPX(rollerBackCan);
//        cargoColorSensor = new MRColorSensor();

        // hatch handler
//        hatchLimitSwitch = new DigitalInput(hatchLimitSwitchDio);

        // lift
        liftFrontSpeedController = new CANSparkMax(liftFrontCan, MotorType.kBrushless);
        liftBackSpeedController = new CANSparkMax(liftFrontCan, MotorType.kBrushless);
        liftMoverSpeedController = new VictorSP(liftMoverPwm);
        liftFrontEncoder = new CANEncoder2(liftFrontSpeedController);
        liftBackEncoder = new CANEncoder2(liftBackSpeedController);

        // cameras
        cameraNormal = CameraServer.getInstance().startAutomaticCapture(cameraNormalUsb);
        cameraInverted = CameraServer.getInstance().startAutomaticCapture(cameraInvertedUsb); 
        // cameraNormal = new UsbCamera("USB Camera 0", cameraNormalUsb);
        // cameraInverted = new UsbCamera("USB Camera 1", cameraInvertedUsb);

        // Pixy2
        pixy2Normal = Pixy2.createInstance(new I2CLink());
        pixy2Normal.init(2);
        // pixy2Inverted = Pixy2.createInstance(new I2CLink(2));
        // pixy2Inverted.init();
    }
}
