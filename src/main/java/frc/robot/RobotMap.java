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

// import common.pixy2Api.*;
// import common.pixy2Api.Pixy2;

/**
 * Robot's Hardware Specifics
 * @see FieldMap
 */
public class RobotMap {

    // **** PRACTICE BOT SETTINGS ****

    // === ADDRESSES/PLUGINS ============================

    // PWM Ports
    private final int liftMoverPwm = 0;                     // 30amp / VictorSP / Neverest 40 

    // CAN Device IDs
    private final int driveLeftFrontCan = 1;                // 40amp / VictorSPX / CIM / ToughBox 
    private final int driveLeftBackCan = 2;                 // 40amp / VictorSPX / CIM / ToughBox
    private final int driveRightFrontCan = 3;               // 40amp / VictorSPX / CIM / ToughBox
    private final int driveRightBackCan = 4;                // 40amp / VictorSPX / CIM / ToughBox 

    private final int armLeftCan = 1;                       // 40amp / SparkMax / NEO / CimSport 100:1
    private final int armRightCan = 6;                      // 40amp / SparkMax / NEO / CimSport 100:1
    private final int wristCan = 8;                         // 30amp / SparkMax / NEO / CimSport 64:1

    private final int grabberCan = 5;                       // 30amp / VictorSPX / 775 / PG188 188:1
    private final int rollerFrontCan = 11;                  // 30amp / VictorSPX / 775 / CimSport 4:1
    private final int rollerBackCan = 12;                   // 30amp / VictorSPX / 775 / CimSport 4:1

    private final int liftFrontCan = 7;                     // 40amp / SparkMax / NEO / CimSport 20:1
    private final int liftBackCan = 9;                      // 40amp / SparkMax / NEO / CimSport 20:1
    // private final int liftFrontCan = 7;                     // 40amp / SparkMax / NEO / CimSport 12:1
    // private final int liftBackCan = 9;                      // 40amp / SparkMax / NEO / CimSport 12:1

    // DIO Ports
    private final int grabberEncoderADio = 0;                       // PG188 encoder
    private final int grabberEncoderBDio = 1;                       // PG188 encoder

    // private final int armLimitSwitchDio = 5;                // Rev Magnetic limit switch
    // private final int wristLimitSwitchDio = 6;              // Rev Magnetic limit switch

    // I2C Addresses
    // private final int pixy2NormalI2C = 0x53;
    // private final int pixy2InvertedI2C = 0x63; 

    // // USB Ports RoboRio
    private final int cameraNormalUsb = 0;
    private final int cameraInvertedUsb = 1;

    // USB Ports (driver station)
    private final int driveXboxControllerUsb = 0;
    private final int armXboxControllerUsb = 1;
    private final int armButtonsUsb = 2;

    // TODO: Add buttons to control height
 

    // === CONSTANTS - TUNING etc =====================================

    // drive
    public final static double driveSpeedLimiter = .50d;                        // used to limit output when not afterburning
    public final static double driveRotationLimiter = .40d;                     // used to limit output

    // safety
    public final static double safetyExpiration = .25d;

    // arm geometries (in inches)
    // arm geometries
    public final static double armPivotHeight = 37.25d;                         // pivot above ground
    public final static double armLength = 19d;                                 // pivot to pivot

    public final static double armAngleStowed = 5d;                             // starting angle for arm  
    public final static double armAngleRocketHatch1 = armAngleStowed;
    public final static double armAngleRocketHatch2 = 120.879;
    public final static double armAngleShipHatch = armAngleRocketHatch1;
    public final static double armAngleStationHatch = armAngleRocketHatch1;

    public final static double armAngleRocketCargo1 = 59.126;
    public final static double armAngleRocketCargo2 = 163.848;
    public final static double armAngleDepotCargo = 42.330;             // requires wrist to be straight with arm instead of held level
    public final static double armAngleStationCargo = 111.213;
    public final static double armAngleShipCargo = 94.528;

    public final static double armEncoderConversionFactor = 42d;                // clicks per rpm for NEO
    public final static double armGearboxConversionFactor = 100d;               // 100:1 Andymark gearbox reduction
    public final static double armEncoderClicksPerDegree = (armEncoderConversionFactor * armGearboxConversionFactor)/360d;  
    public final static double armPidLocality = armEncoderClicksPerDegree * 5d;   // area around setpoint to use PID with 
    public final static double armPowerLimit = .26d;                            // power limit
    public final static double armPidKp = armPowerLimit / armPidLocality;       // PID kP correction factor  
    public final static double armPidKi = .0015d;                               // PID kI correction factor 
    public final static double armPidKd = 0d;                                   // PID kD correction factor
    public final static double armFeedForwardFactor = .13d;                     // power for feed forward amount
    public final static double armRampFactor = .1d;                            // ramp change power limit per cycle (60hz) 

    // wrist
    public final static double wristAngleStowed = 5d;                             // starting angle for arm  
    public final static double wristAngleRocketHatch1 = 90 + armAngleRocketHatch1;
    public final static double wristAngleRocketHatch2 = 90 + armAngleRocketHatch2;
    public final static double wristAngleShipHatch = wristAngleRocketHatch1;
    public final static double wristAngleStationHatch = wristAngleRocketHatch1;

    public final static double wristAngleRocketCargo1 = 90 + armAngleRocketCargo1;
    public final static double wristAngleRocketCargo2 = 90 + armAngleRocketCargo2;
    public final static double wristAngleDepotCargo = 180;         // requires wrist to be straight with arm instead of held level
    public final static double wristAngleStationCargo = 90 + armAngleStationCargo;
    public final static double wristAngleShipCargo = 90 + armAngleShipCargo;

    public final static double wristLength = 7.75d; 
//    public final static double wristStowedAngle = 5d;                               // angle to fold back the grabber for protection
    public final static double wristStowedAngle = 15d;                               // angle to fold back the grabber for protection

    public final static double wristEncoderConversionFactor = 42d;                  // clicks per rpm for NEO
    public final static double wristGearboxConversionFactor = 64d;                  // 64:1 Andymark gearbox reduction
    public final static double wristEncoderClicksPerDegree = (armEncoderConversionFactor * armGearboxConversionFactor)/360d;  
    public final static double wristPidLocality = wristEncoderClicksPerDegree * 3d; // area around setpoint to use PID with 
//    public final static double wristPowerLimit = .25d;                            // power limit
    public final static double wristPowerLimit = .35d;                              // power limit
//    public final static double wristPidKu = .010d;                                // steady oscillation
    public final static double wristPidKu = .010d;                                  // steady oscillation
    public final static double wristPidKp =  wristPowerLimit / wristPidLocality;    // PID kP correction factor  
//    public final static double wristPidKi = 0d;                                     // PID kI correction factor
    public final static double wristPidKi = 0d;                                     // PID kI correction factor
    public final static double wristPidKd = 0d;                                     // PID kD correction factor
//    public final static double wristFeedForwardFactor = .1d;                       // hold at horizontal power. find by testing
    public final static double wristFeedForwardFactor = .1d;                       // hold at horizontal power. find by testing
//    public final static double wristRampFactor = .04d;                              // ramp change power limit per cycle (60hz)
    public final static double wristRampFactor = .1d;                              // ramp change power limit per cycle (60hz)


// TODO: Assume that roboinit and autoinit and teleopinit can happen at different times and handle it
// TODO: Mechanically add magnetic limit switch to arm and apply a zero function 
// TODO: Allow baseClicks to move lower if encoder says they go there. For both Arm and Wrist
// TODO: Bumper for wrist. New electronics board and mounting. 

    // grabber
    public final static double grabberLength = 10d;
    public final static double grabberEncoderClicksPerDegree = 1316d/360d;      // PG188 output shaft include gear reduction
    public final static double grabberLocality = grabberEncoderClicksPerDegree * 3d;   // area around setpoint to use PID with
    public final static double grabberPowerLimit = .620d;                         // power limit
    public final static double grabberPidKp = grabberPowerLimit / grabberLocality;  // PID kP correction factor
    public final static double grabberPidKi = 0d;                               // PID kI correction factor
    public final static double grabberPidKd = 0d;                               // PID kD correction factor
    // public final static double grabberHatchOpen = 30d;                          // degrees to open
    public final static double grabberCargoOpen = 60d;                          // degrees to open
    public final static double grabberRampFactor = .1d;                         // ramp change power limit per cycle (60hz)
    public final static double rollerIntake = .75d;                             // speed of intake
    public final static double rollerExpell = -.5d;                            // speed of expelling
    public final static double rollerRampFactor = .1d;                         // ramp change power limit per cycle (60hz)

    // lift 
//    public final static double liftHeight = 19.5d;                            // inches to raise bot
    public final static double liftMoverPower = 1d;                             // power to roll forward
    public final static double liftStartingError = 1d;                          // additional inches lift needs to extend to raise the robot 6 or 19 inches
//    public final static double liftEncoderClicksPerDegree = 42d*(12d * 24d/18d)/360d;      // 12:1 output shaft with 24/18 sprocket reduction
    public final static double liftEncoderClicksPerDegree = 42d*(20d * (24d/18d))/360d;  // 20:1 output shaft with 24/18 sprocket reduction
    public final static double liftEncoderClicksPerInch = (42d*20d)/(2d* Math.PI * 1d); //Clicks per revolution / Distance of curcomferance 
    public final static double liftLocality = liftEncoderClicksPerDegree * 2d;  // area around setpoint to use PID with
    public final static double liftPower = .85d;                                // extend lift power limit
    public final static double liftPidKp = liftPower / liftLocality;            // PID kP correction factor
//    public final static double liftPidKp = .85;                                 // PID kP correction factor
    public final static double liftPidKi = .0d;                                 // PID kI correction factor
    public final static double liftPidKd = .0d;                                 // PID kD correction factor
    public final static double liftStow = -.2d;                                 // retraction power 
    public final static double liftRamp = .1d;                                  // acceleration

    // // camera
    // public final static double cameraElevation = 35d;           // Camera height from floor;
    // public final static double cameraAngle = 30d;               // Angle between vertical and camera facing in degrees;
    // public final static double cameraFovX = 60d;                // Field of view in degrees (FOV) Pixy2
    // public final static double cameraFovY = 40d;                // Field of view in degrees (FOV) Pixy2
    // public final static double cameraMaxX = 80d;                // Max X resolution Pixy2
    // public final static double cameraMaxY = 60d;                // Max Y resolution Pixy2
    
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

    public CANEncoder2 leftArmEncoder;
//    public DigitalInput armLimitSwitch;

    public CANEncoder2 wristEncoder;
//    public DigitalInput wristLimitSwitch;

    public Encoder grabberEncoder;
    //    public DigitalInput grabberLimitSwitch;
//    public DigitalInput hatchLimitSwitch;               // TODO: Use two limit switches on physical bot just linked together
//    public MRColorSensor cargoColorSensor;

    public CANEncoder2 liftFrontEncoder;
    public CANEncoder2 liftBackEncoder;

    // public Pixy2 pixy2Normal;
    // public Pixy2 pixy2Inverted;

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
        leftArmEncoder = new CANEncoder2(armLeftSpeedController);
//        armLimitSwitch = new DigitalInput(armLimitSwitchDio);

        // wrist
        wristSpeedController =  new CANSparkMax(wristCan, MotorType.kBrushless);
        wristEncoder = new CANEncoder2(wristSpeedController);
//        wristLimitSwitch = new DigitalInput(wristLimitSwitchDio);

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
        liftBackSpeedController = new CANSparkMax(liftBackCan, MotorType.kBrushless);
        liftMoverSpeedController = new VictorSP(liftMoverPwm);
        liftFrontEncoder = new CANEncoder2(liftFrontSpeedController);
        liftBackEncoder = new CANEncoder2(liftBackSpeedController);

        // cameras
        cameraNormal = CameraServer.getInstance().startAutomaticCapture(cameraNormalUsb);
        cameraInverted = CameraServer.getInstance().startAutomaticCapture(cameraInvertedUsb); 
        // cameraNormal = new UsbCamera("USB Camera 0", cameraNormalUsb);
        // cameraInverted = new UsbCamera("USB Camera 1", cameraInvertedUsb);

        // Pixy2
        // pixy2Normal = Pixy2.createInstance(new I2CLink());
        // pixy2Normal.init(2);
        // pixy2Inverted = Pixy2.createInstance(new I2CLink(2));
        // pixy2Inverted.init();
    }
}
