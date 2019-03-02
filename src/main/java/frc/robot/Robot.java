package frc.robot;

import common.instrumentation.Telemetry;
import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Components.*;

public class Robot extends TimedRobot {
    private Telemetry telemetry = new Telemetry("Robot");
    
    private RobotMap robotMap;

//    private CameraManager cameraManager;
//    private Mapper mapper;

    private Operator operator;
    private Drive drive;
    private Arm arm;
    private Wrist wrist;
    private Grabber grabber;
//    private Lifter lifter;

    @Override
    public void robotInit() {
        // === setup and cleanup ===

        // setup hardware specific stuff
        robotMap = new RobotMap();

        // vision
//        cameraManager = new CameraManager(robotMap);
//        mapper = new Mapper(robotMap);
        
        // setup logical subsystem components
        operator = new Operator(robotMap);
        drive = new Drive(robotMap);
        arm = new Arm(robotMap);
        wrist = new Wrist(robotMap, arm);
        grabber = new Grabber(robotMap);
 //       lifter = new Lifter(robotMap);
    }

    // === Modes ===

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        putTelemetry();
    }

    @Override
    public void disabledInit() {
        stop();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        arm.init();
        wrist.init();
//        cameraManager.init();
    }

    @Override
    public void autonomousPeriodic() {
        run();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        run();
    }

    @Override
    public void testPeriodic() {
        run();
    }

    // === Executing per Period ===

    public void stop() {
        drive.stop();
        arm.stop();
        wrist.stop();
        grabber.stop();
//        lifter.stop();
//        mapper.stop();
//        cameraManager.stop();
    }

    public void run() {
        // get changes in situation
        if (this.isAutonomous() || this.isOperatorControl()) {
            // drive facing
            if (operator.driveXboxController.getYButton()) {
                drive.setFacing(true);
            } else if (operator.driveXboxController.getXButton()) {
                drive.setFacing(false);
            }

            // arm facing
            arm.setFacing(arm.getAngle() < 180);
        }

        // // get any informational component changes
        // cameraManager.run();
        // mapper.run();


        if (this.isAutonomous() || this.isOperatorControl()) {
            // driver set up default move - may be overwritten by other elements
            drive.move(-operator.driveXboxController.getY(Hand.kLeft), operator.driveXboxController.getX(Hand.kRight), false);

            // // assist in straight driving? add turbo function?
            // if (operator.driveXboxController.getBumper(Hand.kLeft)) {
            //     drive.assistStraight();
            // }

            // // assist in turning to target?
            // if (operator.driveXboxController.getBumper(Hand.kRight)) {
            //     drive.assistRotation(mapper.getVector());
            // }
        }

        if (this.isAutonomous() || this.isOperatorControl()) {
            // arm height
            if (operator.cargoFloorButton.get()) {
                arm.moveFloorCargo(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.cargoRocket1Button.get()) {
                arm.moveRocketCargo1(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.cargoRocket2Button.get()) {
                arm.moveRocketCargo2(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.cargoRocket3Button.get()) {
                arm.moveRocketCargo3(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.cargoShipButton.get()) {
                arm.moveShipCargo(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.cargoStationButton.get()) {
                arm.moveStationCargo(arm.getFacingNormal());
                wrist.moveAligned();
            }

            if (operator.hatchRocket1Button.get()) {
                arm.moveRocketHatch1(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.hatchRocket2Button.get()) {
                arm.moveRocketHatch2(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.hatchRocket3Button.get()) {
                arm.moveRocketHatch3(arm.getFacingNormal());
                wrist.moveAligned();
            }
        }

        if (this.isTest() || this.isAutonomous() || this.isOperatorControl()) {
            // joystick moves arm manually overwriting previous height selection - only to correct for placing
            arm.moveManual(-operator.armXboxController.getY(Hand.kLeft));

            // POV moves wrist manually overwriting previous position selection - only to correct for placing
            wrist.moveManual(-operator.armXboxController.getY(Hand.kRight));
        }

        if (this.isTest() || this.isAutonomous() || this.isOperatorControl()) {
            if (operator.armXboxController.getBumper(Hand.kRight)) {
                grabber.expell();
            } else if (operator.armXboxController.getBumper(Hand.kLeft)) {
                grabber.intake();
            } else {
                grabber.hold();
            }
            if (operator.armXboxController.getXButton()) {
                grabber.close();
            } else if (operator.armXboxController.getYButton()) {
                grabber.openCargo();
            } else {
                grabber.grip();
            }
        }

        if (this.isTest() || this.isAutonomous() || this.isOperatorControl()) {
            // // handle lifting by overloading the drive controller
            // if (operator.driveXboxController.getStartButton()) {
            //     lifter.setLiftingActive();
            // }
            // if (operator.driveXboxController.getYButton()) {
            //     lifter.setMaroonActive();
            // }
            // if (operator.driveXboxController.getXButton()) {
            //     lifter.setGoldActive();
            // }
            // if (lifter.getLiftingActive()) {
            //    if (operator.driveXboxController.getPOV() != -1) {
            //        lifter.move(Geometry.getYFromAngle(operator.driveXboxController.getPOV()));
            //    }
            // } 
            // if (operator.driveXboxController.getBackButton()) {
            //     lifter.stop();
            // }
        }

        // apply component changes in order
        drive.run();
        arm.run();
        wrist.run();
        grabber.run();
//        lifter.run();

        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putString("Version", "1.0.0");
    }
}
