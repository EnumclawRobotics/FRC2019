package frc.robot;

//import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Components.*;

public class Robot extends TimedRobot {
    private RobotMap robotMap;

    private CameraManager cameraManager;
    private Mapper mapper;

    private Operator operator;
    private Drive drive;
    private Arm arm;
    private Wrist wrist;
    private Grabber grabber;
    private Lifter lifter;

    @Override
    public void robotInit() {
        // === setup and cleanup ===

        // setup hardware specific stuff
        robotMap = new RobotMap();

        // vision
        cameraManager = new CameraManager(robotMap);
        mapper = new Mapper(robotMap);
        
        // setup logical subsystem components
        operator = new Operator(robotMap);
        drive = new Drive(robotMap);
        arm = new Arm(robotMap);
        wrist = new Wrist(robotMap, arm);
        grabber = new Grabber(robotMap);
        lifter = new Lifter(robotMap);
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
    }

    // === Executing per Period ===

    public void stop() {
        drive.stop();
        arm.stop();
        wrist.stop();
        grabber.stop();
        lifter.stop();
        mapper.stop();
        cameraManager.stop();
    }

    public void run() {
        // get situation changes
        if (this.isAutonomous() || this.isOperatorControl()) {
            // drive facing
            drive.setFacing(operator.driveFacingToggleButton.toggleOn());

            // arm facing
            arm.setFacing(operator.armFacingToggleButton.toggleOn());
        }

        // get any informational component changes
        cameraManager.run();
        mapper.run();

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
            // if (operator.hatchShipButton.get()) {
            //     arm.moveShipHatch(arm.getFacingNormal());
            //     wrist.moveAligned();
            // }
            // if (operator.hatchStationButton.get()) {
            //     arm.moveStationHatch(arm.getFacingNormal());
            //     wrist.moveAligned();
            // }

            // driver set up default move - may be overwritten by other elements
            drive.move(-operator.driveXboxController.getY(Hand.kLeft), operator.driveXboxController.getX(Hand.kRight), false);

            // assist in straight driving? add turbo function?
            if (operator.driveXboxController.getBumper(Hand.kLeft)) {
                drive.assistStraight();
            }

            // assist in turning to target?
            if (operator.driveXboxController.getBumper(Hand.kRight)) {
                drive.assistRotation(mapper.getVector());
            }
        }

        if (this.isTest() || this.isAutonomous() || this.isOperatorControl()) {
            // joystick moves arm manually overwriting previous height selection - only to correct for placing
            arm.moveManual(-operator.armXboxController.getY(Hand.kLeft), arm.getFacingNormal());

            // POV moves wrist manually overwriting previous position selection - only to correct for placing
            wrist.moveManual(-operator.armXboxController.getY(Hand.kRight));

            // lifter 
            // lifting back up 
            // TODO: Put a protection in that we cant lift up but can go down if going too fast
            lifter.moveBackLift((operator.armXboxController.getBumper(Hand.kRight) ? 1 : 0) 
                                - operator.armXboxController.getTriggerAxis(Hand.kRight));

            // lifting front up 
            // TODO: Put a protection in that we cant lift up but can go down if going too fast
            lifter.moveFrontLift((operator.armXboxController.getBumper(Hand.kLeft) ? 1 : 0) 
                                - operator.armXboxController.getTriggerAxis(Hand.kLeft));

            // rolling forward / back
            lifter.move(operator.armXboxController.getXButtonPressed() ? -1 : 0);
            lifter.move(operator.armXboxController.getYButtonPressed() ? 1 : 0);
        }

        // apply component changes in order
        drive.run();
        arm.run();
        wrist.run();
        grabber.run();
        lifter.run();

        putTelemetry();
    }

    private void putTelemetry() {
    }
}
