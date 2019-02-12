package frc.robot;

import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Components.*;

public class Robot extends TimedRobot {
    private RobotMap robotMap;

    private Operator operator;
    private Drive drive;
    private Arm arm;
    private Wrist wrist;
    private Grabber grabber;

    private CameraManager cameraManager;
    private Mapper mapper;

    @Override
    public void robotInit() {
        // === setup and cleanup ===

        // setup hardware specific stuff
        robotMap = new RobotMap();

        // setup logical subsystem components
        operator = new Operator(robotMap);
        drive = new Drive(robotMap);
        arm = new Arm(robotMap);
        wrist = new Wrist(robotMap, arm);
        grabber = new Grabber(robotMap);

        // vision
        cameraManager = new CameraManager(robotMap);
        mapper = new Mapper(robotMap);
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
    }

    public void run() {
        if (this.isAutonomous() || this.isOperatorControl()) {
            // arm facing
            if (operator.normalFacingButton.get()) {
                arm.setFacingNormal(true);
            }
            if (operator.invertedFacingButton.get()) {
                arm.setFacingNormal(false);
            }

            // arm height
            if (operator.floorCargoButton.get()) {
                arm.moveFloorCargo(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.rocketCargo1Button.get()) {
                arm.moveRocketCargo1(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.rocketCargo2Button.get()) {
                arm.moveRocketCargo2(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.rocketCargo3Button.get()) {
                arm.moveRocketCargo3(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.shipCargoButton.get()) {
                arm.moveShipCargo(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.stationCargoButton.get()) {
                arm.moveStationCargo(arm.getFacingNormal());
                wrist.moveAligned();
            }

            if (operator.rocketHatch1Button.get()) {
                arm.moveRocketHatch1(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.rocketHatch2Button.get()) {
                arm.moveRocketHatch2(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.rocketHatch3Button.get()) {
                arm.moveRocketHatch3(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.shipHatchButton.get()) {
                arm.moveShipHatch(arm.getFacingNormal());
                wrist.moveAligned();
            }
            if (operator.stationHatchButton.get()) {
                arm.moveStationHatch(arm.getFacingNormal());
                wrist.moveAligned();
            }

            // drive facing
            drive.setFacing(operator.driveFacingToggleButton.toggleOn());

            // driver set up default move - may be overwritten by other elements
            drive.move(-operator.driveXboxController.getY(), operator.driveXboxController.getX(), false);

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
            arm.moveManual(-operator.armJoystick.getY(), arm.getFacingNormal());

            // POV moves wrist manually overwriting previous position selection - only to correct for placing
            wrist.moveManual(Geometry.getXFromAngle(operator.armJoystick.getPOV()));
        }

        // apply component changes
        drive.run();
        arm.run();
        wrist.run();
        grabber.run();

        putTelemetry();
    }

    private void putTelemetry() {
    }
}
