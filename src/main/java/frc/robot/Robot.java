package frc.robot;

import common.oiHelpers.ToggleButton;
import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.Components.*;

public class Robot extends TimedRobot {
    public RobotMap robotMap;

    public Operator operator;
    public Drive drive;
    public Arm arm;
    public Wrist wrist;
    public Grabber grabber;

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

        // update components
        drive.setFacing(operator.driveFacing.toggleOn());
        drive.move(-operator.xboxController.getY(), operator.xboxController.getX(), false);



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
    }

    if (this.isTest()) {
        arm.moveManual(-operator.armJoystick.getY(), arm.getFacingNormal());
        wrist.moveManual(Geometry.getYFromAngle(operator.armJoystick.getPOV(0)));
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
