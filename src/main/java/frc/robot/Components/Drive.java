package frc.robot.Components;

import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.*;
import frc.robot.HardwareMap;

/**
 * Drive base that requires constant motive direction from either joystick or sensor assist
 * DESIGNED TO ONLY APPLY CHANGES IN RUN()
 */
public class Drive {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Drive");

    DifferentialDrive differentialDrive; 
    ADXRS450_Gyro gyro;

    double speed = 0;
    double rotation = 0;
    boolean squareInputs = false;
    boolean facingNormal = true;

    double headingTarget = 0;
    double distanceTarget = 0;
    double headingError = 0;
    double distanceError = 0; 

    States state = States.Stopped;

    public enum States {
        Stopped, Moving, AssistingStraight, AssistingTurn
    }

    public States getState() {
        return state;
    }
    
    public Drive(HardwareMap hardwareMap) {
        differentialDrive = new DifferentialDrive(hardwareMap.leftDriveSpeedController, hardwareMap.rightDriveSpeedController);
        differentialDrive.setExpiration(hardwareMap.safetyExpiration);
        differentialDrive.setSafetyEnabled(true);

        gyro = hardwareMap.driveGyro;
        stop();
    }

    public void cleanup() {
        stop();
    }

    // === Executing per Period ===
    public void run() {
        differentialDrive.arcadeDrive(facingNormal ? speed : -speed, rotation, squareInputs);
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Distance Target", distanceTarget);
        telemetry.putDouble("Heading", gyro.getAngle());
        telemetry.putDouble("Heading Target", headingTarget);
        telemetry.putBoolean("Facing Normal (bot)", facingNormal);
        telemetry.putDouble("Speed (forward|back)", speed);
        telemetry.putDouble("Rotation (turn left|right)", rotation);
        telemetry.putString("Version", "1.0.0");
    }

    // === User Trigerrable States ===
    public void setFacing(boolean facingNormal) {
        this.facingNormal = facingNormal;
    }

    public void stop() {
        state = States.Stopped;

        speed = 0;
        rotation = 0;
        squareInputs = false;
    }

    // should be default command - user joystick inputs
    public void move(double speed, double rotation, boolean squareInputs) {
        state = States.Moving;

        this.speed = speed;
        this.rotation = rotation;
        this.squareInputs = squareInputs;
    }

    // layered onto Move to help drive straight according to gyro
    public void assistStraight() {
        if (state != States.AssistingStraight) {
            state = States.AssistingStraight;
            headingTarget = gyro.getAngle();
        }

        headingError = (gyro.getAngle() - headingTarget);
        this.rotation = (headingError * .1d);                         // proportional correction based on drift. change value
    }

    // Expects x,y distances in terms of current bot facing and center of the bot.
    // Figures out rotation needed based on current speed in order to line up best 
    public void assistLineup(double x1, double y1, double x2, double y2) {
        if (state != States.AssistingTurn) {
            state = States.AssistingTurn;
        }

        // which point is the waypoint and which is endpoint? assume waypoint closest by distance.
        // make waypoint 6" further away from endpoint.

        // if center of bot is within 6" of waypoint then target turning to endpoint
            // how to use points to plot a curve to end up at endpoint and perfectly facing endpoint?
        // else target turning toward waypoint
            // how to use points to plot a curve to end up at waypoint and facing endpoint?
        
        // how to adjust for current speed? Needed? Pathmarker software? Quick approximation?

        headingError = (gyro.getAngle() - headingTarget);
        this.rotation = headingError * .1d;  
    }

    // === Internally Trigerrable States ===

}