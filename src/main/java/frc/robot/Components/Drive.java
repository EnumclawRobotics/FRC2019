package frc.robot.Components;

import common.instrumentation.Telemetry;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.*;
import common.pixy2Api.*;

/**
 * Drive base that requires constant motive direction from either joystick or sensor assist
 * DESIGNED TO ONLY APPLY CHANGES IN RUN()
 */
public class Drive {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Drive");

    DifferentialDrive differentialDrive; 
//    ADXRS450_Gyro gyro;
//    Mapper mapper;

    double speed = 0;
    double rotation = 0;
    boolean isLimiterOn = false;
    boolean facingNormal = true;

//    double headingTarget = 0;
//    double distanceTarget = 0;
 
    double headingError = 0;
    double distanceError = 0; 

    States state = States.Stopped;

    public enum States {
        Stopped, Moving, 
        // AssistingStraight, 
        AssistingTurn
    }

    public States getState() {
        return state;
    }
    
    public Drive(RobotMap robotMap) {
        // we need to invert the right hand side motors
//        robotMap.driveRightMaroonSpeedController.setInverted(true);
//        robotMap.driveRightBlackSpeedController.setInverted(true);

        // set up drive
        differentialDrive = new DifferentialDrive(new SpeedControllerGroup(robotMap.driveLeftFrontSpeedController, 
                                                                            robotMap.driveLeftBackSpeedController), 
                                                new SpeedControllerGroup(robotMap.driveRightFrontSpeedController, 
                                                                            robotMap.driveRightBackSpeedController));

        // ensure that motors get stopped if runaway
//        differentialDrive.setExpiration(RobotMap.safetyExpiration);
//        differentialDrive.setSafetyEnabled(true);

        // gyro = robotMap.driveGyro;
        stop();
    }

    public void cleanup() {
        stop();
    }

    // === Executing per Period ===
    public void setlimiterOff() {
        isLimiterOn = false;
    }

    public void run() {
        // if operator is not turning it off then limit forward speed
        if (isLimiterOn) {
            speed = speed * RobotMap.driveSpeedLimiter;
        }
        // always apply rotation limits
        rotation = rotation * RobotMap.driveRotationLimiter; 

        differentialDrive.arcadeDrive(facingNormal ? speed : -speed, rotation, false);
        putTelemetry();

        isLimiterOn = true;
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
//        telemetry.putDouble("Distance Target", distanceTarget);
//        telemetry.putDouble("Heading", gyro.getAngle());
//        telemetry.putDouble("Heading Target", headingTarget);
        telemetry.putBoolean("Facing Normal (bot)", facingNormal);
        telemetry.putDouble("Speed (forward|back)", speed);
        telemetry.putDouble("Rotation (turn left|right)", rotation);
        telemetry.putString("Version", "1.0.0");
    }

    // === User Trigerrable States ===
    public void setFacing(boolean facingNormal) {
        this.facingNormal = facingNormal;
    }

    public void toggleFacing() {
        this.facingNormal = !this.facingNormal;
    }

    public void stop() {
        state = States.Stopped;

        speed = 0;
        rotation = 0;
        
        differentialDrive.stopMotor();
    }

    // should be default command - user joystick inputs
    public void move(double speed, double rotation, boolean squareInputs) {
        state = States.Moving;

        this.speed = speed;
        this.rotation = rotation;
    }

    // // layered onto Move to help drive straight according to gyro
    // public void assistStraight() {
    //     if (state != States.AssistingStraight) {
    //         state = States.AssistingStraight;
    //         headingTarget = gyro.getAngle();
    //     }
    //
    //     headingError = (gyro.getAngle() - headingTarget);
    //     this.rotation = (headingError * .1d);                         // proportional correction based on drift. change value
    // }

    // // Figures out rotation needed based on current speed in order to line up with primary white line found 
    // public void assistRotation(Vector vector) {
    //     if (state != States.AssistingTurn) {
    //         state = States.AssistingTurn;
    //     }

    //     // can camera find a primary while line? then supply some rotation help, otherwise ignore
    //     if (vector != null) {
    //         this.rotation = Mapper.getRotation(vector, speed);
    //     }
    // }
}