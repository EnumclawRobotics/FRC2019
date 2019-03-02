package frc.robot.Components;

import common.instrumentation.Telemetry;
import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;

/**
* Arm/shoulder for holding up wrist 
*  DESIGNED TO ONLY APPLY CHANGES IN RUN()
*/
public class Arm {
    // -- setup and cleanup ===
    private Telemetry telemetry = new Telemetry("Robot/Arm");

    // parts of this subsystem
    private SpeedController speedController;
    private CANEncoder2 encoder;
    private DigitalInput limitSwitch; 
    
    // set through command
    private double targetHeight = 0;
    private boolean facingNormal = true;                // Seperate setting from which end is front on the bot 

    // set at autonomous init
    private double baseClicks = 0;
    
    // derived as needed and stored for reporting
    private double targetAngle = 0;
    private double targetClicks = 0;
    private double feedForward = 0;
    private double power = 0;

    private States state = States.Stopped;

    public enum States {
        Stopped,
        MovingManual,
        MovingFloorCargo, 
        MovingStationHatch, MovingStationCargo,
        MovingRocketHatch1, MovingRocketHatch2, MovingRocketHatch3,
        MovingRocketCargo1, MovingRocketCargo2, MovingRocketCargo3, 
        MovingShipHatch, MovingShipCargo 
    }

    // Constructor that saves controller and sensor references
    public Arm(RobotMap robotMap) {
        speedController = robotMap.armSpeedController; 

        //((MotorSafety)speedController).setSafetyEnabled(true);
        //((MotorSafety)speedController).setExpiration(RobotMap.safetyExpiration);

        encoder = robotMap.armEncoder;
        limitSwitch = robotMap.armLimitSwitch;

        stop();
    }

    public void init() {
        baseClicks = encoder.get();
    }

    // not triggerable by user 
    public void stop() {
        state = States.Stopped;
        power = 0;
        speedController.stopMotor();
    }

    // === PER CYCLE ===

    public States getState() {
        return state;
    }

    public boolean getFacingNormal() {
        return facingNormal;
    }

    // degrees 0 - 360
    public double getAngle() {
        return angleFromClicks(encoder.get());
    }    

    // degrees
    public double getTargetAngle() {
        return targetAngle;
    }
    
    public void setFacing(boolean facingNormal) {
        this.facingNormal = facingNormal;
    }

    public void moveManual(double power) {
        state = States.MovingManual;

        this.power = power;

        // if (targetHeight < FieldMap.heightFloorCargo) {
        //     targetHeight = FieldMap.heightFloorCargo;
        // } else if (targetHeight > FieldMap.heightRocketCargo3) {
        //     targetHeight = FieldMap.heightRocketCargo3;
        // } else {
        //     targetHeight += move;     
        // }
        // this.facingNormal = facingNormal;

    }

    public void moveRocketHatch1(boolean facingNormal) {
        state = States.MovingRocketHatch1;
        targetHeight = FieldMap.heightRocketHatch1;
        this.facingNormal = facingNormal;
    }
    public void moveRocketHatch2(boolean facingNormal) {
        state = States.MovingRocketHatch2;
        targetHeight = FieldMap.heightRocketHatch2;
        this.facingNormal = facingNormal;
    }
    public void moveRocketHatch3(boolean facingNormal) {
        state = States.MovingRocketHatch3;
        targetHeight = FieldMap.heightRocketHatch3;
        this.facingNormal = facingNormal;
    }

    public void moveRocketCargo1(boolean facingNormal) {
        state = States.MovingRocketCargo1;
        targetHeight = FieldMap.heightRocketCargo1;
        this.facingNormal = facingNormal;
    }
    public void moveRocketCargo2(boolean facingNormal) {
        state = States.MovingRocketCargo2;
        targetHeight = FieldMap.heightRocketCargo2;
        this.facingNormal = facingNormal;
    }
    public void moveRocketCargo3(boolean facingNormal) {
        state = States.MovingRocketCargo3;
        targetHeight = FieldMap.heightRocketCargo3;
        this.facingNormal = facingNormal;
    }

    public void moveFloorCargo(boolean facingNormal) {
        state = States.MovingFloorCargo;
        targetHeight = FieldMap.heightFloorCargo;
        this.facingNormal = facingNormal;
    }

    public void moveStationHatch(boolean facingNormal) {
        state = States.MovingStationHatch;
        targetHeight = FieldMap.heightStationHatch;
        this.facingNormal = facingNormal;
    }
    public void moveStationCargo(boolean facingNormal) {
        state = States.MovingStationCargo;
        targetHeight = FieldMap.heightStationCargo;
        this.facingNormal = facingNormal;
    }

    public void moveShipHatch(boolean facingNormal) {
        state = States.MovingShipHatch;
        targetHeight = FieldMap.heightShipHatch;
        this.facingNormal = facingNormal;
    }
    public void moveShipCargo(boolean facingNormal) {
        state = States.MovingShipCargo;
        targetHeight = FieldMap.heightShipCargo;
        this.facingNormal = facingNormal;
    }

    public void run() {
        // if (state != States.Stopped) {
        //     if (state != States.MovingManual) {
        //         // current angle implies how much force gravity applies and so what we need to make neutral
        //         double angle = getAngle();
        //         feedForward = Geometry.gravity(angle) * RobotMap.armFeedForwardFactor;

        //         // moving to a height
        //         targetAngle = angleFromHeight(targetHeight, facingNormal);
        //         targetClicks = clicksFromAngle(targetAngle);

        //         // apply an (P)id error correction
        //         double errorClicks = targetClicks - encoder.get();
        //         double correctionP = errorClicks * RobotMap.armKpFactor;
        //         power = Geometry.clip(feedForward + correctionP, -1, 1);

        //         // is limit switch saying we are going too far?
        //         if (limitSwitch.get() 
        //             && ((angle > 180 && power > 0)                  // opposite side too far 
        //                 || (angle < 180 && power < 0))) {           // normal side too far
        //             power = 0;
        //         }
        //     }

            // set the power on the motors
            speedController.set(power);
            putTelemetry();
//        }
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Angle", getAngle());
        telemetry.putDouble("Power", power);
        telemetry.putDouble("Clicks", encoder.get());
        telemetry.putDouble("TargetHeight", targetHeight);
        telemetry.putDouble("TargetAngle", targetAngle);
        telemetry.putDouble("TargetClicks", targetClicks);
        telemetry.putDouble("FeedForward", feedForward);
        telemetry.putString("Version", "1.1.0");
    }

    // === Helpers ===

    // given clicks figure out angle 
    private double angleFromClicks(double clicks) {
        return (((double)(clicks - baseClicks)) / RobotMap.armEncoderClicksPerDegree);
    }
    
    // given a target angle figure out target clicks  
    private double clicksFromAngle(double angle) {
        return (angle * RobotMap.armEncoderClicksPerDegree) + baseClicks;
    }

    // getting an angle in degrees from a height above ground
    private double angleFromHeight(double targetHeight, boolean facingNormal) {
        double height;
        double angle;

        // figure angle about pivot point given reach points in the vertical plane
        if (facingNormal) {
            if (RobotMap.armPivotHeight >= targetHeight) { 
                // reaching low on start side - height is cos
                height = (RobotMap.armPivotHeight - targetHeight)/RobotMap.armLength;
                angle = Math.toDegrees(Math.acos(height));
            } else {
                // reaching high on start side - height is sin 
                height = (targetHeight - RobotMap.armPivotHeight)/RobotMap.armLength;
                angle = Math.toDegrees(Math.asin(height)) + 90;
            }
        }
        else {
            if (RobotMap.armPivotHeight >= targetHeight) { 
                // reaching low on opposite side - height is sin
                height = (RobotMap.armPivotHeight - targetHeight)/RobotMap.armLength;
                angle = Math.toDegrees(Math.asin(height)) + 270;
            } else {
                // reaching high on opposite side - height is cos
                height = (RobotMap.armPivotHeight - targetHeight)/RobotMap.armLength;
                angle = Math.toDegrees(Math.acos(height)) + 180;
            }
        }

        return angle;
    }
}