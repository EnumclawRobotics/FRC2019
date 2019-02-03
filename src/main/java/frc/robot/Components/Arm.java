package frc.robot.Components;

import com.revrobotics.*;
import common.instrumentation.Telemetry;
import common.util.Geometry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import frc.robot.*;

/**
* Arm/shoulder for holding up wrist 
*  DESIGNED TO ONLY APPLY CHANGES IN RUN()
*/
public class Arm {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Arm");

    DifferentialDrive arm;             // NOTE: we will never run this differentially. Using it for default safety enabled             
    CANEncoder armEncoder; 
    DigitalInput armLimitSwitch;
    
    boolean facingNormal = true;
    double baseClicks = 0;
    double targetHeight = 0;
    double targetAngle = 0;
    double targetClicks = 0;
    double feedForward = 0;
    double power = 0;

    States state = States.Stopped;

    public enum States {
        Stopped, 
        MovingRocketHatch1, MovingRocketHatch2, MovingRocketHatch3,
        MovingRocketCargo1, MovingRocketCargo2, MovingRocketCargo3, 
        MovingFloorCargo, 
        MovingStationHatch, MovingStationCargo,
        MovingShipHatch, MovingShipCargo 
    }

    public States getState() {
        return state;
    }
    
    public Arm(RobotMap robotMap) {
        arm = new DifferentialDrive(robotMap.leftArmSpeedController, 
                                            robotMap.rightArmSpeedController);
        arm.setExpiration(RobotMap.safetyExpiration);
        arm.setSafetyEnabled(true);
        arm.setRightSideInverted(true);                 // ensure that both motors work in the same direction
        arm.setDeadband(0);                             // even supply little amounts of power

        armEncoder = robotMap.armEncoder;
        armLimitSwitch = robotMap.armLimitSwitch;

        stop();
    }

    // === Executing per Period ===

    // used by wrist
    public double getAngle() {
        return angleFromClicks(armEncoder.getPosition());
    }    

    // set before setting height using the 'move' items
    public void setFacing(boolean facingNormal) {
        this.facingNormal = facingNormal;
    }

    public void moveRocketHatch1() {
        state = States.MovingRocketHatch1;
        targetHeight = FieldMap.heightRocketHatch1;
    }
    public void moveRocketHatch2() {
        state = States.MovingRocketHatch2;
        targetHeight = FieldMap.heightRocketHatch2;
    }
    public void moveRocketHatch3() {
        state = States.MovingRocketHatch3;
        targetHeight = FieldMap.heightRocketHatch3;
    }

    public void moveRocketCargo1() {
        state = States.MovingRocketCargo1;
        targetHeight = FieldMap.heightRocketCargo1;
    }
    public void moveRocketCargo2() {
        state = States.MovingRocketCargo2;
        targetHeight = FieldMap.heightRocketCargo2;
    }
    public void moveRocketCargo3() {
        state = States.MovingRocketCargo3;
        targetHeight = FieldMap.heightRocketCargo3;
    }

    public void moveFloorCargo() {
        state = States.MovingFloorCargo;
        targetHeight = FieldMap.heightFloorCargo;
    }

    public void moveStationHatch() {
        state = States.MovingStationHatch;
        targetHeight = FieldMap.heightStationHatch;
    }
    public void moveStationCargo() {
        state = States.MovingStationCargo;
        targetHeight = FieldMap.heightStationCargo;
    }

    public void moveShipHatch() {
        state = States.MovingShipHatch;
        targetHeight = FieldMap.heightShipHatch;
    }
    public void moveShipCargo() {
        state = States.MovingShipCargo;
        targetHeight = FieldMap.heightShipCargo;
    }

    public void run() {
        // current angle implies how much force gravity applies
        double currentAngle = angleFromClicks(armEncoder.getPosition());
        feedForward = Math.abs(Math.cos(Math.toRadians(currentAngle)) * RobotMap.armFeedForwardFactor);

        // we need to correct the sign to map the cos() to how our arm reacts with gravity
        // opposite side of the bot gravity helps compared to the motor direction so reverse the feedforward 
        if ((currentAngle >= 180 && currentAngle < 360)) {
            feedForward = -feedForward;
        }

        // moving to position - compare error degrees to 90 degrees for a percentage
        targetAngle = armAngleFromHeight(targetHeight, facingNormal);
        targetClicks = clicksFromAngle(targetAngle);

        double error = ((targetClicks - armEncoder.getPosition()) / RobotMap.armEncoderCicksPerDegree) / 90;
        double correctionPower = .5d;
        power = Geometry.clip(feedForward + (error * correctionPower), -1, 1);

        // always ensure that we never use rotation. i.e. both arm motors should move as one not try to fight each other
        arm.arcadeDrive(power, 0, false);
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("Arm Clicks", armEncoder.getPosition());
        telemetry.putDouble("Target Clicks", targetClicks);
        telemetry.putDouble("FeedForward", feedForward);
        telemetry.putDouble("Power", power);
        telemetry.putString("Version", "1.0.0");
    }

    // === User Trigerrable States ===
    public void stop() {
        state = States.Stopped;
        power = 0;
    }

    // === Helpers ===

    // given clicks figure out angle 
    public double angleFromClicks(double clicks) {
        return (((double)(clicks - baseClicks)) / RobotMap.armEncoderCicksPerDegree);
    }
    
    // given a target height figure out target clicks  
    public double clicksFromAngle(double angle) {
        return (angle * RobotMap.armEncoderCicksPerDegree) + baseClicks;
    }

    // getting an arm angle in degrees from a height above ground
    public double armAngleFromHeight(double targetHeight, boolean facingNormal) {
        double sine;
        double angle;

        // figure sine about pivot point for all heights eg given '|'s in |X| shape figure out the X angles
        if (facingNormal) {
            if (RobotMap.heightArmPivot >= targetHeight) { 
                // reaching low on start side
                sine = (RobotMap.heightArmPivot - targetHeight)/RobotMap.armLength;
                angle = Math.toDegrees(Math.asin(sine));
            } else {
                // reaching high on start side
                sine = (RobotMap.heightArmPivot - targetHeight)/RobotMap.armLength;
                angle = Math.toDegrees(Math.asin(sine)) + 90;
            }
        }
        else {
            if (RobotMap.heightArmPivot >= targetHeight) { 
                // reaching low on opposite side
                sine = (RobotMap.heightArmPivot - targetHeight)/RobotMap.armLength;
                angle = Math.toDegrees(Math.asin(sine)) + 270;
            } else {
                // reaching high on opposite side
                sine = (RobotMap.heightArmPivot - targetHeight)/RobotMap.armLength;
                angle = Math.toDegrees(Math.asin(sine)) + 180;
            }
        }

        return angle;
    }

}