package frc.robot.Components;

import com.revrobotics.CANSparkMax.IdleMode;

import common.instrumentation.Telemetry;
import common.util.Functions;
import edu.wpi.first.wpilibj.*;
import frc.robot.*;
import common.util.PID;

/**
* Arm/shoulder for holding up wrist 
*  DESIGNED TO ONLY APPLY CHANGES IN RUN()
*/
public class Arm {
    // -- setup and cleanup ===
    private Telemetry telemetry = new Telemetry("Robot/Arm");

    // parts of this subsystem
    private RampSpeedController leftSpeedController;
    private RampSpeedController rightSpeedController;
    private CANEncoder2 encoder;
    private DigitalInput limitSwitch; 
    private PID pid;

    // involved parts
    private Wrist wrist;

    // set/derived through command
    private double targetHeight = 0;
    private double targetAngle = 0;
    private double targetClicks = 0;

    // set at autonomous init
    private double baseClicks = 0;
    
    // derived as needed and stored for reporting
    private double feedForward = 0;
    private double pidPower = 0;
    private double power = 0;

    private States state = States.Stopped;

    public enum States {
        Stopped,
        MovingStowed,
        MovingFloorCargo, 
        MovingStationHatch, MovingStationCargo,
        MovingRocketHatch1, MovingRocketHatch2, MovingRocketHatch3,
        MovingRocketCargo1, MovingRocketCargo2, MovingRocketCargo3, 
        MovingShipHatch, MovingShipCargo, 
        MovingManual,
    }

    // Constructor that saves controller and sensor references
    public Arm(RobotMap robotMap) {
        // config motor controllers
        robotMap.armLeftSpeedController.setIdleMode(IdleMode.kBrake);
        robotMap.armRightSpeedController.setInverted(true);
        robotMap.armRightSpeedController.setIdleMode(IdleMode.kBrake);

        // keep references
        leftSpeedController = new RampSpeedController(robotMap.armLeftSpeedController, RobotMap.armRampFactor); 
        rightSpeedController = new RampSpeedController(robotMap.armRightSpeedController, RobotMap.armRampFactor); 
        encoder = robotMap.armEncoder;
        limitSwitch = robotMap.armLimitSwitch;

        pid = new PID();
        //pidController.setZnGainsP(RobotMap.armPidKu); 
        pid.setGainsPID(RobotMap.armPidKp, RobotMap.armPidKi, RobotMap.armPidKd);

        stop();
    }

    // assumes that arm is Stowed at start of autonomous
    public void init(Wrist wrist) {
        // involved 
        this.wrist = wrist;

        baseClicks = encoder.get();

        // reset derived fields
        targetClicks = baseClicks;                        
        feedForward = 0;
        power = 0;
    }

    // not triggerable by user 
    public void stop() {
        state = States.Stopped;
        power = 0;
        leftSpeedController.stopMotor();
        rightSpeedController.stopMotor();
    }

    private void setMotors(double value) {
        leftSpeedController.set(value);
        rightSpeedController.set(value);
    }

    // === PER CYCLE ===

    public States getState() {
        return state;
    }

    // degrees 0 - 360
    public double getAngle() {
        return angleFromClicks(getClicks());
    }    

    // start side goes [0-180) 
    public boolean getFacingNormal() {
        return (getAngle() < 180);
    }

    // encoder clicks
    public double getClicks() {
        return encoder.get();
    }

    public void moveStowed() {
        state = States.MovingStowed;
        targetHeight = -1;
        targetClicks = (getFacingNormal() ? baseClicks : baseClicks + RobotMap.armEncoderClicksPerDegree * (360 - 2 * RobotMap.armStowedAngle));
    }

    public void moveRocketHatch1() {
        state = States.MovingRocketHatch1;
        targetHeight = FieldMap.heightRocketHatch1;
        targetClicks = clicksFromAngle(targetAngle);
    }
    public void moveRocketHatch2() {
        state = States.MovingRocketHatch2;
        targetHeight = FieldMap.heightRocketHatch2;
        targetClicks = clicksFromAngle(targetAngle);
    }
    public void moveRocketHatch3() {
        state = States.MovingRocketHatch3;
        targetHeight = FieldMap.heightRocketHatch3;
        targetClicks = clicksFromAngle(targetAngle);
    }

    public void moveRocketCargo1() {
        state = States.MovingRocketCargo1;
        targetHeight = FieldMap.heightRocketCargo1;
        targetClicks = clicksFromAngle(targetAngle);
    }

    public void moveRocketCargo2() {
        state = States.MovingRocketCargo2;
        targetHeight = FieldMap.heightRocketCargo2;
        targetClicks = clicksFromAngle(targetAngle);
    }
    public void moveRocketCargo3() {
        state = States.MovingRocketCargo3;
        targetHeight = FieldMap.heightRocketCargo3;
        targetClicks = clicksFromAngle(targetAngle);
    }

    public void moveFloorCargo() {
        state = States.MovingFloorCargo;
        targetHeight = FieldMap.heightFloorCargo;
        targetClicks = clicksFromAngle(targetAngle);
    }

    public void moveStationHatch() {
        state = States.MovingStationHatch;
        targetHeight = FieldMap.heightStationHatch;
        targetClicks = clicksFromAngle(targetAngle);
    }
    public void moveStationCargo() {
        state = States.MovingStationCargo;
        targetHeight = FieldMap.heightStationCargo;
        targetClicks = clicksFromAngle(targetAngle);
    }

    public void moveShipHatch() {
        state = States.MovingShipHatch;
        targetHeight = FieldMap.heightShipHatch;
        targetClicks = clicksFromAngle(targetAngle);
    }
    public void moveShipCargo() {
        state = States.MovingShipCargo;
        targetHeight = FieldMap.heightShipCargo;
        targetClicks = clicksFromAngle(targetAngle);
    }

    public void moveManual(double controlPower) {
        // ignore deadband and defaults where we are not moving joystick
        if (Math.abs(controlPower) > .01) {
            state = States.MovingManual;
            targetHeight = -1;
            targetClicks = getClicks() + (controlPower *  RobotMap.armEncoderClicksPerDegree * .5);
        }
    }

    public void run() {
        if (state != States.Stopped) {
            // current angle implies how much force gravity applies and so what we need to make neutral
            double angle = getAngle();
            feedForward = Functions.gravity(angle) * RobotMap.armFeedForwardFactor;
            // TODO: Incorporate change in wrist angle 

            // get PID output that is best to go towards the target clicks
            pidPower = pid.update(targetClicks - encoder.get(), RobotMap.armPidLocality);    

            // add in bias and reduce the power to the allowed range
            // **** be safe for now until we get the settings right ***
            power = Functions.clip(feedForward + pidPower, -.22d, .22d);

            // is limit switch saying we are going too far?
            // if (limitSwitch.get() && 
            //         ((angle > 180 && power > 0)                  // opposite side too far 
            //         || (angle < 180 && power < 0))) {           // normal side too far
            //     power = 0;
            // }

            // set the power on the motors
            setMotors(power);
        }
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putBoolean("LimitSwitch.get()", limitSwitch.get());
        telemetry.putDouble("Angle", getAngle());
        telemetry.putDouble("PIDPower", pidPower);
        telemetry.putDouble("FeedForward", feedForward);
        telemetry.putDouble("Power", power);
        telemetry.putDouble("Clicks", encoder.get());
        telemetry.putDouble("TargetHeight", targetHeight);
        telemetry.putDouble("TargetAngle", targetAngle);
        telemetry.putDouble("TargetClicks", targetClicks);
        telemetry.putString("Version", "1.0.0");
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

        // TODO: Figure out angle where height is sine and then rotate it based on zero degrees for arm is straight down
        height = (targetHeight - RobotMap.armPivotHeight)/RobotMap.armLength; 
        angle = Math.toDegrees(Math.acos(height)) + ((facingNormal) ? 90 : 270);

        // // figure angle about pivot point given reach points in the vertical plane
        // if (facingNormal) {
        //     if (RobotMap.armPivotHeight >= targetHeight) { 
        //         // reaching low on start side - height is cos
        //         height = (RobotMap.armPivotHeight - targetHeight)/RobotMap.armLength;
        //         angle = Math.toDegrees(Math.acos(height));
        //     } else {
        //         // reaching high on start side - height is sin 
        //         height = (targetHeight - RobotMap.armPivotHeight)/RobotMap.armLength;
        //         angle = Math.toDegrees(Math.asin(height)) + 90;
        //     }
        // }
        // else {
        //     if (RobotMap.armPivotHeight >= targetHeight) { 
        //         // reaching low on opposite side - height is sin
        //         height = (RobotMap.armPivotHeight - targetHeight)/RobotMap.armLength;
        //         angle = Math.toDegrees(Math.asin(height)) + 270;
        //     } else {
        //         // reaching high on opposite side - height is cos
        //         height = (RobotMap.armPivotHeight - targetHeight)/RobotMap.armLength;
        //         angle = Math.toDegrees(Math.acos(height)) + 180;
        //     }
        // }

        return angle;
    }
}