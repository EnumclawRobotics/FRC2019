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
//    private DigitalInput limitSwitch; 
    private PID pid;

    // involved parts
    private Wrist wrist;

    // set/derived through command
    private double targetAngle = 0;
    private double targetClicks = 0;

    // set at autonomous init
    private double baseClicks = 0;
    
    // derived as needed and stored for reporting
    private double feedForward = 0;
    private double pidPower = 0;
    private double power = 0;

    private States state = States.Stopped;

    private boolean isInited = false;

    public enum States {
        Stopped,
        MovingFront,
        MovingBack,
        MovingStowed,
        MovingRocketHatch1, MovingRocketHatch2, 
        MovingDepotCargo, 
        MovingStationCargo,
        MovingRocketCargo1, MovingRocketCargo2,  
        MovingShipCargo, 
        MovingManual,
        HoldingManual,
    }

    // Constructor that saves controller and sensor references
    public Arm(RobotMap robotMap) {
        // config motor controllers
        // Zero ange is straight down 
        // arm uses positive clicks to rotate from front over top to back end 
        //     and so needs to run in reverse direction from drive. 
        // the Drive inverts right side so we'll invert the Arm's Left side.
        robotMap.armLeftSpeedController.setIdleMode(IdleMode.kBrake);
        robotMap.armLeftSpeedController.setInverted(true);
        robotMap.armRightSpeedController.setIdleMode(IdleMode.kBrake);

        // keep references
        leftSpeedController = new RampSpeedController(robotMap.armLeftSpeedController, RobotMap.armRampFactor); 
        rightSpeedController = new RampSpeedController(robotMap.armRightSpeedController, RobotMap.armRampFactor); 
        encoder = robotMap.leftArmEncoder;
        encoder.setPositionConversionFactor(RobotMap.armEncoderConversionFactor);
//        limitSwitch = robotMap.armLimitSwitch;

        pid = new PID();
        pid.setGainsPID(RobotMap.armPidKp, RobotMap.armPidKi, RobotMap.armPidKd);

        stop();
    }

    // assumes that arm is Stowed at start of autonomous
    public void init(Wrist wrist) {
        if (!isInited) {
            // involved systems
            this.wrist = wrist;

            // set up baseline
            baseClicks = getClicks();

            // reset derived fields
            targetClicks = baseClicks;
            feedForward = 0;
            power = 0;

            // show that we are inited
            isInited = true;
        }
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

    // start side goes [0-180) while opposite goes [180-360)  
    public boolean getFacingNormal() {
        return (getAngle() < 180);
    }

    // encoder clicks
    public double getClicks() {
        return encoder.getPosition();
    }

    // moving into a storage position 
    public void moveStowed() {
        state = States.MovingStowed;
        targetAngle = getFacingNormal() ? RobotMap.armAngleStowed : (360 - RobotMap.armAngleStowed);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // moving to the front side
    public void moveFront() {
        state = States.MovingFront;
        targetAngle = RobotMap.armAngleRocketHatch2;
        targetClicks = clicksFromAngle(targetAngle);
    }

    // moving to the back side 
    public void moveBack() {
        state = States.MovingBack;
        targetAngle = (360 - RobotMap.armAngleRocketHatch2);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // rocket hatch 1
    public void moveRocketHatch1() {
        state = States.MovingRocketHatch1;
        targetAngle = getFacingNormal() ? RobotMap.armAngleRocketHatch1 : (360 - RobotMap.armAngleRocketHatch1);
        targetClicks = clicksFromAngle(targetAngle);
    }
    // rocket hatch 2
    public void moveRocketHatch2() {
        state = States.MovingRocketHatch2;
        targetAngle = getFacingNormal() ? RobotMap.armAngleRocketHatch2 : (360 - RobotMap.armAngleRocketHatch2);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // rocket cargo 1
    public void moveRocketCargo1() {
        state = States.MovingRocketCargo1;
        targetAngle = getFacingNormal() ? RobotMap.armAngleRocketCargo1 : (360 - RobotMap.armAngleRocketCargo1);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // rocket cargo 2
    public void moveRocketCargo2() {
        state = States.MovingRocketCargo2;
        targetAngle = getFacingNormal() ? RobotMap.armAngleRocketCargo2 : (360 - RobotMap.armAngleRocketCargo2);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // depot cargo
    public void moveDepotCargo() {
        state = States.MovingDepotCargo;
        targetAngle = getFacingNormal() ? RobotMap.armAngleDepotCargo : (360 - RobotMap.armAngleDepotCargo);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // station cargo
    public void moveStationCargo() {
        state = States.MovingStationCargo;
        targetAngle = getFacingNormal() ? RobotMap.armAngleStationCargo : (360 - RobotMap.armAngleStationCargo);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // ship cargo
    public void moveShipCargo() {
        state = States.MovingShipCargo;
        targetAngle = getFacingNormal() ? RobotMap.armAngleShipCargo : (360 - RobotMap.armAngleShipCargo);
        targetClicks = clicksFromAngle(targetAngle);
    }

    // move a bit based on controller intensity 
    public void moveManual(double controlPower) {
        // ignore deadband and defaults where we are not moving joystick
        if (Math.abs(controlPower) > .05d) {
            state = States.MovingManual;
            targetAngle = -1;
            targetClicks = getClicks() + (controlPower * RobotMap.armPidLocality);
        }
        // were we moving manual and now we are not? try holding
        else if (state == States.MovingManual) {
            state = States.HoldingManual;
            targetClicks = getClicks();
        }
    }

    public void run() {
        // arm not zeroed right? update zero position
        if (getClicks() < baseClicks) {
            baseClicks = getClicks();
        }

        if (state != States.Stopped) {
            // current angle implies how much force gravity applies and so what we need to make neutral
            double angle = getAngle();
            feedForward = Functions.gravity(angle) * RobotMap.armFeedForwardFactor;
            // TODO: Incorporate change in wrist angle 

            // get PID output that is best to go towards the target clicks
//            pidPower = pid.update(targetClicks - getClicks(), RobotMap.armPidLocality, RobotMap.armPowerLimit);    
            pidPower = pid.update(targetClicks - getClicks(), RobotMap.armPidLocality, .35d);    

            // add in bias and reduce the power to the allowed range
            // **** be safe for now until we get the settings right ***
            power = feedForward + pidPower;

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
//        telemetry.putBoolean("LimitSwitch.get()", limitSwitch.get());
        telemetry.putDouble("Angle", getAngle());
        telemetry.putDouble("PidKp", RobotMap.armPidKp);
        telemetry.putDouble("PIDPower", pidPower);
        telemetry.putDouble("FeedForward", feedForward);
        telemetry.putDouble("Power", power);
        telemetry.putDouble("Clicks", getClicks());
        telemetry.putDouble("TargetAngle", targetAngle);
        telemetry.putDouble("TargetClicks", targetClicks);
        telemetry.putString("Version", "1.0.0");
    }

    // === Helpers ===

    // given clicks figure out angle 
    private double angleFromClicks(double clicks) {
        return ((clicks - baseClicks) / RobotMap.armEncoderClicksPerDegree);
    }
    
    // given a target angle figure out target clicks
    private double clicksFromAngle(double angle) {
        return (angle * RobotMap.armEncoderClicksPerDegree) + baseClicks;
    }
}