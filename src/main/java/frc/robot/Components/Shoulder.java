package frc.robot.Components;

import com.revrobotics.*;
import common.instrumentation.Telemetry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import frc.robot.HardwareMap;

/**
* Shoulder for holding up arm
*  DESIGNED TO ONLY APPLY CHANGES IN RUN()
*/
public class Shoulder {
    // -- setup and cleanup ===
    Telemetry telemetry = new Telemetry("Robot/Drive");

    DifferentialDrive shoulder;             // NOTE: we will never run this differentially. Using it for default safety enabled             
    CANEncoder shoulderEncoder; 
    DigitalInput shoulderLimitSwitch;
    
    double baseLineAngle = 0;            
    double targetAngle = 0;
    double feedForward = 0;
    double speed = 0;

    boolean facingNormal = true;

    States state = States.Stopped;

    public enum States {
        Stopped, 
        MovingCargoFloor, MovingCargo1, MovingCargo2, MovingCargo3, MovingCargoShip, MovingHatch1, MovingHatch2, MovingHatch3,
        Holding
    }

    public double[] Heights;


    public States getState() {
        return state;
    }
    
    public Shoulder(HardwareMap hardwareMap) {
        shoulder = new DifferentialDrive(hardwareMap.leftShoulderSpeedController, 
                                            hardwareMap.rightShoulderSpeedController);
        shoulder.setExpiration(hardwareMap.safetyExpiration);
        shoulder.setSafetyEnabled(true);
        shoulder.setRightSideInverted(true);                // ensure that both motors work in the same direction

        shoulderEncoder = hardwareMap.shoulderEncoder;
        shoulderLimitSwitch = hardwareMap.shoulderLimitSwitch;

        stop();
    }

    // === Executing per Period ===
    public double getCurrentAngle() {
        return shoulderEncoder.getPosition();       // TODO: Confirm what getPosition() returns
    }

    public void setFacing(boolean facingNormal) {
        this.facingNormal = facingNormal;
    }

    public void run() {
        // always ensure that we never use rotation. i.e. both shoulder motors should move as one not try to fight each other
        shoulder.arcadeDrive(speed, 0, false);
        putTelemetry();
    }

    private void putTelemetry() {
        telemetry.putString("State", state.toString());
        telemetry.putDouble("CurrentAngle", getCurrentAngle());
        telemetry.putDouble("Angle", targetAngle);
        telemetry.putDouble("FeedForward", feedForward);
        telemetry.putDouble("Speed (forward|back)", speed);
        telemetry.putString("Version", "1.0.0");
    }

    // === User Trigerrable States ===
    public void stop() {
        state = States.Stopped;
        speed = 0;
    }

    // whenever we arent moving use PID controller to hold at desired height
    public void hold() {
        state = States.Holding;

        feedForward = Math.cos(targetAngle);        // power is a function of angle given everything else. degrees? clicks? radians?
    }

    // === Internally Trigerrable States ===

}