package common.util;

import edu.wpi.first.wpilibj.*;

public class PID {
    double priorTime = 0;
    double priorError = 0;
    double integral = 0;

    double kP = 0;
    double kI = 0;
    double kD = 0;    

    double output = 0;

    // create PID with experimented constants
    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void reset() {
        priorTime = 0;
        priorError = 0;
        integral = 0;
    }

    // updates PID controller based on target metric
    public double update(double desired, double actual) { 
        // since last time
        double currentTime = Timer.getFPGATimestamp();
        double iterationTime = (priorTime == 0 ? .02 : currentTime - priorTime); 

        // accumulate error - reset to current if we just overshot in order to throw away any windup
        double currentError = desired - actual;
        // if (Math.signum(currentError) != Math.signum(priorError)) {
        //     integral = currentError;
        // }
        integral = integral + (currentError * iterationTime);
        
        // future estimate based on most recent performance
        double derivative = (iterationTime != 0) ? (currentError - priorError)/iterationTime : 0;

        // output based on experimented values - (proportional + accumulated error - future expected as a brake)
        output = kP*currentError + kI*integral - kD*derivative;
        
        // update tracked values
        priorError = currentError;
        priorTime = currentTime;

        return output;
    }

    public double getOutput() {
        return output;
    }
} 