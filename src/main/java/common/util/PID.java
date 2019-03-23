package common.util;

import edu.wpi.first.wpilibj.*;

public class PID {
    double priorTime = 0;
    double priorError = 0;
    double integral = 0;
    double derivative = 0;

    double kP = 0;                  // .60Ku
    double kI = 0;                  // 1.2Ku/Tu
    double kD = 0;                  // 3KuTu/40
    
    double output = 0;

    public void setGainsP(double kP) {
        this.kP = kP;
        this.kI = 0;
        this.kD = 0;
    }

    public void setGainsPI(double kP, double kI) {
        this.kP = kP;
        this.kI = kI;
        this.kD = 0;
    }

    public void setGainsPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setZnGainsP(double kU) {
        this.kP = .50d * kU;
        this.kI = 0d;
        this.kD = 0d;
    }

    public void setZnGainsPI(double kU, double tU) {
        this.kP = .45d * kU;
        this.kI = .54d * kU/tU;
        this.kD = 0d;
    }

    public void setZnGainsPID(double kU, double tU) {
        this.kP = .60d * kU;
        this.kI = (1.2d * kU)/tU;
        this.kD = (3d * kU * tU) / 40d;
    }

    public void reset() {
        priorTime = 0;
        priorError = 0;
        integral = 0;
        derivative = 0;
    }

    // // updates PID controller based on target metric
    // public double update(double desired, double actual, double locality) { 
    //     return update(desired - actual, locality);
    // }

    // updates PID controller based on error variance in target metric - use locality to see if we are even getting close
    public double update(double currentError, double locality, double powerLimit) { 
        // reset derivative since its a one-time thing
        derivative = 0; 

        // setpoint far away? reset any integral and derivative 
        if (Math.abs(currentError) > locality) {
            reset();
            integral = 0;
            derivative = 0;
            currentError = Math.signum(currentError) * locality;
        } 
        // setpoint getting closer? we can start using integral and derivative
        else {
            // since last time
            double currentTime = Timer.getFPGATimestamp();
            double iterationTime = (priorTime == 0 ? 1d/60d : currentTime - priorTime); 

            // accumulate error 
            // reset to current if we just overshot in order to throw away any windup?
            // if (Math.signum(currentError) != Math.signum(priorError)) {
            //     integral = currentError;
            // }
            integral = integral + (currentError * iterationTime);

            // future estimate based on most recent performance
            derivative = (iterationTime != 0) ? (currentError - priorError)/iterationTime : 0;

            // update tracked values
            priorError = currentError;
            priorTime = currentTime;
        }

        // proportional + accumulated error - future expected as a brake)
        output = kP*currentError + kI*integral - kD*derivative;
        
        // restrict to limit
        output = Functions.clip(output, -powerLimit, powerLimit);

        return output;
    }

    public double getOutput() {
        return output;
    }
} 