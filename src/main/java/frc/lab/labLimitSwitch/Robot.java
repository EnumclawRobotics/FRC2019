/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labLimitSwitch;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import common.instrumentation.*;

/**
* Use to test out a 2 DIO channel encoder. 
* Turn encoder by hand to see SmartDashboard value change 
*/
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabLimitSwitch");  

  DigitalInput digitalInput;
  private Counter counter;

  @Override
  public void robotInit() {
    digitalInput = new DigitalInput(0);   // DIO ports
    counter = new Counter(0);             // DIO ports    // counter is a simple case of a forward only 1 port encoder
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    putTelemetry();
  }

  @Override
  public void teleopPeriodic() {
    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putBoolean("DigitalInput.get", digitalInput.get());
    telemetry.putDouble("Counter.get", counter.get());
    // telemetry.putDouble("Counter.getDistance", counter.getDistance());     // same as get for pulse=1
    telemetry.putString("Version", "1.0.0");
  } 
}
