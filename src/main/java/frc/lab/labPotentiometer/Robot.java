/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labPotentiometer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import common.instrumentation.*;

/**
* Use to test out an analog potentiometer 
* Turn pot by hand to see Dashboard value change 
*/
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabPotentiometer");  
  AnalogPotentiometer potentiometer;

  @Override
  public void robotInit() {
    potentiometer = new AnalogPotentiometer(0, 2.8, 0);   // Analog port
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
  }

  private void putTelemetry() {
    telemetry.putDouble("Potentiometer Value", potentiometer.get());
    telemetry.putDouble("Potentiometer Scaled", potentiometer.get() * 100);
    telemetry.putString("Version", "1.0.0");
  } 
}
