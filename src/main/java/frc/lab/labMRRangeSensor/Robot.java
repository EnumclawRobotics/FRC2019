/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labMRRangeSensor;

import edu.wpi.first.wpilibj.TimedRobot;

import common.instrumentation.*;
import common.i2cSensors.*;

/**
* Use to test out a 2 DIO channel encoder. 
* Turn encoder by hand to see SmartDashboard value change 
*/
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabMrRangeSensor");  

  MRRangeSensor rangeSensor;

  @Override
  public void robotInit() {
    rangeSensor = new MRRangeSensor();        // Roborio i2c port
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
    telemetry.putDouble("Distance cm", rangeSensor.getDistance());
    telemetry.putString("Version", "1.0.0");
  } 
}
