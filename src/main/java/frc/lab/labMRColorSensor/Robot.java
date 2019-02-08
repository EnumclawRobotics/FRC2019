/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labMRColorSensor;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.FieldMap;
import common.instrumentation.*;
import common.i2cSensors.*;

/**
* Use to test out a 2 DIO channel encoder. 
* Turn encoder by hand to see SmartDashboard value change 
*/
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabMRColorSensor");  

  MRColorSensor colorSensor;
  int[] cargoColor;
  double isMatchThreshold;

  @Override
  public void robotInit() {
    colorSensor = new MRColorSensor();        // Roborio i2c port
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
    telemetry.putDouble("Color Number", colorSensor.getColorNumber());
    telemetry.putBoolean("IsMatch to CargoColor", Math.abs(colorSensor.getColorNumber() - FieldMap.cargoColorNumber) < FieldMap.cargoColorNumberVariance);
    telemetry.putDouble("IsMatch Threshold", isMatchThreshold);
    telemetry.putString("Version", "1.0.0");
  } 
}
