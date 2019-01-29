/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labMRColorSensor;

import edu.wpi.first.wpilibj.TimedRobot;

import common.instrumentation.*;
import common.util.Similarity;
import common.i2cSensors.*;

/**
* Use to test out a 2 DIO channel encoder. 
* Turn encoder by hand to see SmartDashboard value change 
*/
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabColorSensor");  

  MRColorSensor colorSensor;
  int[] cargoColor;
  double isMatchThreshold;

  @Override
  public void robotInit() {
    colorSensor = new MRColorSensor();        // Roborio i2c port
    cargoColor = colorSensor.createColor( 256, 256, 256, 256);    // fill real cargo values in RGBW order
    isMatchThreshold = .1d;
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
    telemetry.putDouble("Normalized Red", colorSensor.getRed());
    telemetry.putDouble("Normalized Green", colorSensor.getGreen());
    telemetry.putDouble("Normalized Blue", colorSensor.getBlue());
    telemetry.putDouble("Normalized White", colorSensor.getWhite());
    telemetry.putBoolean("IsMatch to CargoColor", Similarity.isMatch(colorSensor.getColor(), cargoColor, isMatchThreshold));
    telemetry.putDouble("IsMatch Threshold", isMatchThreshold);
    telemetry.putString("Version", "1.0.0");
  } 
}
