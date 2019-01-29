/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labADXRSGyro;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import common.instrumentation.*;

/**
 * Use to Test a gyro. The X and Y are point values the Z is an integrated heading
 */
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabADXRSGyro");
  ADXRS450_Gyro gyro;

  @Override
  public void robotInit() {
    gyro = new ADXRS450_Gyro();             // RoboRio spi port - default address 
    gyro.calibrate();                       // Robot should remain still for 10 seconds 
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    putTelemetry();

    // TODO: perhaps add something here that decides if recalibrate is needed? Did we move? Can we save a calibration and just use that?
  }

  @Override
  public void teleopPeriodic() {
    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putDouble("Gyro.getAngle (heading)", gyro.getAngle());
    telemetry.putString("Version", "1.0.0");
  }
}
