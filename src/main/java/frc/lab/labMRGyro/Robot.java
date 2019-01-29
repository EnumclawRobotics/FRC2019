/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labMRGyro;

import edu.wpi.first.wpilibj.TimedRobot;
import common.i2cSensors.*;
import common.instrumentation.*;

/**
 * Use to Test a gyro. The X and Y are point values the Z is an integrated heading
 */
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabMrGyro");
  MRGyro gyro;

  @Override
  public void robotInit() {
    gyro = new MRGyro(0x10);            // RoboRio I2C port - 7bit address 
    gyro.calibrate();
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
    telemetry.putDouble("Gyro.X (Accelerometer)", gyro.getX());
    telemetry.putDouble("Gyro.Y (Accelerometer)", gyro.getY());
    telemetry.putDouble("Gyro.IntegratedZ", gyro.getIntegratedZ());
    telemetry.putString("Version", "1.0.0");
  }
}
