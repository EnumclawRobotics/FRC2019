/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labEncoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;

import common.instrumentation.*;

/**
* Use to test out a 2 DIO channel encoder. 
* Turn encoder by hand to see SmartDashboard value change 
*/
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabEncoder");  

  private Encoder quadratureEncoder;

  @Override
  public void robotInit() {
    quadratureEncoder = new Encoder(0, 1);        // DIO ports
    quadratureEncoder.setDistancePerPulse(1);     // 1:1   ie count pulses
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
    telemetry.putDouble("Encoder Count", quadratureEncoder.getDistance());
    telemetry.putString("Version", "1.0.0");
  } 
}
