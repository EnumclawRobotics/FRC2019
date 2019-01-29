/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labMotorEncoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;

import common.instrumentation.*;

/**
 * Use for testing a Motor with a built in Encoder
 * Joystick forward and back should turn it
 */
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabMotorEncoder");  

  Encoder quadratureEncoder;
  SpeedController motorController;
  Joystick joystick;

  @Override
  public void robotInit() {
    joystick = new Joystick(0);                 // USB
    motorController = new Spark(0);             // PWM port
    quadratureEncoder = new Encoder(0, 1);      // DIO ports
    quadratureEncoder.setDistancePerPulse(1);   // 1:1 so counting pulses
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
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    motorController.set(joystick.getY());
    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putDouble("Joystick.getY", joystick.getY());
    telemetry.putDouble("Encoder Count", quadratureEncoder.getDistance());
    telemetry.putString("Version", "1.0.0");
  }

}
