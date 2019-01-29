/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labServo;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;

import common.instrumentation.*;

/**
 * Use for testing a Motor with a built in Encoder
 * Joystick forward and back should turn it
 */
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabServo");  

  Servo servo;
  Joystick joystick;

  @Override
  public void robotInit() {
    joystick = new Joystick(0);                 // USB
    servo = new Servo(0);                       // PWM port
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
    servo.set((joystick.getY() + 1)/2);       // translate joystick [-1,1] to [0,1]
    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putDouble("Joystick.getY", joystick.getY());
    telemetry.putDouble("Servo.getSpeed", servo.getSpeed());
    telemetry.putDouble("Servo.getPosition", servo.getPosition());
    telemetry.putDouble("Servo.getAngle", servo.getAngle());
    telemetry.putString("Version", "1.0.0");
  }
}
