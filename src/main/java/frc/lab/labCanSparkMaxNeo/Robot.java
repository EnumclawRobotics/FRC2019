/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labCanSparkMaxNeo;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import common.instrumentation.*;

/**
 * Use for testing a Motor with a built in Encoder
 * Joystick forward and back should turn it
 */
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabMotorEncoder");  

  CANEncoder quadratureEncoder;
  CANSparkMax motorController;
  Joystick joystick;

  @Override
  public void robotInit() {
    joystick = new Joystick(0);                 // USB
    motorController = new CANSparkMax(1, MotorType.kBrushless);       // CAN device ID - Only needs to be unique within controller type
    quadratureEncoder = new CANEncoder(motorController);              // Encoder is through CAN too
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
    motorController.set(-joystick.getY());
    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putDouble("Joystick.getY", joystick.getY());
    telemetry.putDouble("Encoder Position", quadratureEncoder.getPosition());
    telemetry.putDouble("Encoder Velocity", quadratureEncoder.getVelocity());
    telemetry.putString("Version", "1.0.1");
  }

}
