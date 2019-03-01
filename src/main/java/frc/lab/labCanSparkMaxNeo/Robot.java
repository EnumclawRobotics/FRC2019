/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labCanSparkMaxNeo;

import edu.wpi.first.wpilibj.XboxController;
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

  double debounceTarget = 0; 
  int deviceId = 1;
  CANEncoder quadratureEncoder;
  CANSparkMax motorController;
  XboxController xboxController = new XboxController(0);

  @Override
  public void robotInit() {
  }

   /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
    motorController = null;
    quadratureEncoder = null;
  }

  @Override
  public void disabledPeriodic() {
  }


  @Override
  public void teleopInit() {
    motorController = new CANSparkMax(deviceId, MotorType.kBrushless);        // CAN device ID - Only needs to be unique within controller type
    quadratureEncoder = new CANEncoder(motorController);                      // Encoder is through CAN too
  }

  @Override
  public void teleopPeriodic() {
    motorController.set(-xboxController.getY());
    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putDouble("Device ID", deviceId);
    telemetry.putDouble("Joystick.getY", -xboxController.getY());
    if (quadratureEncoder != null) {
      telemetry.putDouble("Encoder Position", quadratureEncoder.getPosition());
      telemetry.putDouble("Encoder Velocity", quadratureEncoder.getVelocity());
    }
    telemetry.putString("Version", "1.0.2");
  }

}
