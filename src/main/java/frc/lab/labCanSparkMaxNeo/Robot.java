/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labCanSparkMaxNeo;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

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
  int deviceId = 3;
  CANEncoder quadratureEncoder;
  CANSparkMax motorController;
  Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
    joystick = new Joystick(0);                 // USB
  }

   /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    // if not teleop then allow user to cycle the CAN device id by clicking the trigger
    if (!this.isOperatorControl()) {
      if (joystick.getRawButton(1)) {
        if (Timer.getFPGATimestamp() >= debounceTarget) {
          debounceTarget = Timer.getFPGATimestamp() + .5;
          if (deviceId > 9) {
            deviceId = 1;
          } else {
            deviceId ++;
          }
        }
      }
    }
  
    putTelemetry();
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
    motorController.set(-joystick.getY());
    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putDouble("Device ID", deviceId);
    telemetry.putDouble("Joystick.getY", joystick.getY());
    if (quadratureEncoder != null) {
      telemetry.putDouble("Encoder Position", quadratureEncoder.getPosition());
      telemetry.putDouble("Encoder Velocity", quadratureEncoder.getVelocity());
    }
    telemetry.putString("Version", "1.0.1");
  }

}
