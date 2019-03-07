/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labXboxController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;

import common.instrumentation.*;

/**
 * Use for testing a Motor with a built in Encoder
 * Joystick forward and back should turn it
 */
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabXboxController");  

  XboxController xboxController;

  @Override
  public void robotInit() {
    xboxController = new XboxController(0);                                     // USB
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
  }

  private void putTelemetry() {
    telemetry.putDouble("Left Y", xboxController.getY(Hand.kLeft));
    telemetry.putDouble("Left X", xboxController.getX(Hand.kLeft));
    telemetry.putBoolean("Left StickButton", xboxController.getStickButton(Hand.kLeft));
    telemetry.putBoolean("Left Bumper", xboxController.getBumper(Hand.kLeft));
    telemetry.putDouble("Left Trigger", xboxController.getTriggerAxis(Hand.kLeft));

    telemetry.putDouble("Right Y", xboxController.getY(Hand.kRight));
    telemetry.putDouble("Right X", xboxController.getX(Hand.kRight));
    telemetry.putBoolean("Right StickButton", xboxController.getStickButton(Hand.kRight));
    telemetry.putBoolean("Right Bumper", xboxController.getBumper(Hand.kRight));
    telemetry.putDouble("Right Trigger", xboxController.getTriggerAxis(Hand.kRight));

    telemetry.putDouble("POV Hat", xboxController.getPOV());

    telemetry.putBoolean("A Button", xboxController.getAButton());
    telemetry.putBoolean("B Button", xboxController.getBButton());
    telemetry.putBoolean("X Button", xboxController.getXButton());
    telemetry.putBoolean("Y Button", xboxController.getYButton());

    telemetry.putBoolean("Back Button", xboxController.getBackButton());
    telemetry.putBoolean("Start Button", xboxController.getStartButton());

    telemetry.putString("Version", "1.0.0");
  }
}
