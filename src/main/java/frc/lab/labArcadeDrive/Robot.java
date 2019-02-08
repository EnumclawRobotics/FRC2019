/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.lab.labArcadeDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Spark;

import common.instrumentation.*;

/**
* Single joystick control of basic chassis using CAN, SparkMax and NEO motors 
*/
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabArcadeDrive");  

  private DifferentialDrive differentialDrive;
  private XboxController xboxController;

  @Override
  public void robotInit() {
    xboxController = new XboxController(0);                                                             // USB
    differentialDrive = new DifferentialDrive(new Spark(0), new Spark(1));   
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
    differentialDrive.arcadeDrive(-xboxController.getY(Hand.kLeft), xboxController.getX(Hand.kRight));
    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putDouble("Joystick.getY (Forward/Back)", xboxController.getY(Hand.kLeft));
    telemetry.putDouble("Joystick.getX (rotation)", xboxController.getY(Hand.kRight));
    telemetry.putString("Version", "1.0.0");
  }
}
