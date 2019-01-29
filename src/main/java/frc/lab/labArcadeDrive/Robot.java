/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.lab.labArcadeDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import common.instrumentation.*;

/**
* Single joystick control of basic chassis using CAN, SparkMax and NEO motors 
*/
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabArcadeDrive");  

  private DifferentialDrive differentialDrive;
  private Joystick joystick;

  @Override
  public void robotInit() {
    joystick = new Joystick(0);                                                             // USB
    differentialDrive = new DifferentialDrive(new SpeedControllerGroup(new CANSparkMax(1, MotorType.kBrushless), 
                                                                        new CANSparkMax(2, MotorType.kBrushless)), 
                                              new SpeedControllerGroup(new CANSparkMax(3, MotorType.kBrushless), 
                                                                        new CANSparkMax(4, MotorType.kBrushless)));
      
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
    differentialDrive.arcadeDrive(-joystick.getY(), joystick.getX());
    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putDouble("Joystick.getY (Forward/Back)", joystick.getY());
    telemetry.putDouble("Joystick.getX (rotation)", joystick.getY());
    telemetry.putString("Version", "1.0.0");
  }
}
