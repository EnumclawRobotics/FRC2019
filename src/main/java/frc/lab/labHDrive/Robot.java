/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.lab.labHDrive;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Components.CANVictorSPX;
import edu.wpi.first.wpilibj.VictorSP;

import common.instrumentation.*;

/**
* Single joystick control of basic chassis using CAN, SparkMax and NEO motors 
*/
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabHDrive");  

  private VictorSP frontStrafe;
  private VictorSP backStrafe;

  private DifferentialDrive differentialDrive;
  private XboxController xboxController;

  @Override
  public void robotInit() {
    xboxController = new XboxController(0);                                                             // USB
    differentialDrive = new DifferentialDrive(new SpeedControllerGroup(new CANVictorSPX(1), new CANVictorSPX(2)),
                                                new SpeedControllerGroup(new CANVictorSPX(3), new CANVictorSPX(4)));
    frontStrafe = new VictorSP(9); 
    backStrafe = new VictorSP(0);                                
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
    double forward = -xboxController.getY(Hand.kLeft);
    double turn = xboxController.getX(Hand.kRight);
    double strafe = xboxController.getX(Hand.kLeft);

    differentialDrive.arcadeDrive(forward, turn);

    frontStrafe.set(strafe + turn); 
    backStrafe.set(strafe - turn);

    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putDouble("Joystick.getY-Left (Forward / Back)", xboxController.getY(Hand.kLeft));
    telemetry.putDouble("Joystick.getX-Right (Turn Left / Turn Right)", xboxController.getX(Hand.kRight));
    telemetry.putDouble("Joystick.getX-Left (Strafe Left / Strafe Right)", xboxController.getX(Hand.kLeft));
    telemetry.putString("Version", "1.0.0");
  }
}
