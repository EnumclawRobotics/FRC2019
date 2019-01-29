package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Components.*;

public class Robot extends TimedRobot {
  public HardwareMap hardwareMap;

  public Operator operator;
  public Drive drive;
  public Shoulder shoulder;
  public Wrist wrist;
  public Grabber grabber;
  
  @Override
  public void robotInit() {
    // === setup and cleanup ===

    // setup hardware specific stuff
    hardwareMap = new HardwareMap();

    // setup logical subsystem components
    operator = new Operator(hardwareMap);
    drive = new Drive(hardwareMap);
    shoulder = new Shoulder(hardwareMap);
    wrist = new Wrist(hardwareMap);
    grabber = new Grabber(hardwareMap);
  }

  // === Modes ===

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    putTelemetry();
  }

  @Override
  public void disabledInit() {
    stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    run();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    run();
  }

  @Override
  public void testPeriodic() {
  }

  // === Executing per Period ===

  public void stop() {
    drive.stop();
    shoulder.stop();
    wrist.stop();
    grabber.stop();
  }

  public void run() {
    drive.run();
    shoulder.run();
    wrist.run();
    grabber.run();

    putTelemetry();
  }

  private void putTelemetry() {
  }
}
