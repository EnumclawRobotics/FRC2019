package frc.robot;

import common.oiHelpers.ToggleButton;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.Components.*;

public class Robot extends TimedRobot {
  public RobotMap robotMap;

  public Operator operator;
  public Drive drive;
  public Arm arm;
  public Wrist wrist;
  public Grabber grabber;

  @Override
  public void robotInit() {
    // === setup and cleanup ===

    // setup hardware specific stuff
    robotMap = new RobotMap();

    // setup logical subsystem components
    operator = new Operator(robotMap);
    drive = new Drive(robotMap);
    arm = new Arm(robotMap);
    wrist = new Wrist(robotMap, arm);
    grabber = new Grabber(robotMap);
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
    arm.init();
    wrist.init();
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
    arm.stop();
    wrist.stop();
    grabber.stop();
  }

  public void run() {
    // update components
    drive.setFacing(operator.driveFacing.toggleOn());
    drive.move(-operator.speedJoystick.getY(), operator.turnJoystick.getX(), false);

    // operator.xboxController.



    //arm.moveFloorCargo(facingNormal);
    //wrist.moveAligned();
    
    //arm.moveRocketCargo1(facingNormal);
    //wrist.moveHorizontal();
    //arm.moveRocketCargo2(facingNormal);
    //wrist.moveHorizontal();
    //arm.moveRocketCargo3(facingNormal);
    //wrist.moveHorizontal();

    //arm.moveRocketHatch1(facingNormal);
    //wrist.moveHorizontal();
    //arm.moveRocketHatch2(facingNormal);
    ///wrist.moveHorizontal();
    //arm.moveRocketHatch3(facingNormal);
    //wrist.moveHorizontal();

    //arm.moveStationHatch(facingNormal);
    //wrist.moveHorizontal();
    //arm.moveStatioCargo(facingNormal);
    //wrist.moveHorizontal();

    //arm.moveShipHatch(facingNormal);
    //wrist.moveHorizontal();
    //arm.moveShipCargo(facingNormal);
    //wrist.moveHorizontal();

    //arm.moveStationHatch(facingNormal);
    //wrist.moveFolded();

    // apply component changes
    drive.run();
    arm.run();
    wrist.run();
    grabber.run();

    putTelemetry();
  }

  private void putTelemetry() {
  }
}
