/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lab.labToggleButton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import common.instrumentation.*;
import common.oiHelpers.ToggleButton;

/**
 * Use for testing a Motor with a built in Encoder
 * Joystick forward and back should turn it
 */
public class Robot extends TimedRobot {
  Telemetry telemetry = new Telemetry("Robot/LabToggleButton");  

  ToggleButton toggleButton8;
  Joystick joystick;

  @Override
  public void robotInit() {
    joystick = new Joystick(0);                                     // USB
    toggleButton8 = new ToggleButton(joystick, 8, .5d);            // joystick, raw Button number, and debounce period
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
    if (toggleButton8.updateToggle()) {
      // toggled on
    }
    else {
      // toggled off
    }

    putTelemetry();
  }

  private void putTelemetry() {
    telemetry.putBoolean("Joystick.getRawButton8", joystick.getRawButton(8));
    telemetry.putBoolean("ToggleButton.toggleOn", toggleButton8.toggleOn());
    telemetry.putString("Version", "1.0.0");
  }
}
