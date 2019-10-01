/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class OI {
  //constants and initializations
  double[] previousTrigger = new double[4];
  public static final double JOY_DEADZONE = 0.1;
  boolean quickTurn = false;

  // Initialize joysticks
  public final XboxController driveJoy = new XboxController(0);
  public final XboxController opJoy = new XboxController(1);

/*-----------------------------------------------------------------------------*/

  //driveJoy axis values
  public double getDriveJoyXL() {
    double raw = driveJoy.getRawAxis(0);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

  public double getDriveJoyYL() {
    double raw = driveJoy.getRawAxis(1);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

  public double getDriveJoyXR() {
    double raw = driveJoy.getRawAxis(4);
    if (isQuickTurn()) {
      return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw > 0 ? (raw * raw) / 2 : (-raw * raw) / 2;
    } else {
      return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
    }
  }

  public boolean isQuickTurn() {
    if (getDriveJoyBRPressed()) {
      quickTurn = !quickTurn;
    }
    return quickTurn;
  }

  public double getDriveJoyYR() {
    double raw = driveJoy.getRawAxis(5);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }
  
/*-----------------------------------------------------------------------------*/

//opJoy axis values
  public double getOpJoyXL() {
    double raw = opJoy.getRawAxis(0);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

  public double getOpJoyYL() {
    double raw = opJoy.getRawAxis(1);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

  public double getOpJoyXR() {
    double raw = opJoy.getRawAxis(4);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

  public double getOpJoyYR() {
    double raw = opJoy.getRawAxis(5);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

/*------------------------------------------------------------------------------*/

//buttons, bumpers, and triggers
  public boolean getOpJoyBLPressed() {
    return opJoy.getBumperPressed(Hand.kLeft);
  }

  public boolean getOpJoyBRPressed() {
    return opJoy.getBumperPressed(Hand.kRight);
  }

  public boolean getDriveJoyBLPressed() {
    return driveJoy.getBumperPressed(Hand.kLeft);
  }

  public boolean getDriveJoyBRPressed() {
    return driveJoy.getBumperPressed(Hand.kRight);
  }

  /*------------------------------------------------------------------------*/

  boolean getDriveJoyLTPressed() {
    if (Math.abs(driveJoy.getTriggerAxis(Hand.kLeft)) > .1 && previousTrigger[0] == 0) {
      previousTrigger[0] = driveJoy.getTriggerAxis(Hand.kLeft);
      return true;
    }
    previousTrigger[0] = driveJoy.getTriggerAxis(Hand.kLeft);
    return false;
  }

  boolean getDriveJoyRTPressed() {
    if (Math.abs(driveJoy.getTriggerAxis(Hand.kRight)) > .1 && previousTrigger[1] == 0) {
      previousTrigger[1] = driveJoy.getTriggerAxis(Hand.kRight);
      return true;
    }
    previousTrigger[1] = driveJoy.getTriggerAxis(Hand.kRight);
    return false;
  }

  /*--------------------------------------------------------------------------*/

  boolean getOpJoyLTPressed() {
    if (Math.abs(opJoy.getTriggerAxis(Hand.kLeft)) > .1 && previousTrigger[2] == 0) {
      previousTrigger[2] = opJoy.getTriggerAxis(Hand.kLeft);
      return true;
    }
    previousTrigger[2] = opJoy.getTriggerAxis(Hand.kLeft);
    return false;
  }

  boolean getOpJoyRTPressed() {
    if (Math.abs(opJoy.getTriggerAxis(Hand.kRight)) > .1 && previousTrigger[3] == 0) {
      previousTrigger[3] = opJoy.getTriggerAxis(Hand.kRight);
      return true;
    }
    previousTrigger[3] = opJoy.getTriggerAxis(Hand.kRight);
    return false;
  }
}
