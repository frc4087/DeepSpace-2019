/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.OI;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

  public OI m_oi;

  // Pneumatics
  Compressor c = new Compressor(0);
  public static ToggledSolenoid winchPiston = new ToggledSolenoid(2, 5);
  public static ToggledSolenoid shifters = new ToggledSolenoid(3, 4);
  public static DoubleSolenoid hatchIntake = new DoubleSolenoid(6, 7);
  public static ToggledSolenoid intakeActuator = new ToggledSolenoid(0, 1);

  //forward, reverse

  //public static DoubleSolenoid testPiston = new DoubleSolenoid(6,7);

  // Subsystems
  public static Spark leftWinch = new Spark(4);
  public static Spark rightWinch = new Spark(5);
  public static Winch winch = new Winch(leftWinch, rightWinch, winchPiston);
  public static VisionTracking m_visiontracking = new VisionTracking();

  // Drivebase
  public static WPI_TalonSRX LeftDrive1 = new WPI_TalonSRX(3);
  public static WPI_TalonSRX RightDrive1 = new WPI_TalonSRX(2);
  public static WPI_TalonSRX LeftDrive2 = new WPI_TalonSRX(4);
  public static WPI_TalonSRX RightDrive2 = new WPI_TalonSRX(1);
  public SpeedControllerGroup m_left = new SpeedControllerGroup(LeftDrive1, LeftDrive2);
  public SpeedControllerGroup m_right = new SpeedControllerGroup(RightDrive1, RightDrive2);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  public boolean shiftState;

  @Override
  public void robotInit() {
    m_oi = new OI();
    CameraServer.getInstance().startAutomaticCapture();
  }

  void matchPeriodic() {
    
    c.setClosedLoopControl(true);;
    
    if (m_oi.driveJoy.getXButton()) {
      m_visiontracking.setTracking(true);
      double steeringAdjust = .038 * m_visiontracking.pidX();
      m_drive.arcadeDrive(-m_oi.getDriveJoyYL() * m_visiontracking.get("tv"), steeringAdjust);
    } else {
      m_visiontracking.setTracking(false);
      m_drive.curvatureDrive(-m_oi.getDriveJoyYL(), m_oi.getDriveJoyXR(), m_oi.isQuickTurn());
    }

    if (m_oi.opJoy.getAButtonPressed()) {
      new winchDeploy();
    } else {
      winch.setWinch(-m_oi.getOpJoyYL());
    }

   
    if (Math.abs(m_oi.driveJoy.getTriggerAxis(Hand.kLeft)) > 0.1) {
      hatchIntake.set(Value.kForward);
    } else if (Math.abs(m_oi.driveJoy.getTriggerAxis(Hand.kRight)) > .1) {
      hatchIntake.set(Value.kReverse); 
    } else {
      hatchIntake.set(Value.kOff);
    }
    if (m_oi.getOpJoyBLPressed()) {
      intakeActuator.togglePiston();
    }
    if (m_oi.getDriveJoyBLPressed()) {
      shifters.togglePiston();
      shiftState = !shiftState;
    }
    if (m_oi.opJoy.getBButtonPressed()) {
      winchPiston.togglePiston();
    }
/*
    if (m_oi.getOpJoyBRPressed()) {
      testPiston.set(Value.kForward);
    } else if (m_oi.getOpJoyBLPressed()) {
      testPiston.set(Value.kReverse);
    } else {
      testPiston.set(Value.kOff);
    }*/

    SmartDashboard.putString("Gear", shiftState ? "Low" : "High");
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
    c.setClosedLoopControl(false);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    matchPeriodic();
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    matchPeriodic();
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }
}