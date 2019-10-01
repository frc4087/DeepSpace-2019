/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Drivebase extends Subsystem {
  
  private WPI_TalonSRX leftMotor;
  private WPI_TalonSRX rightMotor;
  private VictorSP topleftSlave;
  private VictorSP midleftSlave;
  private VictorSP toprightSlave;
  private VictorSP midrightSlave;

  DifferentialDrive m_drive;

  public Drivebase() {
   
    leftMotor = new WPI_TalonSRX(RobotMap.TAL_LEFT_DRIVE.value);
    rightMotor = new WPI_TalonSRX(RobotMap.TAL_RIGHT_DRIVE.value);
    topleftSlave = new VictorSP(RobotMap.VA_LEFT_DRIVE.value);
    toprightSlave = new VictorSP(RobotMap.VA_RIGHT_DRIVE.value);
    midleftSlave = new VictorSP(RobotMap.VB_LEFT_DRIVE.value);
    midrightSlave = new VictorSP(RobotMap.VB_RIGHT_DRIVE.value);

   // Robot.initTalon(leftMotor);
   // Robot.initTalon(rightMotor);

    SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor, midleftSlave, topleftSlave);
    SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor, midrightSlave, toprightSlave);

    m_drive = new DifferentialDrive(m_left, m_right);   
  }

  public void tankDrive(ControlMode mode, double leftValue, double rightValue) {

    leftMotor.set(mode, -leftValue);
    rightMotor.set(mode, rightValue);
  }
  
  public void arcadeDrive(double leftValue, double rightValue) {
    m_drive.arcadeDrive(leftValue, rightValue);
  }

  public void curvatureDrive(double leftValue, double rightValue, boolean button){
    m_drive.curvatureDrive(leftValue, rightValue, button);
  }


  @Override
  public void initDefaultCommand() {
  }
}
