/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Spark;

public class Winch extends Subsystem {

  Spark leftWinch, rightWinch;
  ToggledSolenoid winchPiston;

  public Winch(Spark leftWinch_, Spark rightWinch_, ToggledSolenoid winchPiston_) {
    leftWinch = leftWinch_;
    rightWinch = rightWinch_;
    winchPiston = winchPiston_;
  }

  public void setWinch(double percentOutput) {
    leftWinch.set(-percentOutput);
    rightWinch.set(percentOutput);
  }

  public void togglePiston() {
    winchPiston.togglePiston();
  }

  @Override
  public void initDefaultCommand() {
  }
}
