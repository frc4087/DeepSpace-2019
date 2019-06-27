/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.winch_commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

public class addSlack extends TimedCommand {

  public addSlack(double timeout) {
    super(timeout);
    requires(Robot.winch);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.winch.setWinch(-1);
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
