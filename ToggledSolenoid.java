/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ToggledSolenoid {
    DoubleSolenoid piston;
    boolean toggleBool = false;
    boolean button; 

    public ToggledSolenoid(int forward, int reverse) {
        piston = new DoubleSolenoid(forward, reverse);
    }

    public void set(boolean b) {
        toggleBool = !toggleBool;
        piston.set(b ? Value.kForward : Value.kReverse);
    }

    public void togglePiston() {
        toggleBool = !toggleBool;
        piston.set(toggleBool ? Value.kForward : Value.kReverse);
    }
}
