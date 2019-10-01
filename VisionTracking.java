package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class VisionTracking extends Command {

    NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double tv = get("tv");
    double previousError, previousZoom = 0;
    //public double tar;

    public double get(String var) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(var).getDouble(0.0);
    }

    public VisionTracking() {
    }

    public void setTracking(boolean tracking) {
        m_limelightTable.getEntry("camMode").setNumber(0);
        m_limelightTable.getEntry("ledMode").setNumber(0);
    }

    protected void initialize() {
    }

    protected void execute() {
    }

    public double pidX() {
        double min = 0.05;
        double kP = -0.04;// -0.038;
        double tx = get("tx");
        double error = -tx;

        if (tx > 0.0) {
            return kP * error - min;
        } else {
            return kP * error + min;
        }
    }

    public double getTarget() {
        double _minhor = 38, _minvert = 17;
        double  _maxhor = 195, _maxvert = 65;
        double whratio = get("thor") / get("tvert");
        double minratio = _minhor / _minvert;
        double maxratio = _maxhor / _maxvert;

        SmartDashboard.putNumber("minratio", minratio);
        SmartDashboard.putNumber("maxratio", maxratio);

        if (get("tv") == 1) {
            if (minratio < whratio && whratio < maxratio) {
                return 1;
            } else {
                return 0; 
            }
        } else {
            return 0;
        }
    }

    public double zoomForward() {
        if (getTarget() == 0) {
            return previousZoom;
        } else {
            double lmin = .5;
            double dkP = 0.1;
            double currentta = get("ta");
            double idealta = 7;
            double error = idealta - currentta;
            return previousZoom = Math.max(dkP * error, lmin);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    protected void interrupted() {
        end();
    }
}/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
