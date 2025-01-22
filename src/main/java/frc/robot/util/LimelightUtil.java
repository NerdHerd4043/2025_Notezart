// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class LimelightUtil {
    public static double getTargetsAngle() {
        return LimelightHelpers.getTX("limelight1");
    }

    public static double getTargetX() {
        double d = 5; // Need to find d using limelight
        double rar = 3.14 / 2; // Need to create array to refference
        return (rar);
    }
}
