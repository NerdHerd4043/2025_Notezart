// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map;

/** Add your docs here. */
public class LimelightUtil {
    public double offset;
    public double ID;

    private Map<Double, OffsetRatios> IDRatios = Map.of(
            3.0, new OffsetRatios(1.0, 3.0)); // Imagine there are more items in the map

    public double getTargetX(double offset) {
        this.offset = offset;
        this.ID = LimelightHelpers.getFiducialID("limelight1");

        final double xRatio = this.IDRatios.get(this.ID).xRatio;

        return (this.offset * xRatio);
    }

    public double getTargetY(double offset) {
        this.offset = offset;
        this.ID = LimelightHelpers.getFiducialID("limelight1");

        final double yRatio = this.IDRatios.get(this.ID).yRatio;

        return (this.offset * yRatio);
    }
}
