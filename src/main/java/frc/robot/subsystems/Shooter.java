// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private SparkMax topShootMotor = new SparkMax(ShooterConstants.topShootMotorID, MotorType.kBrushless);
  private SparkMax bottomShootMotor = new SparkMax(ShooterConstants.bottomShootMotorID, MotorType.kBrushless);

  private SparkMaxConfig topShootMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig bottomShootMotorConfig = new SparkMaxConfig();

  /** Creates a new Shooter. */
  public Shooter() {
    topShootMotorConfig.idleMode(IdleMode.kBrake);
    bottomShootMotorConfig.idleMode(IdleMode.kBrake);

    topShootMotorConfig.smartCurrentLimit(ShooterConstants.currentLimit);
    bottomShootMotorConfig.smartCurrentLimit(ShooterConstants.currentLimit);

    bottomShootMotorConfig.follow(ShooterConstants.topShootMotorID);

    topShootMotor.configure(topShootMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomShootMotor.configure(bottomShootMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spinShooter(double speed) {
    topShootMotor.set(speed);
  }

  public boolean isReady() {
    return -topShootMotor.configAccessor.encoder
        .getVelocityConversionFactor() > ShooterConstants.targetFlywheelVelocity;
  }

  public void stopShooter() {
    topShootMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Flywheel vel", -topShootEncoder.getVelocity());
  }
}
