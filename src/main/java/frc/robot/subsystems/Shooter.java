// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax topShootMotor = new CANSparkMax(ShooterConstants.topShootMotorID, MotorType.kBrushless);
  private CANSparkMax bottomShootMotor = new CANSparkMax(ShooterConstants.bottomShootMotorID, MotorType.kBrushless);

  private RelativeEncoder topShootEncoder = topShootMotor.getEncoder();

  /** Creates a new Shooter. */
  public Shooter() {
    topShootMotor.restoreFactoryDefaults();
    bottomShootMotor.restoreFactoryDefaults();

    topShootMotor.setIdleMode(IdleMode.kBrake);
    bottomShootMotor.setIdleMode(IdleMode.kBrake);

    topShootMotor.setSmartCurrentLimit(ShooterConstants.currentLimit);
    bottomShootMotor.setSmartCurrentLimit(ShooterConstants.currentLimit);

    bottomShootMotor.follow(topShootMotor);
  }

  public void spinShooter(double speed) {
    topShootMotor.set(speed);
  }

  public boolean isReady() {
    return -topShootEncoder.getVelocity() > ShooterConstants.targetFlywheelVelocity;
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
