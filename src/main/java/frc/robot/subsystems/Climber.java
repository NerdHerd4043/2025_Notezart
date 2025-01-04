// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climbMotor = new SparkMax(ClimberConstants.climberMotorID, MotorType.kBrushless);

  private SparkMaxConfig climbMotorConfig = new SparkMaxConfig();

  public Climber() {
    climbMotorConfig.idleMode(IdleMode.kBrake);

    climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runClimber(double motorSpeed) {
    climbMotor.set(motorSpeed);
  }

  public void stopClimber() {
    climbMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
