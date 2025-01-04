// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax climbMotor = new CANSparkMax(ClimberConstants.climberMotorID, MotorType.kBrushless);

  public Climber() {
    climbMotor.restoreFactoryDefaults();

    climbMotor.setIdleMode(IdleMode.kBrake);
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
