// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  /** Creates a new RunIntake. */
  public final Intake intake;
  public final double intakeMotorSpeed;
  public final double kickupMotorSpeed;

  public RunIntake(Intake intake, double intakeMotorSpeed, double kickupMotorSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.intakeMotorSpeed = intakeMotorSpeed;
    this.kickupMotorSpeed = kickupMotorSpeed;

    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(intakeMotorSpeed, kickupMotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
