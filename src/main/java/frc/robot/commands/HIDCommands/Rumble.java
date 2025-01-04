// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HIDCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class Rumble extends Command {
  private final XboxController driveStick;
  private final DigitalInput beamBreak;
  private final BooleanSupplier shooterReady;

  // TODO: Switch to BooleanSupplier instead of taking the beamBreak directly
  /** Creates a new Rumble. */
  public Rumble(XboxController driveStick, DigitalInput beamBreak, BooleanSupplier shooterReady) {
    this.driveStick = driveStick;
    this.beamBreak = beamBreak;
    this.shooterReady = shooterReady;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Can this be simplified?
    if (beamBreak.get() || shooterReady.getAsBoolean()) {
      driveStick.setRumble(RumbleType.kBothRumble, 0);
    } else {
      driveStick.setRumble(RumbleType.kBothRumble, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveStick.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
