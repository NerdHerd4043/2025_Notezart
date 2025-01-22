// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CANdleConstants;

// import java.util.function.BooleanSupplier;

// import com.ctre.phoenix.led.Animation;
// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdleConfiguration;
// import com.ctre.phoenix.led.ColorFlowAnimation;
// import com.ctre.phoenix.led.StrobeAnimation;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

public class CANdleSystem extends SubsystemBase {
  // private final CANdle candle = new CANdle(CANdleConstants.id, "rio");

  public enum AnimationType {
    // ColorFlow,
    // Flash,
  }

  /*
   * These three values store our solid color data in the event we lack an
   * animation
   */
  // private int r, g, b;

  /*
   * This variable is the enum representation of our currently selected
   * annotation. It will be null if no animation is selected.
   */
  // private AnimationType currentAnimation;
  /*
   * This stores the actual animation object from the CANdle API
   */
  // private Animation toAnimate;

  /** Creates a new CANdle. */
  public CANdleSystem() {
    // CANdleConfiguration config = new CANdleConfiguration();

    // config.statusLedOffWhenActive = true;
    // config.disableWhenLOS = true;
    // config.stripType = LEDStripType.GRB;
    // config.brightnessScalar = 0.8;
    // config.vBatOutputMode = VBatOutputMode.On;

    // setBlue();

    // candle.configAllSettings(config);
  }

  // public AnimationType getCurrentAnimation() {
  // return currentAnimation;
  // }

  public void setColors(int r, int g, int b) {
    // this.r = r;
    // this.g = g;
    // this.b = b;
  }

  public void setOrange() {
    // setColors(255, 25, 0);
    // changeAnimation(null);
  }

  public void setBlue() {
    // setColors(0, 0, 255);
    // changeAnimation(null);
  }

  public void setGreen() {
    // setColors(0, 255, 0);
    // changeAnimation(null);
  }

  // public void setFlashing() {
  // changeAnimation(AnimationType.Flash);
  // }

  // public Command getDefaultCommand(BooleanSupplier shooterReady,
  // BooleanSupplier hasNote) {
  // return this.run(() -> {
  // if (!hasNote.getAsBoolean()) {
  // this.setBlue();
  // } else if (shooterReady.getAsBoolean()) {
  // this.setGreen();
  // } else {
  // this.setOrange();
  // }
  // }).ignoringDisable(true);
  // }

  // public void changeAnimation(AnimationType toChange) {
  // currentAnimation = toChange;

  // if (currentAnimation != null) {
  // toAnimate = switch (currentAnimation) {
  // case ColorFlow -> new ColorFlowAnimation(128, 20, 70, 0, 0.7,
  // CANdleConstants.ledCount, Direction.Forward, 8);
  // case Flash -> new StrobeAnimation(255, 0, 0);
  // };
  // }
  // }

  // public void ledsOff() {
  // setColors(0, 0, 0);
  // changeAnimation(null);
  // }

  @Override
  public void periodic() {
    // // INFO: If problems occur, switch this back to toAnimate
    // if (currentAnimation != null) {
    // candle.animate(toAnimate);
    // } else {
    // candle.setLEDs(r, g, b);
    // }
  }
}
