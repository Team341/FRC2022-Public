// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class SetColor extends CommandBase {

  LED mLED;

  /**
   * Sets the color of the LED
   * @param led
   */
  public SetColor(LED led) {
    // Use addRequirements() here to declare subsystem dependencies.

    mLED = led;
    addRequirements(mLED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // mLED.getLED().animate(new RainbowAnimation(1.0, 0.5, 63));
    // mLED.getLED().animate(new FireAnimation(1.0, 0.3, 63, 0.2, 0.2));
    // mLED.getLED().animate(new ColorFlowAnimation(0, 0, 255));
    // mLED.getLED().animate(new LarsonAnimation(255, 0, 0, 0, 0.5, 63, BounceMode.Front, 3));
    // mLED.getLED().animate(new RgbFadeAnimation(1.0, 1.0, 63));
    // mLED.getLED().animate(new StrobeAnimation(0, 0, 255, 0, 0.8, 63));
    // mLED.getLED().animate(new TwinkleAnimation(0, 0, 255, 0, 0.8, 63, TwinklePercent.Percent42));
    mLED.getLED().animate(new TwinkleOffAnimation(0, 0, 255, 0, 0.8, 63, TwinkleOffPercent.Percent42));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
