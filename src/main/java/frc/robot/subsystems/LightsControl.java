// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsControl extends SubsystemBase {

  private static AddressableLED led;
  // private static AddressableLED led2;

  private static int config = 0;
  private static int animFrame = 0;
  private static int animDir = 1;
  private static int animRunCount = 0;
  private static int animStage = 2;
  private static int skipFrame = 0;
  private static AddressableLEDBuffer ledBuffer;
  
  /** Creates a new LightsControl. */
  public LightsControl() {
    led = new AddressableLED(5);

    ledBuffer = new AddressableLEDBuffer(1*130);
    led.setLength(ledBuffer.getLength());
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 255, 0, 0);
     }

    // Set the data
    led.setData(ledBuffer);

    led.start();
  }

  public void setLightConfig(int configNum) {
    config = configNum;
    if (configNum == 0 || configNum == 4 || configNum == 3) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for black/off
        ledBuffer.setRGB(i, 0, 0, 0);
      }
    } else if (configNum == 1) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for orange
        ledBuffer.setRGB(i, 255, 191, 0);
      }  
    } else if (configNum == 2) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for purple
        ledBuffer.setRGB(i, 191, 64, 191);
      }
    }

    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (config == 0 || config == 4) {
      // Animation One: Up and Down
      if (animStage == 0) {

        // For every pixel
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setRGB(i, (int) (255.0 / ((animFrame / 4.0) + 1)), 0, 0);
        }

        animFrame += animDir;
        if (animFrame >= 100) {
          animFrame -= 1;
          animDir *= -1;
        } else if (animFrame < 0) {
          animFrame += 1;
          animDir *= -1;
        }

      }

      // Animation Two: Moving white sections on background
      else if (animStage == 1) {
        if (skipFrame < 2) {
          skipFrame += 1;
          return;
        }

        skipFrame = 0;
        
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          ledBuffer.setRGB(i, (config == 0 ? 255 : 0), 0, (config == 4 ? 255 : 0));
        }

        var i = animFrame % 12;
        while (i < ledBuffer.getLength())
        {
          ledBuffer.setRGB(i, 255, 255, 255);
          if (i + 1 < ledBuffer.getLength())
            ledBuffer.setRGB(i + 1, 255, 255, 255);
          i += 12;
        }

        animFrame += 1;
        if (animFrame >= 800) {
          animFrame = 0;
        }
      }

      // Animation Three trailing lights
      else if (animStage == 2) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          int r = getR(i);
          int g = getG(i);
          int b = getB(i);

          if (r > 15) r -= 15; else r = (config == 0 ? 15 : 0);
          if (g > 15) g -= 15; else g = 0;
          if (b > 15) b -= 15; else b = (config == 4 ? 15 : 0);
          ledBuffer.setRGB(i, r, g, b);
        }

        ledBuffer.setRGB(animFrame, (config == 0 ? 240 : 0), 0, (config == 4 ? 240 : 0));
        
        animFrame += animDir;
        if ((animFrame <= 0) || (animFrame >= ledBuffer.getLength() - 1)) {
          // Ensure valid even when switching modes
          if (animFrame < 0) animFrame = 0;
          if (animFrame > ledBuffer.getLength() - 1) animFrame = ledBuffer.getLength() - 1;
          animDir = -animDir;
        }
      }

      animRunCount += 1;
      if (animRunCount >= 800) {
        animRunCount = 0;
        animStage += 1;
        if (animStage > 2) {
          animStage = 0;
        }
      }

      led.setData(ledBuffer);
    }

  }

  private int getR(int i) {
    return (int) (ledBuffer.getLED(i).red * 255.0);
  }
  private int getG(int i) {
    return (int) (ledBuffer.getLED(i).green * 255.0);
  }
  private int getB(int i) {
    return (int) (ledBuffer.getLED(i).blue * 255.0);
  }
}