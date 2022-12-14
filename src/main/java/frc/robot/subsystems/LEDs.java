// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  
  public static class ConLED {
    public static int TIME_TO_CLIMB = 15;
    public static enum mode {
      OFF,
      COLONELS,
      KITT,
      VOLTAGE,
      CLIMBTIME,
      DISABLED,
      AUTONOMOUS,
      TELEOP,
    };
  }
      
  private ConLED.mode m_mode = ConLED.mode.OFF;
  
  // #ifdef ENABLE_LED
  int m_delay = 0;
  int m_currentPixel = 0;
  int m_blink = 1;
  // void Colonels(); // Blue & White
  // void Kitt(); // Cylon
  int m_kittDelta = 1;
  // void Voltage();
  // void Teleop();   // Blue or Red depending on alliance
  // void Disabled(); // RSL Orange
  // void ClimbTime(); // Flashing Green
  
  DriverStation.Alliance m_alliance;
  static int kLedLength = 13;
  static int kLedAPwmPort = 7;
  static int kLedBPwmPort = 9;
  // Must be a PWM header, not MXP or DIO
  AddressableLED m_ledA = new AddressableLED(kLedAPwmPort);
  //frc::AddressableLED m_ledB{kLedBPwmPort};
  // Both LED strips MUST Be the same length
  //std::array<frc::AddressableLED::LEDData, kLedLength> m_ledBuffer;  // Reuse the buffer
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(kLedLength);

  // #endif // ENABLE_LED

  public LEDs() {
    // #ifdef ENABLE_LED
      m_ledA.setLength(kLedLength);
      // m_ledB.SetLength(kLedLength);
      for (int i = 0; i < kLedLength; i++) {
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
      m_ledA.setData(m_ledBuffer);
      m_ledA.start();
      // m_ledB.SetData(m_ledBuffer);
      // m_ledB.Start();
    
    // #endif // ENABLE_LED
  }

  public void periodic() {
    // #ifdef ENABLE_LED
    if (m_mode != ConLED.mode.OFF) {
      switch (m_mode) {
        case COLONELS: 
          colonels();
          break;
        case KITT:
        case AUTONOMOUS:
          kitt();
          break;
        case VOLTAGE:
          voltage();
          break;
        case TELEOP:
          if (DriverStation.getMatchTime() >= ConLED.TIME_TO_CLIMB)
          // || frc::DriverStation::GetMatchType() == frc::DriverStation::MatchType::kNone)
          // Uncomment above during teleop practice to keep LEDs from flashing green
          {
            teleop();
          } else {
            climbTime();
          }
          break;
        case DISABLED:
          disabled();
          break;
        default:
          climbTime();
          break;
      }
    }
    // #endif // ENABLE_LED
  }
  
  public void init() {
    // #ifdef ENABLE_LED
    m_alliance = DriverStation.getAlliance();
    
    for (int i = 0; i < kLedLength; i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    m_ledA.setData(m_ledBuffer);
    // m_ledB.SetData(m_ledBuffer);
    
    // #endif // ENABLE_LED
  }
  
  public void on() {
    System.out.println("LED::On");
    switch (m_mode) {
      case COLONELS:
        colonels();
        break;
      case KITT:
      case AUTONOMOUS:
        kitt();
        break;
      case VOLTAGE:
        voltage();
        break;
      case TELEOP:
        teleop();
        break;
      case DISABLED:
        disabled();
        break;
      default:
        climbTime();
        break;
    }
  }

  public void setMode(ConLED.mode newMode) {
    m_mode = newMode;
  } 
  
  public void off() {
    System.out.println("LED::Off");
    m_mode = ConLED.mode.OFF;
    init();
  }
  
  /**
  * Add helper macros because the LEDs we use have the B and G channels swapped.
  * Instead of setting (R,G,B) the AddressableLED interface actually sets (R,B,G)
  * And, for the same reason the Hue channel of HSV goes around the circle counterclockwise
  * Reference: https://en.wikipedia.org/wiki/HSL_and_HSV
  * Let's change the API (only for code below) to something more intuitive
  */
  
  /**
  #define SetTrue_R_G_B(_r,_g,_b) SetRGB(_r, _b, _g)
  #define SetTrue_H360_S_V(_h,_s,_v) SetHSV((360-(_h))/2,_s,_v)  // A full 0-360 hue
  #define GetR r
  #define GetG b
  #define GetB g
  #define TurnOff SetRGB(0,0,0)
  */
  private void setTrue_R_G_B(AddressableLEDBuffer m_ledBuffer, int i, int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, b, g);
  }
  private void setTrue_H360_S_V(AddressableLEDBuffer m_ledBuffer, int i, int h, int s, int v) {
    m_ledBuffer.setHSV(i, (360-(h))/2, s, v);
  }
  private int getR(AddressableLEDBuffer m_ledBuffer, int i) {
    return (int) m_ledBuffer.getLED(i).red;
  }
  private int getG(AddressableLEDBuffer m_ledBuffer, int i) {
    return (int) m_ledBuffer.getLED(i).blue;
  }
  private int getB(AddressableLEDBuffer m_ledBuffer, int i) {
    return (int) m_ledBuffer.getLED(i).green;
  }
  private void turnOff(AddressableLEDBuffer m_ledBuffer, int i) {
    m_ledBuffer.setRGB(i, 0, 0, 0);
  }

  // #ifdef ENABLE_LED
  public void colonels() {
    if (--m_delay <= 0) {
      m_delay = 20;
      
      int colors[][] = {
        {210, 255, 128}, // blue
        {210, 255, 128}, // blue
        {210, 255, 128}, // blue
        {  0,   0,  64}, // white
        //{  6, 255, 128}, // gold
      };
      int ncolors = colors.length/colors[0].length;
      int ix;
      for (int i = 0; i < kLedLength; i++) {
        ix = (m_currentPixel + i) % ncolors;
        //m_ledBuffer[i].SetTrue_H360_S_V(colors[ix][0], colors[ix][1], colors[ix][2]);
        setTrue_H360_S_V(m_ledBuffer, i, colors[ix][0], colors[ix][1], colors[ix][2]);
      }
      m_ledA.setData(m_ledBuffer);
      // m_ledB.SetData(m_ledBuffer);
      m_currentPixel = (m_currentPixel + 1) % kLedLength;
    }
  }
  
  public void kitt() {
    if (--m_delay <= 0) {
      m_delay = 1;
      
      for (int i = 0; i < kLedLength; i++) {
        int r = getR(m_ledBuffer, i);
        int g = getG(m_ledBuffer, i);
        int b = getB(m_ledBuffer, i);
        if (r >= 10) r -= 10; else r = 0;
        if (g >= 20) g -= 20; else g = 0;
        if (b >= 20) b -= 20; else b = 0;
        //m_ledBuffer[i].SetTrue_R_G_B(r, g, b);
        setTrue_R_G_B(m_ledBuffer, i, r, g, b);
      }
      //m_ledBuffer[m_currentPixel].SetTrue_R_G_B(64, 64, 64);
      setTrue_R_G_B(m_ledBuffer, m_currentPixel, 64, 64, 64);
      m_ledA.setData(m_ledBuffer);
      // m_ledB.SetData(m_ledBuffer);
      
      m_currentPixel += m_kittDelta;
      if ((m_currentPixel <= 0) || (m_currentPixel >= kLedLength - 1)) {
        // Ensure valid even when switching modes
        if (m_currentPixel < 0) m_currentPixel = 0;
        if (m_currentPixel > kLedLength - 1) m_currentPixel = kLedLength - 1;
        m_kittDelta = -m_kittDelta;
      }
    }
  }
  
  public void voltage() {
    if (--m_delay <= 0) {
      m_delay = 30;
      double voltage = RobotController.getBatteryVoltage();
      double vmin = 9;
      double vmax = 12.5;
      int meter = (int) ((voltage - vmin) / (vmax - vmin) * (double) kLedLength);
      if (meter > kLedLength - 1) meter = kLedLength - 1;
      if (meter < 0) meter = 0;
      //printf("voltage: %f meter: %d\n", voltage, meter);
      
      for (int i = 0; i < kLedLength; i++) {
        if (i <= meter) {
          setTrue_R_G_B(m_ledBuffer, i, 0, 128, 0);
        } else {
          turnOff(m_ledBuffer, i);
        }
      }
      m_ledA.setData(m_ledBuffer);
      // m_ledB.SetData(m_ledBuffer);
    }
  }
  
  public void disabled() {
    if (--m_delay <= 0) {
      m_delay = 30;
      
      for (int i = 0; i < kLedLength; i++) {
        setTrue_R_G_B(m_ledBuffer, i, 255, 48, 0);
      }
      m_ledA.setData(m_ledBuffer);
      // m_ledB.SetData(m_ledBuffer);
    }
  }
  
  public void teleop() {
    if (--m_delay <= 0) {
      m_delay = 30;
      
      for (int i = 0; i < kLedLength; i++) {
        if (m_alliance == DriverStation.Alliance.Red) {
          setTrue_R_G_B(m_ledBuffer, i, 192, 0, 0);
        } 
        else if (m_alliance == DriverStation.Alliance.Blue) {
          setTrue_R_G_B(m_ledBuffer, i, 0, 0, 192);
        } 
        else {
          setTrue_R_G_B(m_ledBuffer, i, 0, 192, 0);
        }
      }
      m_ledA.setData(m_ledBuffer);
      // m_ledB.SetData(m_ledBuffer);
    }
  }

  public void climbTime() {
    if (--m_delay <= 0) {
      m_delay = 10;
      
      for (int i = 0; i < kLedLength; i++) {
        if (m_blink != 0) {
          turnOff(m_ledBuffer, i);
        } else {
          setTrue_R_G_B(m_ledBuffer, i, 0, 192, 0);
        }
      }
      m_ledA.setData(m_ledBuffer);
      // m_ledB.SetData(m_ledBuffer);
      m_blink = 1 - m_blink;
    }
  }
  //#endif // ENABLE_LED
  
}

/** Original H
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#ifdef ENABLE_LED
#include <frc/DriverStation.h>
#include <frc/AddressableLED.h>
#endif // ENABLE_LED

namespace ConLED {
  constexpr int TIME_TO_CLIMB = 15;
  enum mode {
    OFF,
    COLONELS,
    KITT,
    VOLTAGE,
    CLIMBTIME,
    DISABLED,
    AUTONOMOUS,
    TELEOP,
  };
}

class LEDs : public frc2::SubsystemBase {
 public:
  LEDs();
  void Periodic() override;
  void Init();
  void On();
  void Off();
  void SetMode(ConLED::mode newMode);

 private:
  ConLED::mode m_mode = ConLED::OFF;

#ifdef ENABLE_LED
  int m_delay = 0;
  int m_currentPixel = 0;
  int m_blink = 1;
  void Colonels(); // Blue & White
  void Kitt(); // Cylon
  int m_kittDelta = 1;
  void Voltage();
  void Teleop();   // Blue or Red depending on alliance
  void Disabled(); // RSL Orange
  void ClimbTime(); // Flashing Green

  frc::DriverStation::Alliance m_alliance;
  static constexpr int kLedLength = 13;
  static constexpr int kLedAPwmPort = 7;
  static constexpr int kLedBPwmPort = 9;
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_ledA{kLedAPwmPort};
  //frc::AddressableLED m_ledB{kLedBPwmPort};
  // Both LED strips MUST Be the same length
  std::array<frc::AddressableLED::LEDData, kLedLength> m_ledBuffer;  // Reuse the buffer
#endif // ENABLE_LED

};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDs.h"
#include <frc/DriverStation.h>

LEDs::LEDs() {
#ifdef ENABLE_LED
  m_ledA.SetLength(kLedLength);
  // m_ledB.SetLength(kLedLength);
  for (int i = 0; i < kLedLength; i++) {
    m_ledBuffer[i].SetRGB(0, 0, 0);
  }
  m_ledA.SetData(m_ledBuffer);
  m_ledA.Start();
  // m_ledB.SetData(m_ledBuffer);
  // m_ledB.Start();

#endif // ENABLE_LED
}

void LEDs::Periodic() {
#ifdef ENABLE_LED
  if (m_mode != ConLED::OFF) {
    switch (m_mode) {
      case ConLED::COLONELS:
        Colonels();
        break;
      case ConLED::KITT:
      case ConLED::AUTONOMOUS:
        Kitt();
        break;
      case ConLED::VOLTAGE:
        Voltage();
        
        break;
      case ConLED::TELEOP:
        if (frc::DriverStation::GetMatchTime() >= ConLED::TIME_TO_CLIMB)
            // || frc::DriverStation::GetMatchType() == frc::DriverStation::MatchType::kNone)
            // Uncomment above during teleop practice to keep LEDs from flashing green
          {
          Teleop();
        } else {
          ClimbTime();
        }
        break;
      case ConLED::DISABLED:
        Disabled();
        break;
      default:
        ClimbTime();
        break;
    }
  }
#endif // ENABLE_LED
}

void LEDs::Init() {
#ifdef ENABLE_LED
  m_alliance = frc::DriverStation::GetAlliance();

  for (int i = 0; i < kLedLength; i++) {
    m_ledBuffer[i].SetRGB(0, 0, 0);
  }
  m_ledA.SetData(m_ledBuffer);
  // m_ledB.SetData(m_ledBuffer);

#endif // ENABLE_LED
}

void LEDs::On() {
  printf("LED::On\n");
  switch (m_mode) {
    case ConLED::COLONELS:
      Colonels();
      break;
    case ConLED::KITT:
    case ConLED::AUTONOMOUS:
      Kitt();
      break;
    case ConLED::VOLTAGE:
      Voltage();
      break;
    case ConLED::TELEOP:
      Teleop();
      break;
    case ConLED::DISABLED:
      Disabled();
      break;
    default:
      ClimbTime();
      break;
  }
}
void LEDs::SetMode(ConLED::mode newMode) {
  m_mode = newMode;
} 

void LEDs::Off() {
  printf("LED::Off\n");
  m_mode = ConLED::OFF;
  Init();
}


 * Add helper macros because the LEDs we use have the B and G channels swapped.
 * Instead of setting (R,G,B) the AddressableLED interface actually sets (R,B,G)
 * And, for the same reason the Hue channel of HSV goes around the circle counterclockwise
 * Reference: https://en.wikipedia.org/wiki/HSL_and_HSV
 * Let's change the API (only for code below) to something more intuitive
 

#define SetTrue_R_G_B(_r,_g,_b) SetRGB(_r, _b, _g)
#define SetTrue_H360_S_V(_h,_s,_v) SetHSV((360-(_h))/2,_s,_v)  // A full 0-360 hue
#define GetR r
#define GetG b
#define GetB g
#define TurnOff SetRGB(0,0,0)

#ifdef ENABLE_LED
void LEDs::Colonels() {
  if (--m_delay <= 0) {
    m_delay = 20;

    int colors[][3] = {
      {210, 255, 128}, // blue
      {210, 255, 128}, // blue
      {210, 255, 128}, // blue
      {  0,   0,  64}, // white
      //{  6, 255, 128}, // gold
    };
    int ncolors = sizeof(colors)/sizeof(colors[0]);
    int ix;
    for (int i = 0; i < kLedLength; i++) {
      ix = (m_currentPixel + i) % ncolors;
      m_ledBuffer[i].SetTrue_H360_S_V(colors[ix][0], colors[ix][1], colors[ix][2]);
    }
    m_ledA.SetData(m_ledBuffer);
    // m_ledB.SetData(m_ledBuffer);
    m_currentPixel = (m_currentPixel + 1) % kLedLength;
  }
}

void LEDs::Kitt() {
  if (--m_delay <= 0) {
    m_delay = 1;

    for (int i = 0; i < kLedLength; i++) {
      int r = m_ledBuffer[i].GetR;
      int g = m_ledBuffer[i].GetG;
      int b = m_ledBuffer[i].GetB;
      if (r >= 10) r -= 10; else r = 0;
      if (g >= 20) g -= 20; else g = 0;
      if (b >= 20) b -= 20; else b = 0;
      m_ledBuffer[i].SetTrue_R_G_B(r, g, b);
    }
    m_ledBuffer[m_currentPixel].SetTrue_R_G_B(64, 64, 64);
    m_ledA.SetData(m_ledBuffer);
    // m_ledB.SetData(m_ledBuffer);

    m_currentPixel += m_kittDelta;
    if ((m_currentPixel <= 0) || (m_currentPixel >= kLedLength - 1)) {
      // Ensure valid even when switching modes
      if (m_currentPixel < 0) m_currentPixel = 0;
      if (m_currentPixel > kLedLength - 1) m_currentPixel = kLedLength - 1;
      m_kittDelta = -m_kittDelta;
    }
  }
}

void LEDs::Voltage() {
  if (--m_delay <= 0) {
    m_delay = 30;
    double voltage = frc::DriverStation::GetBatteryVoltage();
    constexpr double vmin = 9;
    constexpr double vmax = 12.5;
    int meter = (int) ((voltage - vmin) / (vmax - vmin) * (double) kLedLength);
    if (meter > kLedLength - 1) meter = kLedLength - 1;
    if (meter < 0) meter = 0;
    //printf("voltage: %f meter: %d\n", voltage, meter);

    for (int i = 0; i < kLedLength; i++) {
      if (i <= meter) {
        m_ledBuffer[i].SetTrue_R_G_B(0, 128, 0);
      } else {
        m_ledBuffer[i].TurnOff;
      }
    }
    m_ledA.SetData(m_ledBuffer);
    // m_ledB.SetData(m_ledBuffer);
  }
}

void LEDs::Disabled() {
  if (--m_delay <= 0) {
    m_delay = 30;

    for (int i = 0; i < kLedLength; i++) {
        m_ledBuffer[i].SetTrue_R_G_B(255, 48, 0);
    }
    m_ledA.SetData(m_ledBuffer);
    // m_ledB.SetData(m_ledBuffer);
  }
}

void LEDs::Teleop() {
  if (--m_delay <= 0) {
    m_delay = 30;

    for (int i = 0; i < kLedLength; i++) {
      if (m_alliance == frc::DriverStation::Alliance::kRed) {
        m_ledBuffer[i].SetTrue_R_G_B(192, 0, 0);
      } 
      else if (m_alliance == frc::DriverStation::Alliance::kBlue) {
        m_ledBuffer[i].SetTrue_R_G_B(0, 0, 192);
      } 
      else {
        m_ledBuffer[i].SetTrue_R_G_B(0, 192, 0);
      }
    }
    m_ledA.SetData(m_ledBuffer);
    // m_ledB.SetData(m_ledBuffer);
  }
}
void LEDs::ClimbTime() {
  if (--m_delay <= 0) {
    m_delay = 10;

      for (int i = 0; i < kLedLength; i++) {
        if (m_blink) {
          m_ledBuffer[i].TurnOff;
        } else {
          m_ledBuffer[i].SetTrue_R_G_B(0, 192, 0);
        }
      }
    m_ledA.SetData(m_ledBuffer);
    // m_ledB.SetData(m_ledBuffer);
    m_blink = 1 - m_blink;
  }
}
#endif // ENABLE_LED

 */