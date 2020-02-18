package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.LoadBallIntoChamber;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase
{
<<<<<<< HEAD
//   private static AddressableLED m_chamberLEDs;
//   private static AddressableLEDBuffer m_LEDBuffer;
//   private static AddressableLED m_rainbowFirstPixleHue;
  public LEDs()
  {
    //   m_chamberLEDs = new AddressableLED(Constants.CHAMBER_LEDS_PORT);
    //   m_LEDBuffer = new AddressableLEDBuffer(Constants.CHAMBER_LED_BUFFER_LENGTH);
=======
  private static AddressableLED m_chamberLEDs;
  private static AddressableLEDBuffer m_LEDBuffer;
  private int m_rainbowFirstPixleHue;
  private boolean m_launcherMoving = false;

  public LEDs()
  {
      m_chamberLEDs = new AddressableLED(Constants.CHAMBER_LEDS_PORT);
      m_LEDBuffer = new AddressableLEDBuffer(Constants.CHAMBER_NUMBER_OF_LEDS);
>>>>>>> 09c7605149b6d338685b85ca854e891c7d4b2afe

    //   m_chamberLEDs.setLength(m_LEDBuffer.getLength());
  
    //   // Set the data
    //   m_chamberLEDs.setData(m_LEDBuffer);
    //   m_chamberLEDs.start();
  }
  
  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
<<<<<<< HEAD
  }

//   private void setColor()
//   {
//    //Sets the color for each LED with Red, Green, and Blue input.
//     for (var i = 0; i < m_LEDBuffer.getLength(); i++)
//     {
//         m_LEDBuffer.setRGB(i, 0, 0, 0);
//     }

//     // Assigns color to LEDs.
//     m_chamberLEDs.setData(m_LEDBuffer);
//   }

//   private void rainbow()
//   {
//     for (var i = 0; i < m_LEDBuffer.getLength(); i++)
//     {
//         final var hue = (m_rainbowFirstPixleHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
//         m_LEDBuffer.setHSV(i, hue, 225, 128);
//     }

//     m_rainbowFirstPixleHue += 3;

//     m_rainbowFirstPixleHue %= 180;
//   }
=======
    if (m_launcherMoving == true)
    {
      rainbow();
    }
    else if (LoadBallIntoChamber.getBallCount() > 0)
    {
      setLEDsForBallCount(LoadBallIntoChamber.getBallCount());
    }
    m_chamberLEDs.setData(m_LEDBuffer);  }

  // private void setColor()
  // {
  //  //Sets the color for each LED with Red, Green, and Blue input.
  //   for (var i = 0; i < m_LEDBuffer.getLength(); i++)
  //   {
  //       m_LEDBuffer.setRGB(i, 0, 0, 0);
  //   }

  //   // Assigns color to LEDs.
  //   m_chamberLEDs.setData(m_LEDBuffer);
  // }

  private void rainbow()
  {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++)
    {
        final var hue = (m_rainbowFirstPixleHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
        m_LEDBuffer.setHSV(i, hue, 225, 128);
    }

    m_rainbowFirstPixleHue += 3;

    m_rainbowFirstPixleHue %= 180;
  }

  public void setLEDstateLaunching(boolean launcherMoving)
  {
    m_launcherMoving = launcherMoving;
  }

  public void setLEDsForBallCount (int ballCount)
  {
    //turn off all LEDs
    for (var LED = 0; LED < m_LEDBuffer.getLength(); LED++)
    {
        m_LEDBuffer.setHSV(LED, 0, 0, 0);
    }
    //set LEDs per ball
    for (var ball = 0; ball < ballCount; ball++)
    {
      for(var LED = ball * Constants.CHAMBER_NUMBER_OF_LEDS / 5; LED < (ball + 1) * Constants.CHAMBER_NUMBER_OF_LEDS / 5; LED++)
      {

      }
    }

  }
>>>>>>> 09c7605149b6d338685b85ca854e891c7d4b2afe
}