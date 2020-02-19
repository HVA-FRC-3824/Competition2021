package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.LoadBallIntoChamber;

import java.lang.reflect.Array;
import java.sql.Struct;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase
{
  private static AddressableLED m_chamberLEDs;
  private static AddressableLEDBuffer m_LEDLength;
  private int m_rainbowFirstPixleHue;
  private boolean m_launcherMoving = false;


  private class class_color
  {
    public int H;
    public int S;
    public int V;

    public class_color(int h, int s, int v)
    {
      int H = h;
      int S = s;
      int V = v;
    }
  }
  public LEDs()
  {
      m_chamberLEDs = new AddressableLED(Constants.CHAMBER_LEDS_PORT);
<<<<<<< HEAD
      m_LEDBuffer = new AddressableLEDBuffer(Constants.CHAMBER_NUMBER_OF_LEDS);
=======
      m_LEDLength = new AddressableLEDBuffer(Constants.CHAMBER_NUMBER_OF_LEDS);
>>>>>>> 3976beb820707a43c5a99893ee6582bbccf6b6db

      m_chamberLEDs.setLength(m_LEDLength.getLength());
  
      // Set the data
      m_chamberLEDs.setData(m_LEDLength);
      m_chamberLEDs.start();
  }
  
  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    if (m_launcherMoving == true)
    {
      rainbow();
    }
    else if (LoadBallIntoChamber.getBallCount() > 0)
    {
      setLEDsForBallCount(LoadBallIntoChamber.getBallCount());
    }
    m_chamberLEDs.setData(m_LEDLength);  
  }

  // private void setColor()
  // {
  //  //Sets the color for each LED with Red, Green, and Blue input.
  //   for (var i = 0; i < m_LEDLength.getLength(); i++)
  //   {
  //       m_LEDLength.setRGB(i, 0, 0, 0);
  //   }

  //   // Assigns color to LEDs.
  //   m_chamberLEDs.setData(m_LEDLength);
  // }

  private void rainbow()
  {
    for (var i = 0; i < m_LEDLength.getLength(); i++)
    {
        final var hue = (m_rainbowFirstPixleHue + (i * 180 / m_LEDLength.getLength())) % 180;
        m_LEDLength.setHSV(i, hue, 225, 128);
    }

    m_rainbowFirstPixleHue += 3;

    m_rainbowFirstPixleHue %= 180;
  }

  public void setLEDstateLaunching(boolean launcherMoving)
  {
    m_launcherMoving = launcherMoving;
  }

  /**
   * Method to set the LED string for the give number off balls in the chamber
   */
  public void setLEDsForBallCount (int ballCount)
  {
    class_color [] ball_color = new class_color[5];

    // Initial the ball color array
    ball_color[0] = new class_color(0, 0, 0);    
    ball_color[1] = new class_color(0, 0, 0);    
    ball_color[2] = new class_color(0, 0, 0);    
    ball_color[3] = new class_color(0, 0, 0);    
    ball_color[4] = new class_color(0, 0, 0);

    // Turn off all LEDs
    for (var LED = 0; LED < m_LEDLength.getLength(); LED++)
    {
        m_LEDLength.setHSV(LED, 0, 0, 0);
    }

    // Set the LEDs for each ball
    for (var ball = 0; ball < ballCount; ball++)
    {
      // Loop over the LEDs for each ball
      for(var LED = ball * Constants.CHAMBER_NUMBER_OF_LEDS / 5; LED < (ball + 1) * Constants.CHAMBER_NUMBER_OF_LEDS / 5; LED++)
      {
        // Set the LED HSV value
        m_LEDLength.setHSV(LED, ball_color[ball].H, ball_color[ball].S, ball_color[ball].V);
      }
    }

    // Update the LED string
    m_chamberLEDs.setData(m_LEDLength);
  }
}