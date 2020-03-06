package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDs extends SubsystemBase
{
  private static AddressableLED m_chamberLEDs;
  private static AddressableLEDBuffer m_LEDLength;

  private DriverStation m_ds = DriverStation.getInstance();

  private boolean m_defenseLEDs = false;
  private boolean m_isDefending = false;

  /* Neutral Sequence */
  private int m_neutralStepValue = 0; // step G for blue, B for red, step R for purple
  private int m_neutralPixelToChange = 0;
  private boolean m_neutralChasingDirection = false; // false: inwards - true: outwards

  /* Launching Sequence */
  private int m_rainbowFirstPixleHue;
  private boolean m_isLaunching = false;

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
      m_LEDLength = new AddressableLEDBuffer(Constants.CHAMBER_TOTAL_NUM_OF_LEDS);

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
    if (m_isLaunching)
    {
      this.rainbow();
    }
    else if (m_isDefending)
    {
      this.defenseModeLEDs();
    }
    else
    {
      this.neutral();
    }
    // else if (LoadBallIntoChamber.getBallCount() > 0)
    // {
    //   setLEDsForBallCount(LoadBallIntoChamber.getBallCount());
    // }
    m_chamberLEDs.setData(m_LEDLength);
  }

  public void resetSequences()
  {
    m_isLaunching = false;
  }

  // private void setColor(int r, int g, int b)
  // {
  //  //Sets the color for each LED with Red, Green, and Blue input.
  //   for (var i = 0; i < m_LEDLength.getLength(); i++)
  //   {
  //       m_LEDLength.setRGB(i, 0, 0, 0);
  //   }

  //   // Assigns color to LEDs.
  //   m_chamberLEDs.setData(m_LEDLength);
  // }

  private void neutral()
  {
    // side LEDs
    if (m_neutralChasingDirection)
    {
      this.chaseOutward();
    }
    else if (!m_neutralChasingDirection)
    {
      this.chaseInward();
    }

    // top LEDs
    for (var i = Constants.CHAMBER_SIDE_NUM_OF_LEDS; i < Constants.CHAMBER_SIDE_NUM_OF_LEDS + Constants.CHAMBER_TOP_NUM_OF_LEDS; i++)
    {
      if (m_ds.getAlliance() == DriverStation.Alliance.Blue)
      {
        m_LEDLength.setRGB(i, 0, 0, 255); // blue
      }
      else if (m_ds.getAlliance() == DriverStation.Alliance.Red)
      {
        m_LEDLength.setRGB(i, 255, 0, 0); // red
      }
      else
      {
        m_LEDLength.setRGB(i, 148, 0, 211); // purple
      }
    }
  }

  private void chaseInward()
  {
    for (var i = 0; i < Constants.CHAMBER_SIDE_NUM_OF_LEDS / 2; i++)
    {
      if (i == m_neutralPixelToChange)
      {
        // Initial Side
        m_LEDLength.setRGB(i, 0, m_neutralStepValue, 255); // blue
        m_LEDLength.setRGB((Constants.CHAMBER_SIDE_NUM_OF_LEDS-1) - i, 0, m_neutralStepValue, 255); // blue

        // Other Side
        m_LEDLength.setRGB((Constants.CHAMBER_TOTAL_NUM_OF_LEDS-1) - i, 0, m_neutralStepValue, 255); // blue
        m_LEDLength.setRGB((Constants.CHAMBER_SIDE_NUM_OF_LEDS + Constants.CHAMBER_TOP_NUM_OF_LEDS) + i, 0, m_neutralStepValue, 255); // blue
      }
    }

    if (m_neutralPixelToChange < (Constants.CHAMBER_SIDE_NUM_OF_LEDS / 2) - 1)
    {
      m_neutralPixelToChange++;
    }
    else
    {
      m_neutralChasingDirection = true; // chase outwards
      m_neutralStepValue = 255;
    }
  }

  private void chaseOutward()
  {
    for (var i = (Constants.CHAMBER_SIDE_NUM_OF_LEDS / 2) - 1; i >= 0; i--)
    {
      if (i == m_neutralPixelToChange)
      {
        // Initial Side
        m_LEDLength.setRGB(i, 0, m_neutralStepValue, 255); // blue
        m_LEDLength.setRGB((Constants.CHAMBER_SIDE_NUM_OF_LEDS-1) - i, 0, m_neutralStepValue, 255); // blue

        // Other Side
        m_LEDLength.setRGB((Constants.CHAMBER_TOTAL_NUM_OF_LEDS-1) - i, 0, m_neutralStepValue, 255); // blue
        m_LEDLength.setRGB((Constants.CHAMBER_SIDE_NUM_OF_LEDS + Constants.CHAMBER_TOP_NUM_OF_LEDS) + i, 0, m_neutralStepValue, 255); // blue
      }
    }

    if (m_neutralPixelToChange > 0)
    {
      m_neutralPixelToChange--;
    }
    else
    {
      m_neutralChasingDirection = false; // chase inwards
      m_neutralStepValue = 0;
    }
  }

  private void rainbow()
  {
    for (var i = 0; i < Constants.CHAMBER_SIDE_NUM_OF_LEDS; i++)
    {
      final var hue = (m_rainbowFirstPixleHue + (i * 180 / Constants.CHAMBER_SIDE_NUM_OF_LEDS)) % 180;
      m_LEDLength.setHSV(i, hue, 225, 255/*128*/);
      m_LEDLength.setHSV((Constants.CHAMBER_TOTAL_NUM_OF_LEDS-1) - i, hue, 255, 255/*128*/);
    }

    for (var i = Constants.CHAMBER_SIDE_NUM_OF_LEDS; i < Constants.CHAMBER_SIDE_NUM_OF_LEDS + Constants.CHAMBER_TOP_NUM_OF_LEDS; i++)
    {
      if (RobotContainer.m_launcher.getLaunchReadyStatus())
      {
        m_LEDLength.setRGB(i, 0, 255, 0);
      }
      else
      {
        m_LEDLength.setRGB(i, 255, 0, 0);
      }
    }

    m_rainbowFirstPixleHue += 3;

    m_rainbowFirstPixleHue %= 180;
  }

  public void setIsDefending(boolean status)
  {
    m_isDefending = status;
  }

  public void defenseModeLEDs()
  {
    if (m_defenseLEDs == false)
    {
      for (var i = 0; i < Constants.CHAMBER_TOTAL_NUM_OF_LEDS; i++)
      {
        m_LEDLength.setRGB(i, 255, 0, 0);
      }
      m_defenseLEDs = true;
    }
    else 
    {
      for (var i = 0; i < Constants.CHAMBER_TOTAL_NUM_OF_LEDS; i++)
      {
        m_LEDLength.setRGB(i, 0, 0, 0);
      }
      m_defenseLEDs = false;
    }
    new WaitCommand(1);
  }

  public void setLaunchingStatus(boolean isLaunching)
  {
    m_isLaunching = isLaunching;
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
      for(var LED = ball * Constants.CHAMBER_TOTAL_NUM_OF_LEDS / 5; LED < (ball + 1) * Constants.CHAMBER_TOTAL_NUM_OF_LEDS / 5; LED++)
      {
        // Set the LED HSV value
        m_LEDLength.setHSV(LED, ball_color[ball].H, ball_color[ball].S, ball_color[ball].V);
      }
    }

    // Update the LED string
    m_chamberLEDs.setData(m_LEDLength);
  }
}