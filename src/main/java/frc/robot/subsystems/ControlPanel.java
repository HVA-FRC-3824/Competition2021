<<<<<<< HEAD
package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import javax.annotation.meta.When;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlPanel extends SubsystemBase
{
  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private WPI_TalonSRX m_panelSpinner;

  // private ColorSensorV3 m_colorSensor;

  // public String gameData = DriverStation.getInstance().getGameSpecificMessage();
  // public int currentRed;
  // public int currentBlue;
  // public int currentGreen;
  // public String currentColor;

  public ControlPanel() 
  {
    // m_panelSpinner = new WPI_TalonSRX(Constants.CONTROL_PANEL_SPINNER_ID);
    // RobotContainer.configureTalonSRX(m_panelSpinner, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
    //                                 Constants.CONTROL_PANEL_SPINNER_F, Constants.CONTROL_PANEL_SPINNER_P, 
    //                                 Constants.CONTROL_PANEL_SPINNER_I, Constants.CONTROL_PANEL_SPINNER_D, 0, 0, true);

    // m_colorSensor = new ColorSensorV3(i2cPort);

    // if(gameData.length() > 0)
    // {
    //   switch (gameData.charAt(0))
    //   {
    //     case 'B' :
    //       //Blue case code
    //       break;
    //     case 'G' :
    //       //Green case code
    //       break;
    //     case 'R' :
    //       //Red case code
    //       break;
    //     case 'Y' :
    //       //Yellow case code
    //       break;
    //     default :
    //       //This is corrupt data
    //       break;
    //   }
    // } else 
    //   {
    //   //Code for no data received yet
    //   }
  }


  /**
   * This method will be called once per scheduler run.
   */
  @Override
  public void periodic() 
  {
    // currentRed = m_colorSensor.getRed();
    // SmartDashboard.putNumber("Red", currentRed);
    // currentGreen = m_colorSensor.getGreen() / 2;
    // SmartDashboard.putNumber("Green", currentGreen);
    // currentBlue = m_colorSensor.getBlue();
    // SmartDashboard.putNumber("Blue", currentBlue);
    // getCurrentColor();
  }

  /**
   * Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  // public WPI_TalonSRX getPanelSpinnerTalonSRX() 
  // {
  //   return m_panelSpinner;
  // }

  // public void setPanelSpinnerPower(double power)
  // {
  //   m_panelSpinner.set(ControlMode.PercentOutput, power);
  // }

  // public void setPanelSpinnerRPM(int rpm)
  // {
  //   m_panelSpinner.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.CONTROL_PANEL_SPINNER_TPR));
  // }

  // public String getCurrentColor()
  // {
  //   if(currentRed > currentBlue && currentRed > currentGreen)
  //   {
  //     currentColor = "Yellow";
  //     if (currentRed > currentGreen*2)
  //     {
  //       currentColor = "Red";
  //     }
      
  //   }
  //   else if (currentBlue > currentRed && currentBlue > currentGreen)
  //   {
  //     currentColor = "Blue";
  //   }
  //   else if (currentGreen > currentRed && currentGreen > currentBlue)
  //   {
  //     currentColor = "Green";
  //   }
    
  //   SmartDashboard.putString("Color", currentColor);

  //   return currentColor;
  // }
=======
// package frc.robot.subsystems;

// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;

// import javax.annotation.meta.When;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorSensorV3.RawColor;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class ControlPanel extends SubsystemBase
// {
//   private final I2C.Port i2cPort = I2C.Port.kOnboard;
//   private WPI_TalonSRX m_panelSpinner;

//   private ColorSensorV3 m_colorSensor;

//   public String gameData = DriverStation.getInstance().getGameSpecificMessage();
//   public int currentRed;
//   public int currentBlue;
//   public int currentGreen;
//   public String currentColor;

//   public ControlPanel() 
//   {
//     m_panelSpinner = new WPI_TalonSRX(Constants.CONTROL_PANEL_SPINNER_ID);
//     RobotContainer.configureTalonSRX(m_panelSpinner, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
//                                     Constants.CONTROL_PANEL_SPINNER_F, Constants.CONTROL_PANEL_SPINNER_P, 
//                                     Constants.CONTROL_PANEL_SPINNER_I, Constants.CONTROL_PANEL_SPINNER_D, 0, 0, true);

//     m_colorSensor = new ColorSensorV3(i2cPort);

//     if(gameData.length() > 0)
//     {
//       switch (gameData.charAt(0))
//       {
//         case 'B' :
//           //Blue case code
//           break;
//         case 'G' :
//           //Green case code
//           break;
//         case 'R' :
//           //Red case code
//           break;
//         case 'Y' :
//           //Yellow case code
//           break;
//         default :
//           //This is corrupt data
//           break;
//       }
//     } else 
//       {
//       //Code for no data received yet
//       }
//   }


//   /**
//    * This method will be called once per scheduler run.
//    */
//   @Override
//   public void periodic() 
//   {
//     currentRed = m_colorSensor.getRed();
//     SmartDashboard.putNumber("Red", currentRed);
//     currentGreen = m_colorSensor.getGreen() / 2;
//     SmartDashboard.putNumber("Green", currentGreen);
//     currentBlue = m_colorSensor.getBlue();
//     SmartDashboard.putNumber("Blue", currentBlue);
//     getCurrentColor();
//   }

//   /**
//    * Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
//    * @return TalonFX/TalonSRX object to be configured.
//    */
//   public WPI_TalonSRX getPanelSpinnerTalonSRX() 
//   {
//     return m_panelSpinner;
//   }

//   public void setPanelSpinnerPower(double power)
//   {
//     m_panelSpinner.set(ControlMode.PercentOutput, power);
//   }

//   public void setPanelSpinnerRPM(int rpm)
//   {
//     m_panelSpinner.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.CONTROL_PANEL_SPINNER_TPR));
//   }

//   public String getCurrentColor()
//   {
//     if(currentRed > currentBlue && currentRed > currentGreen)
//     {
//       currentColor = "Yellow";
//       if (currentRed > currentGreen*2)
//       {
//         currentColor = "Red";
//       }
      
//     }
//     else if (currentBlue > currentRed && currentBlue > currentGreen)
//     {
//       currentColor = "Blue";
//     }
//     else if (currentGreen > currentRed && currentGreen > currentBlue)
//     {
//       currentColor = "Green";
//     }
    
//     SmartDashboard.putString("Color", currentColor);

//     return currentColor;
//   }
>>>>>>> ea5c9f697b01014cf5377d8a2284688eba22f41c

// }