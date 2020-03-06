// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;

// public class ControlPanelSpinFour extends CommandBase 
// {
//   // The subsystem the command runs on
//   private String m_startColor;
//   private int m_halfTurn;
//   private boolean m_colorChangeFlag;

//   public ControlPanelSpinFour()
//   {
//     // addRequirements(m_subsystem);
//   }

//   @Override
//   public void initialize()
//   {
//     m_startColor = RobotContainer.m_controlPanel.getCurrentColor();
//     m_halfTurn = 0;
//     m_colorChangeFlag = false;
//     RobotContainer.m_controlPanel.setPanelSpinnerPower(Constants.CONTROL_PANEL_SPINNER_POWER);
//   }
  
//   @Override
//   public void execute()
//   {
//       //get current color
//     String currentColor = RobotContainer.m_controlPanel.getCurrentColor();
     
//       SmartDashboard.putString("color2", m_startColor);
//       SmartDashboard.putNumber("Half Turn Count", m_halfTurn);
    
//       if (m_startColor != currentColor)
//      {
//         m_colorChangeFlag = true;
//      }

//      if (m_colorChangeFlag == true && m_startColor == currentColor)
//     {
//         m_halfTurn++;
//         m_colorChangeFlag = false;
//     }
//   }

//   @Override
//   public void end(boolean interrupted)
//   {
//     RobotContainer.m_controlPanel.setPanelSpinnerPower(0);
//   }

//   @Override
//   public boolean isFinished()
//   {
//     //spin panel until you see the same color 8 times
//     if (m_halfTurn > 8)
//      {
//        return true;
//      }
//     return false;
//   }
// }
