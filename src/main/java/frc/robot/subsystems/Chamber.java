package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chamber extends SubsystemBase {

  private TalonSRX m_chamberStart;
  private TalonSRX m_chamberEnd;
  private DigitalInput m_ballPos_1;
  private DigitalInput m_ballPos_2;
  private DigitalInput m_ballPos_3;
  private DigitalInput m_ballPos_4;

  public Chamber() {

    m_chamberStart = new TalonSRX(Constants.CHAMBER_START_ID);
    m_chamberEnd = new TalonSRX(Constants.CHAMBER_END_ID);
    m_ballPos_1 = new DigitalInput(Constants.CHAMBER_BALL_POS_1_PORT);
    m_ballPos_2 = new DigitalInput(Constants.CHAMBER_BALL_POS_2_PORT);
    m_ballPos_3 = new DigitalInput(Constants.CHAMBER_BALL_POS_3_PORT);
    m_ballPos_4 = new DigitalInput(Constants.CHAMBER_BALL_POS_4_PORT);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}