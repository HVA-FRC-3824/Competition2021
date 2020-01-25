/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.SetPIDValues;
import frc.robot.subsystems.*;
import frc.robot.OI;
import frc.robot.commands.InlineCommands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  /**
   * Autonomous Command and Chooser
   * Allows operators to preselect which autonomous command to run during autonomous period.
   */
  private Command m_autonomousCommand;
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * Class Instantiation (Subsystems, OI, and InlineCommands)
   */
  public static final Chassis m_chassis = new Chassis();
  public static final Intake m_intake = new Intake();
  public static final Chamber m_chamber = new Chamber();
  public static final Launcher m_launcher = new Launcher();
  public static final Climber m_climber = new Climber();
  public static final ControlPanel m_controlPanel = new ControlPanel();

  public static final OI m_OI = new OI();
  public static final InlineCommands m_inlineCommands = new InlineCommands();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    /* Initialize default commands for all subsystems. */
    this.initializeDefaultCommands();

    /* Initialize autonomous command chooser and display on the SmartDashboard. */
    this.initializeAutoChooser();

    /* Initialize PID tuning for use on the SmartDashboard. */
    this.initializePIDValues();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    /**
     * Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
     * commands, running already-scheduled commands, removing finished or interrupted commands,
     * and running subsystem periodic() methods.  This must be called from the robot's periodic
     * block in order for anything in the Command-based framework to work.
     */
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called at the start of autonomous.
   */
  @Override
  public void autonomousInit()
  {
    /**
     * Gets selected autonomous command from autoChooser and schedules said command.
     */
    m_autonomousCommand = m_autoChooser.getSelected();
    if (m_autonomousCommand != null)
      m_autonomousCommand.schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  /**
   * This function is called at the start of operator control.
   */
  @Override
  public void teleopInit() 
  {
    /**
     * This makes sure that the autonomous stops running when
     * teleop starts running. If you want the autonomous to
     * continue until interrupted by another command, remove
     * this line or comment it out.
     */
    if (m_autonomousCommand != null)
      m_autonomousCommand.cancel();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    // m_launcher.setWheelPower(m_OI.getOperatorController().getRawAxis(5));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * Set default command for subsystems.
   * Default commands are commands that run automatically whenever a subsystem is not being used by another command.
   * If default command is set to null, there will be no default command for the subsystem.
   */
  public void initializeDefaultCommands()
  {
    m_chassis.setDefaultCommand(m_inlineCommands.m_driveWithJoystick);
    m_intake.setDefaultCommand(null);
    m_chamber.setDefaultCommand(null);
    m_launcher.setDefaultCommand(m_inlineCommands.m_setLauncherWheelsPower);
    m_climber.setDefaultCommand(null);
    m_controlPanel.setDefaultCommand(null);
  }

  /**
   * Set options for autonomous command chooser and display them for selection on the SmartDashboard.
   */
  public void initializeAutoChooser()
  {
    /* Add options (which autonomous commands can be selected) to chooser. */
    m_autoChooser.setDefaultOption("DEFAULT COMMAND NAME HERE", /*DEFAULT COMMAND HERE*/null);
    m_autoChooser.addOption("COMMAND NAME HERE", /*COMMAND HERE*/null);

    /* Display chooser on SmartDashboard for operators to select which autonomous command to run during the auto period. */
    SmartDashboard.putData("Autonomous Command", m_autoChooser);
  }

  /**
   * Configures TalonSRX objects with passed in parameters.
   * 
   * @param controlMode If true, configure with Motion Magic. If false, configure without Motion Magic.
   *                    (Motion Magic not required for TalonSRXs that will set with ControlMode.Velocity).        
   */
  public static void configureTalonSRX(WPI_TalonSRX talonSRX, boolean controlMode, FeedbackDevice feedbackDevice, boolean setInverted, 
                                       boolean setSensorPhase, double kF, double kP, double kI, double kD, 
                                       int kCruiseVelocity, int kAcceleration)
  {
    /* Factory default to reset TalonSRX and prevent unexpected behavior. */
    talonSRX.configFactoryDefault();

    /* Configure Sensor Source for Primary PID. */
    talonSRX.configSelectedFeedbackSensor(feedbackDevice, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);

    /* Configure TalonSRX to drive forward when LED is green. */
    talonSRX.setInverted(setInverted);
    /* Configure TalonSRX's sensor to increment its value as it moves forward. */
    talonSRX.setSensorPhase(setSensorPhase);

    if (controlMode) 
    {
      /* Set relevant frame periods (Base_PIDF0 and MotionMagic) to periodic rate (10ms). */
      talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.K_TIMEOUT_MS);
      talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.K_TIMEOUT_MS);
    }

    /** 
     * Configure the nominal and peak output forward/reverse.
     * 
     * Nominal Output: minimal/weakest motor output allowed during closed-loop.
     * Peak Output: maximal/strongest motor output allowed during closed-loop.
     */
    talonSRX.configNominalOutputForward(0, Constants.K_TIMEOUT_MS);
    talonSRX.configNominalOutputReverse(0, Constants.K_TIMEOUT_MS);
    talonSRX.configPeakOutputForward(1, Constants.K_TIMEOUT_MS);
    talonSRX.configPeakOutputReverse(-1, Constants.K_TIMEOUT_MS);

    /* Set Motion Magic/Velocity gains (FPID) in slot0. */
    talonSRX.selectProfileSlot(Constants.K_SLOT_IDX, Constants.K_PID_LOOP_IDX);
    talonSRX.config_kF(Constants.K_SLOT_IDX, kF, Constants.K_TIMEOUT_MS);
    talonSRX.config_kP(Constants.K_SLOT_IDX, kP, Constants.K_TIMEOUT_MS);
    talonSRX.config_kI(Constants.K_SLOT_IDX, kI, Constants.K_TIMEOUT_MS);
    talonSRX.config_kD(Constants.K_SLOT_IDX, kD, Constants.K_TIMEOUT_MS);

    if (controlMode) 
    {
      /* Set acceleration and cruise velocity for Motion Magic. */
      talonSRX.configMotionCruiseVelocity(kCruiseVelocity, Constants.K_TIMEOUT_MS);
      talonSRX.configMotionAcceleration(kAcceleration, Constants.K_TIMEOUT_MS);
    }

    /* Reset/zero the TalonSRX's sensor. */
    talonSRX.setSelectedSensorPosition(0, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);
  }

  /**
   * Configures TalonFX (Falcon 500) objects with passed in parameters.
   * Falcon 500s will be used for the chassis and launcher wheels only, thus Motion Magic is not required.
   * (PIDController with Gyro/Vision or ControlMode.Velocity will be used instead).
   */
  public static void configureTalonFX(WPI_TalonFX talonFX, boolean setInverted, boolean setSensorPhase,
                               double kF, double kP, double kI, double kD) 
  {
    /* Factory default to reset TalonFX and prevent unexpected behavior. */
    talonFX.configFactoryDefault();

    /* Configure Sensor Source for Primary PID. */
    talonFX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);

    /* Configure TalonFX to drive forward when LED is green. */
    talonFX.setInverted(setInverted);
    /* Configure TalonFX's sensor to increment its value as it moves forward. */
    talonFX.setSensorPhase(setSensorPhase);

    /** 
     * Configure the nominal and peak output forward/reverse.
     * 
     * Nominal Output: minimal/weakest motor output allowed during closed-loop.
     * Peak Output: maximal/strongest motor output allowed during closed-loop.
     */
    talonFX.configNominalOutputForward(0, Constants.K_TIMEOUT_MS);
    talonFX.configNominalOutputReverse(0, Constants.K_TIMEOUT_MS);
    talonFX.configPeakOutputForward(1, Constants.K_TIMEOUT_MS);
    talonFX.configPeakOutputReverse(-1, Constants.K_TIMEOUT_MS);

    /* Set the Velocity gains (FPID) in slot0. */
    talonFX.selectProfileSlot(Constants.K_SLOT_IDX, Constants.K_PID_LOOP_IDX);
    talonFX.config_kF(Constants.K_SLOT_IDX, kF, Constants.K_TIMEOUT_MS);
    talonFX.config_kP(Constants.K_SLOT_IDX, kP, Constants.K_TIMEOUT_MS);
    talonFX.config_kI(Constants.K_SLOT_IDX, kI, Constants.K_TIMEOUT_MS);
    talonFX.config_kD(Constants.K_SLOT_IDX, kD, Constants.K_TIMEOUT_MS);

    /**
     * Reset/zero the TalonFX's sensor.
     * Will be required for implementation into chassis (position considered), but not launcher (velocity only). 
     */
    talonFX.setSelectedSensorPosition(0, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);
  }

  /**
   * Initializes SmartDashboard data for PID tuning.
   * Creates fields for gains and button for initiating PID configuration.
   */
  public void initializePIDValues() 
  {
    /* Generate number fields on SmartDashboard for PID values to be input into. */
    SmartDashboard.putNumber("F Value", 0.0);
    SmartDashboard.putNumber("P Value", 0.0);
    SmartDashboard.putNumber("I Value", 0.0);
    SmartDashboard.putNumber("D Value", 0.0);

    SmartDashboard.putNumber("Cruise Velocity Value", 0);
    SmartDashboard.putNumber("Acceleration Value", 0);
    
    /**
     * Create button for when pressed on SmartDashboard will configure the PID of the hard coded TalonSRX/TalonFX.
     * Get TalonSRX/TalonFX with get method written in each subsystem for each TalonSRX/TalonFX.
     * When desiring to set the PID values for another TalonSRX/TalonFX, you must hard code in the new parameters for
     * the SetPIDValues command then re-deploy.
     * 
     * SetPIDValues Parameters:
     * TalonSRX object (pass in null if configuring TalonFX).
     * TalonFX object (pass in null if configuring TalonSRX).
     * ControlMode boolean: if true, Motion Magic is being used, if false, Motion Magic is not being used.
     */
    //SmartDashboard.putData("Set PID Values", new SetPIDValues(null, m_launcher.getTopWheelTalonFX(), false));
  }

  /**
   * Convert RPM to units/100ms for TalonSRX/TalonFX to use for ControlMode.Velocity
   */
  public static double convertRPMToVelocity(int rpm)
  {
    /* (RPM * 4096 Units/Revolution / 600 100ms/min) */
    return rpm * 4096 / 600;
  }
}