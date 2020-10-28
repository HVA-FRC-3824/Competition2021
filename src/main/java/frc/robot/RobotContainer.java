package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * button mappings) should be declared here.
 */
public class RobotContainer
{
  /**
   * Instantiation of subsystems.
   * Subsystems were not made to be private as recommended in the documentation because they are 
   * practically being used in every class. Making them private would create the need 
   * to pass in the subsystem to each individual class which is more work than just making them public.
   */
  public static final Chamber m_chamber = new Chamber();
  public static final Chassis m_chassis = new Chassis();
  public static final Climber m_climber = new Climber();
  // public static final ControlPanel m_controlPanel = new ControlPanel();
  public static final Intake m_intake = new Intake();
  public static final Launcher m_launcher = new Launcher();

  public static final Limelight m_limelight = Limelight.getInstance();
  public static final LEDs m_LEDs = new LEDs();

  /**
   * Instantiation of OI and inline commands.
   * The OI class requires the inline commands class to be instantiated when binding 
   * commands to joystick buttons. The inline commands class requires the OI class when
   * retrieiving joystick values. To avoid null pointer exceptions, the OI class is
   * instantiated first without binding commands to joystick buttons, but creating the
   * joystick object. Then, the inline commands class is instantiated. Finally, in the
   * RobotContainer constructor, the method in the OI class to bind commands to joystick
   * buttons is called.
   */
  public static final InlineCommands m_inlineCommands = new InlineCommands();
  public static final OI m_OI = new OI();
  
  /**
   * Instantiation of other commands.
   */

  /**
   * Instantiation of autonomous chooser.
   * Allows operators to preselect which autonomous command to run during autonomous period.
   */
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  /**
   * This code runs at robotInit.
   */
  public RobotContainer() 
  {
    /* Bind commands to joystick buttons. */
    m_OI.configureButtonBindings();

    /* Initialize various systems on robotInit. */
    this.initializeStartup();

    /* Initialize autonomous command chooser and display on the SmartDashboard. */
    this.initializeAutoChooser();

    /* Initialize PID tuning for use on the SmartDashboard. */
    this.initializePIDValues();

    // this.testColorSensing();
  }

  /**
   * Various methods to run when robot is initialized. Cannot put these in
   * robotInit() in Robot.java because subsystems may not be instantiated at that
   * point.
   */
  private void initializeStartup()
  {
    /* Turn off Limelight LED when first started up so it doesn't blind drive team. */
    m_limelight.turnOffLED();

    /* Start ultrasonics. */
    m_chamber.startUltrasonics();
  }

  /**
   * Set default command for subsystems. Default commands are commands that run
   * automatically whenever a subsystem is not being used by another command. If
   * default command is set to null, there will be no default command for the
   * subsystem.
   */
  public static void initializeDefaultCommands()
  {
    m_chassis.setDefaultCommand(m_inlineCommands.m_driveWithJoystick);
    // m_intake.setDefaultCommand(null);
    m_chamber.setDefaultCommand(new ChamberIndexBalls());
    // m_launcher.setDefaultCommand(null);
    // m_climber.setDefaultCommand(null);
    // m_controlPanel.setDefaultCommand(null);
  }

  /**
   * Set options for autonomous command chooser and display them for selection on
   * the SmartDashboard. Using string chooser rather than command chooser because
   * if using a command chooser, will instantiate all the autonomous commands.
   * This may cause problems (e.g. initial trajectory position is from a different
   * command's path).
   */
  private void initializeAutoChooser()
  {
    /* Add options (which autonomous commands can be selected) to chooser. */
    m_autoChooser.setDefaultOption("DEFAULT COMMAND NAME HERE", "default");
    m_autoChooser.addOption("TEST", "test");
    m_autoChooser.addOption("THREE BALL FORWARD", "three_ball_forward");
    m_autoChooser.addOption("THREE BALL BACKWARD", "three_ball_backward");
    m_autoChooser.addOption("SIX BALL", "six_ball");

    /*
     * Display chooser on SmartDashboard for operators to select which autonomous
     * command to run during the auto period.
     */
    SmartDashboard.putData("Autonomous Command", m_autoChooser);
  }

  /**
   * This method is used to pass the autonomous command to the main Robot class.
   * 
   * @return the command to run during the autonomous period.
   */
  public Command getAutonomousCommand() 
  {
    switch (m_autoChooser.getSelected())
    {
      case "default":
        return null;
      case "test":
        return new CommandGroupTemplate();
      case "three_ball_forward":
        return new AutonomousThreeBall(1, 2.0);
      case "three_ball_backward":
        return new AutonomousThreeBall(-1, 3.0);
      case "six_ball":
        return new AutonomousSixBall();
      default:
        System.out.println("\nError selecting autonomous command:\nCommand selected: " + m_autoChooser.getSelected() + "\n");
        return null;
    }
  }

  /**
   * Configures TalonSRX objects with passed in parameters.
   * 
   * @param controlMode If true, configure with Motion Magic. If false, configure
   *                    without Motion Magic. (Motion Magic not required for
   *                    TalonSRXs that will set with ControlMode.Velocity).
   */
  public static void configureTalonSRX(WPI_TalonSRX talonSRX, boolean controlMode, FeedbackDevice feedbackDevice,
      boolean setInverted, boolean setSensorPhase, double kF, double kP, double kI, double kD, int kCruiseVelocity,
      int kAcceleration, boolean resetPos)
  {
    /* Factory default to reset TalonSRX and prevent unexpected behavior. */
    talonSRX.configFactoryDefault();

    /* Configure Sensor Source for Primary PID. */
    talonSRX.configSelectedFeedbackSensor(feedbackDevice, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);

    /* Configure TalonSRX to drive forward when LED is green. */
    talonSRX.setInverted(setInverted);

    /* Configure TalonSRX's sensor to increment its value as it moves forward. */
    talonSRX.setSensorPhase(setSensorPhase);

    // Determine if the internal PID is being used
    if (controlMode)
    {
      /*
       * Set relevant frame periods (Base_PIDF0 and MotionMagic) to periodic rate
       * (10ms).
       */
      talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.K_TIMEOUT_MS);
      talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.K_TIMEOUT_MS);
    }

    /**
     * Configure the nominal and peak output forward/reverse.
     * 
     * Nominal Output: minimal/weakest motor output allowed during closed-loop. Peak
     * Output: maximal/strongest motor output allowed during closed-loop.
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

    // Determine if the internal PID is being used
    if (controlMode)
    {
      /* Set acceleration and cruise velocity for Motion Magic. */
      talonSRX.configMotionCruiseVelocity(kCruiseVelocity, Constants.K_TIMEOUT_MS);
      talonSRX.configMotionAcceleration(kAcceleration, Constants.K_TIMEOUT_MS);
    }

    /* Reset/zero the TalonSRX's sensor. */
    if (resetPos)
    {
      talonSRX.setSelectedSensorPosition(0, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);
    }
  }

  /**
   * Configures TalonFX (Falcon 500) objects with passed in parameters. Falcon
   * 500s will be used for the chassis and launcher wheels only, thus Motion Magic
   * is not required. (PIDController with Gyro/Vision or ControlMode.Velocity will
   * be used instead).
   */
  public static void configureTalonFX(WPI_TalonFX talonFX, boolean setInverted, boolean setSensorPhase, double kF,
      double kP, double kI, double kD) 
  {
    /* Factory default to reset TalonFX and prevent unexpected behavior. */
    talonFX.configFactoryDefault();

    /* Configure Sensor Source for Primary PID. */
    talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.K_PID_LOOP_IDX,
        Constants.K_TIMEOUT_MS);

    /* Configure TalonFX to drive forward when LED is green. */
    talonFX.setInverted(setInverted);
    /* Configure TalonFX's sensor to increment its value as it moves forward. */
    talonFX.setSensorPhase(setSensorPhase);

    /**
     * Configure the nominal and peak output forward/reverse.
     * 
     * Nominal Output: minimal/weakest motor output allowed during closed-loop. Peak
     * Output: maximal/strongest motor output allowed during closed-loop.
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
     * Reset/zero the TalonFX's sensor. Will be required for implementation into
     * chassis (position considered), but not launcher (velocity only).
     */
    talonFX.setSelectedSensorPosition(0, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);
  }

  /**
   * Initializes SmartDashboard data for PID tuning. Creates fields for gains and
   * button for initiating PID configuration.
   */
  private void initializePIDValues()
  {
    /* Generate number fields on SmartDashboard for PID values to be input into. */
    // SmartDashboard.putNumber("F Value", 0.0);
    // SmartDashboard.putNumber("P Value", 0.0);
    // SmartDashboard.putNumber("I Value", 0.0);
    // SmartDashboard.putNumber("D Value", 0.0);

    // SmartDashboard.putNumber("Cruise Velocity Value", 0);
    // SmartDashboard.putNumber("Acceleration Value", 0);
    
    /**
     * Create button for when pressed on SmartDashboard will configure the PID of
     * the hard coded TalonSRX/TalonFX. Get TalonSRX/TalonFX with get method written
     * in each subsystem for each TalonSRX/TalonFX. When desiring to set the PID
     * values for another TalonSRX/TalonFX, you must hard code in the new parameters
     * for the SetPIDValues command then re-deploy.
     * 
     * SetPIDValues Parameters: TalonSRX object (pass in null if configuring
     * TalonFX). TalonFX object (pass in null if configuring TalonSRX). ControlMode
     * boolean: if true, Motion Magic is being used, if false, Motion Magic is not
     * being used.
     */
    // SmartDashboard.putData("Set PID Values", new SetPIDValues(m_intake.getWheelIntakeTalonSRX(), null, false));
  }

  /**
   * Method to spin color wheel four times with color inuput.
   */
  public void testColorSensing()
  {
    // SmartDashboard.putData("Color Wheel Spinning", new ControlPanelSpinFour());
  }

  /**
   * Method to display position, velocity, error, and motor ouput of a TalonSRX.
   * Primarily used for PID tuning.
   */
  public static void displayTalonSRXInfo(WPI_TalonSRX talonSRX, String label)
  {
    SmartDashboard.putNumber(label + " Setpoint", talonSRX.getClosedLoopTarget());
    SmartDashboard.putNumber(label + " Position", talonSRX.getSelectedSensorPosition());
    SmartDashboard.putNumber(label + " Velocity", talonSRX.getSelectedSensorVelocity());
    SmartDashboard.putNumber(label + " Error",    talonSRX.getClosedLoopError());
    SmartDashboard.putNumber(label + " Output",   talonSRX.getMotorOutputVoltage());
  }

  /**
   * Method to display position, velocity, error, and motor ouput of a TalonFX.
   * Primarily used for PID tuning.
   */
  public static void displayTalonFXInfo(WPI_TalonFX talonFX, String label)
  {
    SmartDashboard.putNumber(label + " Setpoint", talonFX.getClosedLoopTarget());
    SmartDashboard.putNumber(label + " Position", talonFX.getSelectedSensorPosition());
    SmartDashboard.putNumber(label + " Velocity", talonFX.getSelectedSensorVelocity());
    SmartDashboard.putNumber(label + " Error",    talonFX.getClosedLoopError());
    SmartDashboard.putNumber(label + " Output",   talonFX.getMotorOutputVoltage());
  }

  /**
   * Convert RPM to units/100ms for TalonSRX/TalonFX to use for ControlMode.Velocity.
   * @param rpm is desired revolutions per minute.
   * @param tpr is the encoder ticks per revolution.
   */
  public static double convertRPMToVelocity(int rpm, int tpr)
  {
    /* (RPM * TPR Units/Revolution / 600 100ms/min) */
    return rpm * tpr / 600;
  }
}