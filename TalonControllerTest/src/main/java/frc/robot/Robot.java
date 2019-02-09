/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final WPI_TalonSRX _talon = new WPI_TalonSRX(1);
  private final XboxController _controller = new XboxController(0);

  private double _numberOfRevolutions = 0.0;
  private double _revolutionLimit = 1;
  
  /* Nonzero to block the config until success, zero to skip checking */
  private final int kTimeoutMs = 30;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    _talon.configFactoryDefault();
    _talon.configSetParameter(ParamEnum.eOpenloopRamp, 0.2, 0, 0);
    initQuadrature();
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
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (_controller.getAButtonPressed()){
      initQuadrature();
      _numberOfRevolutions = 0;
    }

    if (_controller.getBumperPressed(Hand.kRight)){
      _revolutionLimit += 1;
    }

    if (_controller.getBumperPressed(Hand.kLeft)){
      _revolutionLimit -= 1;
    }

    /**
		 * Quadrature is selected for soft-lim/closed-loop/etc. initQuadrature()
		 * will initialize quad to become absolute by using PWD
		 */
    int selSenPos = _talon.getSelectedSensorPosition(0);
    _numberOfRevolutions = selSenPos / 4096.0;
		int pulseWidthWithoutOverflows = 
				_talon.getSensorCollection().getPulseWidthPosition() & 0xFFF;

		/**
		 * Display how we've adjusted PWM to produce a QUAD signal that is
		 * absolute and continuous. Show in sensor units and in rotation
		 * degrees.
		 */
		System.out.print("pulseWidPos:" + pulseWidthWithoutOverflows +
						 "   =>    " + "selSenPos:" + selSenPos);
		System.out.print("      ");
		System.out.print("pulseWidDeg:" + ToDeg(pulseWidthWithoutOverflows) +
						 "   =>    " + "selSenDeg:" + ToDeg(selSenPos));
    System.out.println();
    System.out.println("Number of revolutions: " + _numberOfRevolutions);
    System.out.println("Revolution limit: " + _revolutionLimit);


    var revLimit = 5.0 * (_revolutionLimit - _numberOfRevolutions);
    revLimit = Math.min(1.0, revLimit);
    revLimit = Math.max(0.0, revLimit);
    var output = _controller.getY(Hand.kLeft) * .5 * revLimit;
    _talon.set(ControlMode.PercentOutput, output);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void initQuadrature() {
		/* Update Quadrature position */
		_talon.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
  }
  
  /**
	 * @param units CTRE mag encoder sensor units 
	 * @return degrees rounded to tenths.
	 */
	private String ToDeg(int units) {
		double deg = units * 360.0 / 4096.0;

		/* truncate to 0.1 res */
		deg *= 10;
		deg = (int) deg;
		deg /= 10;

		return "" + deg;
	}
}
