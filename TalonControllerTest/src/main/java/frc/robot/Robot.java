/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
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
  private static final String _testMode = "Manual Test";
  private static final String _motionMagicMode = "Motion Magic";
  private String _modeSelected;
  private final SendableChooser<String> _chooser = new SendableChooser<>();
  private final TalonSRX _talon = new TalonSRX(1);
  private final XboxController _controller = new XboxController(0);
  private final int _slotIndex = 0;
  private final int _loopIndex = 0;
  private final int _timeOutMs = 30;
  private final double _p = 0.2;
  private final double _i = 0.0;
  private final double _d = 0.0;
  private final double _f = 0.2;
  private double _ticksPerRevolution = 4096;
  private boolean _camera1Selected = true;
  private UsbCamera _camera1;
  private UsbCamera _camera2;
  private VideoSink _cameraServer;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    _chooser.setDefaultOption("Test Mode", _testMode);
    _chooser.addOption("Motion Magic Mode", _motionMagicMode);
    SmartDashboard.putData("Motion Mode", _chooser);
    _talon.configFactoryDefault();
    _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											_loopIndex, 
                      _timeOutMs);
                      
    /**
		 * Configure Talon SRX Output and Sensor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
    _talon.setInverted(true);
    
    /* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, _timeOutMs);
    _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, _timeOutMs);
    
    /* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, _timeOutMs);
		_talon.configNominalOutputReverse(0, _timeOutMs);
		_talon.configPeakOutputForward(1, _timeOutMs);
    _talon.configPeakOutputReverse(-1, _timeOutMs);
    
    /* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(_slotIndex, _loopIndex);
		_talon.config_kF(_slotIndex, _f, _timeOutMs);
		_talon.config_kP(_slotIndex, _p, _timeOutMs);
		_talon.config_kI(_slotIndex, _i, _timeOutMs);
    _talon.config_kD(_slotIndex, _d, _timeOutMs);
    
    /* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(10000, _timeOutMs);
    _talon.configMotionAcceleration(3000, _timeOutMs);
    
    /* Zero the sensor */
    _talon.getSelectedSensorPosition(_loopIndex);

    _camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    _camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    _cameraServer = CameraServer.getInstance().getServer();
    _camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    _cameraServer.setSource(_camera1);
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
    _modeSelected = _chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + _modeSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (_modeSelected) {
      case _motionMagicMode:
        // Put custom auto code here
        break;
      case _testMode:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    _modeSelected = _chooser.getSelected();
    super.teleopInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    switch (_modeSelected) {
      case _motionMagicMode:
        var targetPosition = 0;
        if (_controller.getAButton()){
          targetPosition = -17000;
        }
        _talon.set(ControlMode.MotionMagic, targetPosition);
        SmartDashboard.putNumber("Target Position", targetPosition);
        break;
      case _testMode:
        var targetOutput = -_controller.getY(Hand.kLeft);
        targetOutput = targetOutput < 0.05 && targetOutput > -0.05 ? 0 : targetOutput;
        _talon.set(ControlMode.PercentOutput, targetOutput);
        break;
      default:
        // Put default auto code here
        break;
    }
    SmartDashboard.putNumber("EncoderPosition", _talon.getSelectedSensorPosition(_loopIndex));

    if(_controller.getStickButtonPressed(Hand.kRight)){
      _talon.getSensorCollection().setQuadraturePosition(0, 0);
    }

    if(_controller.getBumperPressed(Hand.kRight)){
      _camera1Selected = !_camera1Selected;
      if (_camera1Selected){
        _cameraServer.setSource(_camera1);
      } else {
        _cameraServer.setSource(_camera2);
      }
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
