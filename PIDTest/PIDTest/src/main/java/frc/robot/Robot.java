/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements RobotMap {

  public static WPI_TalonSRX 
      m_rightTalon = new WPI_TalonSRX(MotorPorts.rightChassisUp),
      m_leftTalon = new WPI_TalonSRX(MotorPorts.leftChassisUp);

  public static WPI_VictorSPX
      m_rightVictorA = new WPI_VictorSPX(MotorPorts.rightChassisBack),
      m_leftVictorA = new WPI_VictorSPX(MotorPorts.leftChassisBack),
      m_rightVictorB = new WPI_VictorSPX(MotorPorts.rightChassisFront),
      m_leftVictorB = new WPI_VictorSPX(MotorPorts.leftChassisFront);
  public static Encoder 
    m_rightEncoder = new Encoder(DigitalPorts.rightEncoderA, DigitalPorts.rightEncoderB),
    m_leftEncoder = new Encoder(DigitalPorts.leftEncoderA, DigitalPorts.leftEncoderB);

  public static Gyro m_gyro = new ADXRS450_Gyro(); 
  public static Supplier<Double>
    m_angleKp = () -> Preferences.getInstance().getDouble("angleKp", 0),
    m_angleKi = () -> Preferences.getInstance().getDouble("angleKi", 0),
    m_angleKd = () -> Preferences.getInstance().getDouble("angleKd", 0),
    m_angleSetPoint = () -> Preferences.getInstance().getDouble("angleSetPoint", 0),
    m_angleTolerance = () -> Preferences.getInstance().getDouble("angleSTolerance", 1);
  public static Supplier<Boolean> m_activate = () -> Preferences.getInstance().getBoolean("activate", true);
  public static Consumer<Double> m_setSetPoint = (setPoint) -> Preferences.getInstance().putDouble("angleSetPoint", setPoint);
  public static Consumer<Boolean> m_activatePID = (toAble) -> Preferences.getInstance().putBoolean("activate", toAble);
  public static PIDController 
    m_displacmentChassisPID = new PIDController(0, 0, 0),
    m_angleChassisPID = new PIDController(0, 0, 0),
    m_speedChassisPID = new PIDController(0, 0, 0);

  public final double 
    DISTANCE_PER_PULSE = 0.003,//cm
    DISPLACEMENT_TOLERANCE = 1,//cm
    DISPLACEMENT_SET_POINT = 1000,//cm
    ANGLE_TOLERANCE = 0.01;//angle
  public static Joystick m_joystick = new Joystick(0);
  @Override
  public void robotInit() {
    Preferences.getInstance().putDouble("angleKp", 0);
    Preferences.getInstance().putDouble("angleKi", 0);
    Preferences.getInstance().putDouble("angleKd", 0);
    Preferences.getInstance().putDouble("angleSetPoint", 90);
    Preferences.getInstance().putDouble("angleSTolerance", 1);
    Preferences.getInstance().putBoolean("activate", true);

    
    m_leftTalon.setInverted(true);
    m_leftVictorA.setInverted(true);
    m_leftVictorB.setInverted(true);

    m_gyro.calibrate();

    // new JoystickButton(m_joystick, 1).whenPressed(new InstantCommand(() -> m_setSetPoint.accept(m_angleSetPoint.get() - 10)));
    // new JoystickButton(m_joystick, 2).whenPressed(new InstantCommand(() -> m_activatePID.accept(!m_activate.get())));
    // new JoystickButton(m_joystick, 3).whenPressed(new InstantCommand(() -> m_setSetPoint.accept(m_angleSetPoint.get() + 10)));
    // new JoystickButton(m_joystick, 4).whenPressed(new InstantCommand(() -> m_gyro.reset()));

  }

 
  @Override
  public void testPeriodic() {
    if (m_joystick.getRawButtonPressed(1)) m_setSetPoint.accept(m_angleSetPoint.get() - 10);
    if (m_joystick.getRawButtonPressed(2)) m_activatePID.accept(!m_activate.get());
    if (m_joystick.getRawButtonPressed(3)) m_setSetPoint.accept(m_angleSetPoint.get() + 10);
    if (m_joystick.getRawButtonPressed(4)) m_gyro.reset();
    
    //DisplacemetPID:
    // driveLeft(m_displacmentChassisPID.calculate(m_rightTalon.getSelectedSensorPosition()*DISTANCE_PER_PULSE));
    // driveRight(m_displacmentChassisPID.calculate(m_leftTalon.getSelectedSensorPosition()*DISTANCE_PER_PULSE));
    m_angleChassisPID.setSetpoint(m_angleSetPoint.get());
    m_angleChassisPID.setTolerance(m_angleTolerance.get());
    m_angleChassisPID.setP(m_angleKp.get());
    m_angleChassisPID.setI(m_angleKi.get());
    m_angleChassisPID.setD(m_angleKd.get());
    SmartDashboard.putNumber("error", m_angleChassisPID.getPositionError());
    if(m_activate.get()){
    // driveRight((m_angleChassisPID.calculate(m_gyro.getAngle()))%360);
    // driveLeft(((m_angleChassisPID.calculate(m_gyro.getAngle()))%360)*-0.9);
    driveRight((m_angleChassisPID.calculate(m_gyro.getAngle()))%360);
    driveLeft(((m_angleChassisPID.calculate(m_gyro.getAngle()))%360)*-0.9);
    }else{
      driveLeft(0);
      driveRight(0);
    }
  }

  public void driveRight(double speed) {
    m_rightTalon.set(speed);
    m_rightVictorA.set(speed);
    m_rightVictorB.set(speed);
  }
  public void driveLeft(double speed) {
    m_leftTalon.set(speed);
    m_leftVictorA.set(speed);
    m_leftVictorB.set(speed);
  }

}
