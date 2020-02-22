/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import javax.management.relation.Role;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements RobotMap {

    public static WPI_TalonSRX 
      m_rightChassisTalon = new WPI_TalonSRX(MotorPorts.chassisRightMiddle),
      m_leftTalon = new WPI_TalonSRX(MotorPorts.chassisLeftMiddle),
      m_aimer = new WPI_TalonSRX(MotorPorts.aimer),
      m_rolleta = new WPI_TalonSRX(MotorPorts.spinner);
      
      
      public static WPI_VictorSPX
      m_rightChassisA = new WPI_VictorSPX(MotorPorts.chassisRightBack),
      m_leftChassisA = new WPI_VictorSPX(MotorPorts.chassisLeftFront),
      m_rightChassisB = new WPI_VictorSPX(MotorPorts.chassisRightFront),
      m_leftChassisB = new WPI_VictorSPX(MotorPorts.chassisLeftFront),
      m_shooter = new WPI_VictorSPX(MotorPorts.thrower);


  
  public static Encoder 
    m_leftEncoder = new Encoder(DigitalPorts.leftChassisEncoderA, DigitalPorts.leftChassisEncoderB),
    m_shooterEncoder = new Encoder(DigitalPorts.throwerEncoderA, DigitalPorts.throwerEncoderB);

  public static Gyro m_gyro = new ADXRS450_Gyro(); 

  public static Supplier<Double>
    m_angleKp = () -> Preferences.getInstance().getDouble("Angle/kP", 0),
    m_angleKi = () -> Preferences.getInstance().getDouble("Angle/kI", 0),
    m_angleKd = () -> Preferences.getInstance().getDouble("Angle/kD", 0),
    m_angleSetPoint = () -> Preferences.getInstance().getDouble("Angle/SetPoint", 0),
    m_angleTolerance = () -> Preferences.getInstance().getDouble("Angle/Tolerance", 1),
    
    m_distanceKp = () -> Preferences.getInstance().getDouble("Distance/kP", 0),
    m_distanceKi = () -> Preferences.getInstance().getDouble("Distance/kI", 0),
    m_distanceKd = () -> Preferences.getInstance().getDouble("Distance/kD", 0),
    m_distanceSetPoint = () -> Preferences.getInstance().getDouble("Distance/SetPoint", 0),
    m_distanceTolerance = () -> Preferences.getInstance().getDouble("Distance/Tolerance", 1),

    m_velocityKp = () -> Preferences.getInstance().getDouble("Velocity/kP", 0),
    m_velocityKi = () -> Preferences.getInstance().getDouble("Velocity/kI", 0),
    m_velocityKd = () -> Preferences.getInstance().getDouble("Velocity/kD", 0),
    m_velocitySetPoint = () -> Preferences.getInstance().getDouble("Velocity/SetPoint", 0),
    m_velocityTolerance = () -> Preferences.getInstance().getDouble("Velocity/Tolerance", 1),

    m_aimerKp = () -> Preferences.getInstance().getDouble("Aimer Angle/kP", 0),
    m_aimerKi = () -> Preferences.getInstance().getDouble("Aimer Angle/kI", 0),
    m_aimerKd = () -> Preferences.getInstance().getDouble("Aimer Angle/kD", 0),
    m_aimerSetPoint = () -> Preferences.getInstance().getDouble("Aimer Angle/SetPoint", 0),
    m_aimerTolerance = () -> Preferences.getInstance().getDouble("Aimer Angle/Tolerance", 1),

    m_shooterKp = () -> Preferences.getInstance().getDouble("Shooter Speed/kP", 0),
    m_shooterKi = () -> Preferences.getInstance().getDouble("Shooter Speed/kI", 0),
    m_shooterKd = () -> Preferences.getInstance().getDouble("Shooter Speed/kD", 0),
    m_shooterSetPoint = () -> Preferences.getInstance().getDouble("Shooter Speed/SetPoint", 0),
    m_shooterTolerance = () -> Preferences.getInstance().getDouble("Shooter Speed/Tolerance", 1),

    

    m_rolletaKp = () -> Preferences.getInstance().getDouble("Rolleta/kP", 0),
    m_rolletaKi = () -> Preferences.getInstance().getDouble("Rolleta/kI", 0),
    m_rolletaKd = () -> Preferences.getInstance().getDouble("Rolleta/kD", 0),
    m_rolletaSetPoint = () -> Preferences.getInstance().getDouble("Rolleta/SetPoint", 0),
    m_rolletaTolerance = () -> Preferences.getInstance().getDouble("Rolleta/Tolerance", 1);
    
    public static Supplier<Boolean> 
    m_angleActivate = 
    () -> Preferences.getInstance().getBoolean("Angle/Activate", true),
    
    m_distanceActivate =     
    () -> Preferences.getInstance().getBoolean("Distance/Activate", true),

    m_velocityActivate =     
    () -> Preferences.getInstance().getBoolean("Velocity/Activate", true),

    m_aimerActivate =     
    () -> Preferences.getInstance().getBoolean("Aimer Angle/Activate", true),
    
    m_shooterActivate =     
    () -> Preferences.getInstance().getBoolean("Shooter Speed/Activate", true),
    
    m_rolletaActivate =     
    () -> Preferences.getInstance().getBoolean("Rolleta/Activate", true);
    
    public static Consumer<Double> 
    m_angleSetpointSetter = (setPoint) -> Preferences.getInstance().putDouble("Angle/SetPoint", setPoint),
    m_distanceSetpointSetter = (setPoint) -> Preferences.getInstance().putDouble("Distance/SetPoint", setPoint),
    m_velocitySetpointSetter = (setPoint) -> Preferences.getInstance().putDouble("Velocity/SetPoint", setPoint),
    m_aimerSetpointSetter = (setPoint) -> Preferences.getInstance().putDouble("Aimer Angle/SetPoint", setPoint),
    m_shooterSetpointSetter = (setPoint) -> Preferences.getInstance().putDouble("Shooter Speed/SetPoint", setPoint),
    m_rolletaSetpointSetter = (setPoint) -> Preferences.getInstance().putDouble("Rolleta/SetPoint", setPoint);
    
    public static Consumer<Boolean> 
    m_activateAngle = (toAble) -> Preferences.getInstance().putBoolean("Angle/Activate", toAble),
    m_activateDistance = (toAble) -> Preferences.getInstance().putBoolean("Distance/Activate", toAble),
    m_activateVelocity = (toAble) -> Preferences.getInstance().putBoolean("Velocity/Activate", toAble),
    m_activateAimer = (toAble) -> Preferences.getInstance().putBoolean("Aimer Angle/Activate", toAble),
    m_activateShooter = (toAble) -> Preferences.getInstance().putBoolean("Shooter Speed/Activate", toAble),
    m_activateRolleta = (toAble) -> Preferences.getInstance().putBoolean("Rolleta/Activate", toAble);
    
    public static PIDController 
    m_displacmentChassisPID = new PIDController(0, 0, 0),
    m_angleChassisPID = new PIDController(0, 0, 0),
    m_speedChassisPID = new PIDController(0, 0, 0),
    m_aimerPID = new PIDController(0, 0, 0),
    m_shooterPID = new PIDController(0, 0, 0),
    m_rolletaPID = new PIDController(0, 0, 0);
    
    public final double 
    DISTANCE_PER_PULSE_LEFT = -0.2298, //cm // 1/15
    DISTANCE_PER_PULSE_RIGHT = -0.23075, //cm // 1/15
    ANGLE_PER_PULSE_AIMER = 1, //
    DISTANCE_PER_PULSE_WHEELS = -0.5 * Math.PI, //1
    DISTANCE_PER_PULSE_ROLLETA = 5.89914E-05, //1
    ROLLETA_PERIMITER  = 0.81 * Math.PI;
    
    public static Joystick m_joystick = new Joystick(0);
    
    @Override
    public void robotInit() {
      
      m_rightChassisTalon.setSelectedSensorPosition(0);
      m_leftEncoder.reset();
      Preferences.getInstance().putDouble("Angle/kP", 0);
      Preferences.getInstance().putDouble("Angle/kI", 0);
      Preferences.getInstance().putDouble("Angle/kD", 0);
      Preferences.getInstance().putDouble("Angle/SetPoint", 90);
      Preferences.getInstance().putDouble("Angle/Tolerance", 1);
      Preferences.getInstance().putBoolean("Angle/Activate", true);
      
      Preferences.getInstance().putDouble("Distance/kP", 0);
      Preferences.getInstance().putDouble("Distance/kI", 0);
      Preferences.getInstance().putDouble("Distance/kD", 0);
      Preferences.getInstance().putDouble("Distance/SetPoint", 90);
      Preferences.getInstance().putDouble("Distance/Tolerance", 1);
      Preferences.getInstance().putBoolean("Distance/Activate", true);

      Preferences.getInstance().putDouble("Velocity/kP", 0);
      Preferences.getInstance().putDouble("Velocity/kI", 0);
      Preferences.getInstance().putDouble("Velocity/kD", 0);
      Preferences.getInstance().putDouble("Velocity/SetPoint", 90);
      Preferences.getInstance().putDouble("Velocity/Tolerance", 1);
      Preferences.getInstance().putBoolean("Velocity/Activate", true);

      
      Preferences.getInstance().putDouble("Aimer Angle/kP", 0);
      Preferences.getInstance().putDouble("Aimer Angle/kI", 0);
      Preferences.getInstance().putDouble("Aimer Angle/kD", 0);
      Preferences.getInstance().putDouble("Aimer Angle/SetPoint", 90);
      Preferences.getInstance().putDouble("Aimer Angle/Tolerance", 1);
      Preferences.getInstance().putBoolean("Aimer Angle/Activate", true);

      Preferences.getInstance().putDouble("Shooter Speed/kP", 0);
      Preferences.getInstance().putDouble("Shooter Speed/kI", 0);
      Preferences.getInstance().putDouble("Shooter Speed/kD", 0);
      Preferences.getInstance().putDouble("Shooter Speed/SetPoint", 90);
      Preferences.getInstance().putDouble("Shooter Speed/Tolerance", 1);
      Preferences.getInstance().putBoolean("Shooter Speed/Activate", true);
      
      Preferences.getInstance().putDouble("Rolleta/kP", 0);
      Preferences.getInstance().putDouble("Rolleta/kI", 0);
      Preferences.getInstance().putDouble("Rolleta/kD", 0);
      Preferences.getInstance().putDouble("Rolleta/SetPoint", 90);
      Preferences.getInstance().putDouble("Rolleta/Tolerance", 1);
      Preferences.getInstance().putBoolean("Rolleta/Activate", true);
      
      m_leftTalon.setInverted(true);
      m_leftChassisA.setInverted(true);
      m_leftChassisB.setInverted(true);
      
      m_gyro.calibrate();
      
      m_leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE_LEFT);
      m_shooterEncoder.setDistancePerPulse(DISTANCE_PER_PULSE_WHEELS);

      
      
      // new JoystickButton(m_joystick, 1).whenPressed(new InstantCommand(() -> m_setSetPoint.accept(m_angleSetPoint.get() - 10)));
      // new JoystickButton(m_joystick, 2).whenPressed(new InstantCommand(() -> m_activateAngle.accept(!m_angleActivate.get())));
      // new JoystickButton(m_joystick, 3).whenPressed(new InstantCommand(() -> m_setSetPoint.accept(m_angleSetPoint.get() + 10)));
      // new JoystickButton(m_joystick, 4).whenPressed(new InstantCommand(() -> m_gyro.reset()));

      // m_shooter.set(0.8);
    }
    
    public double getRightDistance() {
      return m_rightChassisTalon.getSelectedSensorPosition() * DISTANCE_PER_PULSE_RIGHT;
    }
    
    public double getRightTicks() {
      return m_rightChassisTalon.getSelectedSensorPosition();
    }
    
    public double getAimerAngle() {
      return m_aimer.getSelectedSensorPosition() * ANGLE_PER_PULSE_AIMER;
    }
  
    public double getWheelRotations() {
      return m_shooterEncoder.getDistance();
    }
  
    public double getWheelSpeeds() {
      return m_shooterEncoder.getRate();
    }

    public double getAimerTicks() {
      return m_aimer.getSelectedSensorPosition();
    }

    public double getRolletaTicks() {
      return m_rolleta.getSelectedSensorPosition();
    }

    public double getRolletaDistance() {
      return m_rolleta.getSelectedSensorPosition() * DISTANCE_PER_PULSE_ROLLETA;
    }

    public double getRolletaAngle() {
      return 
        (getRolletaDistance() % ROLLETA_PERIMITER) / ROLLETA_PERIMITER;
    }

    @Override
    public void robotPeriodic() {
      SmartDashboard.putNumber(
        "ANGLE", getRolletaAngle());
        rolletarPeriodic();
    }

    
    @Override
    public void teleopPeriodic() {
      driveLeft(new Joystick(1).getY() * 0.5);
      driveRight(new Joystick(2).getY() * 0.5);
    }
    
    
    
    @Override
    public void testPeriodic() {
      // driveRight(0.3);
      
      // anglePeriodic();
      
      // aimerPeriodic();

      
    }
    
    @Override
    public void testInit() {
      m_leftEncoder.reset();
      m_rightChassisTalon.setSelectedSensorPosition(0);
    }
  
    public void driveRight(double speed) {
      m_rightChassisTalon.set(speed);
      m_rightChassisA.set(speed);
      m_rightChassisB.set(speed);
    }

    public void driveLeft(double speed) {      
      m_leftTalon.set(speed);
      m_leftChassisA.set(speed);
      m_leftChassisB.set(speed);
    }

    public void resetChassis() {
      m_leftEncoder.reset();
      m_rightChassisTalon.setSelectedSensorPosition(0);

    }
    
    public void drivePeriodic() {
    }
    
   public void chassisDistancePeriodic() {
    
    SmartDashboard.putNumber("error", m_displacmentChassisPID.getPositionError());

    double measurement = (getRightDistance() + m_leftEncoder.getDistance()) / 2;
    SmartDashboard.putNumber("Measurement", measurement);

    double output = m_displacmentChassisPID.calculate(measurement);
    SmartDashboard.putNumber("Output", output);

    if(m_distanceActivate.get()) {

      driveRight(m_displacmentChassisPID.calculate(measurement));
      driveLeft(m_displacmentChassisPID.calculate(measurement));
    }

    else {
      driveLeft(0);
      driveRight(0);
    }
  }

  public void chassisAnglePeriodic() {
    
    if (m_joystick.getRawButtonPressed(1)) m_angleSetpointSetter.accept(m_angleSetPoint.get() - 10);
    if (m_joystick.getRawButtonPressed(2)) m_activateAngle.accept(!m_angleActivate.get());

    if (m_joystick.getRawButtonPressed(3)) m_angleSetpointSetter.accept(m_angleSetPoint.get() + 10);
    if (m_joystick.getRawButtonPressed(4)) {
      m_gyro.reset();
      m_angleChassisPID.reset();
    }
  
    m_angleChassisPID.setSetpoint(m_angleSetPoint.get());
    m_angleChassisPID.setTolerance(m_angleTolerance.get());
    m_angleChassisPID.setPID(m_angleKp.get(), m_angleKi.get(), m_angleKd.get());
    
    SmartDashboard.putNumber("error", m_angleChassisPID.getPositionError());


    if(m_angleActivate.get()) {
      driveRight(
        (m_angleChassisPID.calculate(
          m_gyro.getAngle()))
            % 360);

      driveLeft(((m_angleChassisPID.calculate(m_gyro.getAngle()))%360)*-1);
    }

    else {
      driveLeft(0);
      driveRight(0);
    }
  }

    

  public void leftVelocityPeriodic() {
    
    if (m_joystick.getRawButtonPressed(1)) m_velocitySetpointSetter.accept(m_velocitySetPoint.get() - 10);
    if (m_joystick.getRawButtonPressed(2)) m_activateVelocity.accept(!m_velocityActivate.get());

    if (m_joystick.getRawButtonPressed(3)) m_velocitySetpointSetter.accept(m_velocitySetPoint.get() + 10);
    if (m_joystick.getRawButtonPressed(4)) {
      resetChassis();
      m_speedChassisPID.reset();
    }
  
    m_speedChassisPID.setSetpoint(m_velocitySetPoint.get());
    m_speedChassisPID.setTolerance(m_velocityTolerance.get());
    m_speedChassisPID.setPID(m_velocityKp.get(), m_velocityKi.get(), m_velocityKd.get());
    
    SmartDashboard.putNumber("error", m_speedChassisPID.getPositionError());


    if(m_velocityActivate.get()) {
      driveLeft((m_speedChassisPID.calculate(m_leftEncoder.getRate())));
    }

    else driveLeft(0);
  }

  public void rightVelocityPeriodic() {
    
    if (m_joystick.getRawButtonPressed(1)) m_velocitySetpointSetter.accept(m_velocitySetPoint.get() - 10);
    if (m_joystick.getRawButtonPressed(2)) m_activateVelocity.accept(!m_velocityActivate.get());

    if (m_joystick.getRawButtonPressed(3)) m_velocitySetpointSetter.accept(m_velocitySetPoint.get() + 10);
    if (m_joystick.getRawButtonPressed(4)) {
      resetChassis();
      m_speedChassisPID.reset();
    }
  
    m_speedChassisPID.setSetpoint(m_velocitySetPoint.get());
    m_speedChassisPID.setTolerance(m_velocityTolerance.get());
    m_speedChassisPID.setPID(m_velocityKp.get(), m_velocityKi.get(), m_velocityKd.get());
    
    SmartDashboard.putNumber("error", m_speedChassisPID.getPositionError());


    if(m_velocityActivate.get()) {
      driveRight((m_speedChassisPID.calculate(m_rightChassisTalon.getSelectedSensorVelocity())));
    }

    else driveRight(0);
  }

  public void aimerPeriodic() {
    
    if (m_joystick.getRawButtonPressed(1)) m_aimerSetpointSetter.accept(m_aimerSetPoint.get() - 10);
    if (m_joystick.getRawButtonPressed(2)) m_activateAimer.accept(!m_aimerActivate.get());

    if (m_joystick.getRawButtonPressed(3)) m_aimerSetpointSetter.accept(m_velocitySetPoint.get() + 10);
    if (m_joystick.getRawButtonPressed(4)) {
      m_aimer.setSelectedSensorPosition(0);
      m_aimerPID.reset();
    }
  
    m_aimerPID.setSetpoint(m_aimerSetPoint.get());
    m_aimerPID.setTolerance(m_aimerTolerance.get());
    m_aimerPID.setPID(m_aimerKp.get(), m_aimerKi.get(), m_aimerKd.get());
    
    SmartDashboard.putNumber("error", m_aimerPID.getPositionError());


    if(m_aimerActivate.get()) {
      m_aimer.set(m_aimerPID.calculate(getAimerAngle()));
    }

    else {
      m_aimer.set(0);
    }
  }
  
  public void shooterPeriodic() {
    
    if (m_joystick.getRawButtonPressed(1)) m_shooterSetpointSetter.accept(m_shooterSetPoint.get() - 10);
    if (m_joystick.getRawButtonPressed(2)) m_activateShooter.accept(!m_shooterActivate.get());

    if (m_joystick.getRawButtonPressed(3)) m_shooterSetpointSetter.accept(m_shooterSetPoint.get() + 10);
    if (m_joystick.getRawButtonPressed(4)) {
      m_shooterEncoder.reset();
      m_shooterPID.reset();
    }

    m_shooterPID.setSetpoint(m_shooterSetPoint.get());
    m_shooterPID.setTolerance(m_shooterTolerance.get());
    m_shooterPID.setPID(m_shooterKp.get(), m_shooterKi.get(), m_shooterKd.get());
    
    SmartDashboard.putNumber("error", m_shooterPID.getVelocityError());

    if(m_shooterActivate.get()) {
      if (!m_shooterPID.atSetpoint()) {
        m_shooter.set(m_shooterPID.calculate(getWheelSpeeds()));
      }
    }

    else {
      m_shooter.set(0);
    }   
  }  

  public void rolletarPeriodic() {

    //Wheel diameter 7.62 cm = 0.0762 m
    
    if (m_joystick.getRawButtonPressed(1)) m_rolletaSetpointSetter.accept(m_rolletaSetPoint.get() - 10);
    if (m_joystick.getRawButtonPressed(2)) m_activateRolleta.accept(!m_rolletaActivate.get());

    if (m_joystick.getRawButtonPressed(3)) m_rolletaSetpointSetter.accept(m_rolletaSetPoint.get() + 10);
    if (m_joystick.getRawButtonPressed(4)) {
      m_rolleta.setSelectedSensorPosition(0);
      m_rolletaPID.reset();
    }
  


    m_rolletaPID.setSetpoint(m_rolletaSetPoint.get());
    m_rolletaPID.setTolerance(m_rolletaTolerance.get());
    m_rolletaPID.setPID(m_rolletaKp.get(), m_rolletaKi.get(), m_rolletaKd.get());
    

    SmartDashboard.putNumber("error", m_rolletaPID.getPositionError());

    if(m_rolletaActivate.get()) {
        m_rolleta.set(m_rolletaPID.calculate(getRolletaAngle()));
    }

    else {
      m_rolleta.set(0);
    }
  }



}