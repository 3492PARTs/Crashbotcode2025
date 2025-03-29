// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  SparkMax leftfrontmotor;
  SparkMax rightfrontmotor;
  SparkMax leftrearmotor;
  SparkMax rightrearmotor;
  XboxController controller;
  SlewRateLimiter limiter;

  public Robot() {
    m_robotContainer = new RobotContainer();
    leftfrontmotor = new SparkMax(5, MotorType.kBrushless);
    rightfrontmotor = new SparkMax(23, MotorType.kBrushless);
    leftrearmotor = new SparkMax(6, MotorType.kBrushless);
    rightrearmotor = new SparkMax(16, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    limiter = new SlewRateLimiter(0.5);

    globalConfig.idleMode(IdleMode.kBrake);
    leftfrontmotor.configure(globalConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightfrontmotor.configure(globalConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftrearmotor.configure(globalConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightrearmotor.configure(globalConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = new XboxController(0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double forward = controller.getLeftY();
    double rot = controller.getRightX();
    forward = limiter.calculate(forward);
    

    leftfrontmotor.set(forward-rot);
    leftrearmotor.set(forward-rot);
    rightfrontmotor.set(forward+rot);
    rightrearmotor.set(forward+rot);

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
