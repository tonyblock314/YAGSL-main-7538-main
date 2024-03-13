// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterSpin extends Command {
  private Shooter m_subsystem;
  private DoubleSupplier rightdepressed;
  private BooleanSupplier /*rightPressed,*/ bPressed;
  
  /** Creates a new Spin. */
  public ShooterSpin(Shooter subsystem, /*BooleanSupplier rightPressed,*/ DoubleSupplier rightdepressed, BooleanSupplier bPressed) {
    this.m_subsystem = subsystem;
    //this.rightPressed = rightPressed;
    this.rightdepressed = rightdepressed;
    this.bPressed = bPressed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right;
    //double speed = ShooterConstants.SHOOTER_SPEED;
    
    /*while (rightPressed.getAsBoolean())
    {
      m_subsystem.setShooterSpeed(-speed);
    }  */

    if (bPressed.getAsBoolean())
    {
      m_subsystem.setShooterSpeed(0);
    }

    if(rightdepressed.getAsDouble() < OperatorConstants.RT_DEADBAND) {
      right = 0;
    } else {
      right = rightdepressed.getAsDouble();
    }

    double varspeed = (right);

    m_subsystem.setShooterSpeed(varspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
