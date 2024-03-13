// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;
//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeSpin extends Command {
  private Intake m_subsystem;
  private BooleanSupplier leftPressed, rightPressed, aPressed;
  //private DoubleSupplier leftdepressed;
  
  /** Creates a new Spin. */
  public IntakeSpin(Intake subsystem, BooleanSupplier leftPressed, BooleanSupplier rightPressed, BooleanSupplier aPressed) {
    this.m_subsystem = subsystem;
    this.leftPressed = leftPressed;
    this.rightPressed = rightPressed;
    this.aPressed = aPressed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inspeed = IntakeConstants.INTAKE_STANDARD_UP;
    double downspeed = IntakeConstants.INTAKE_STANDARD_DOWN;

    //double left;
    /* 
    if (leftdepressed.getAsDouble() < OperatorConstants.LT_DEADBAND) {
      left = 0;
    } else {
      left = leftdepressed.getAsDouble();
    }
    */

    if (leftPressed.getAsBoolean()) {
      m_subsystem.setIntakeSpeed(-downspeed);
    } else if (rightPressed.getAsBoolean()) {
      m_subsystem.setIntakeSpeed(inspeed);
    } else {
      m_subsystem.setIntakeSpeed(0);
    }

    if (aPressed.getAsBoolean())
    {
      m_subsystem.setIntakeSpeed(0);
    }

    //double varspeed = left * IntakeConstants.SCALING_FACTOR_INTAKE;
    //m_subsystem.setIntakeSpeed(varspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
