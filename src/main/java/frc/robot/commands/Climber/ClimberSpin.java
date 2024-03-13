// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.climber.Climber;

public class ClimberSpin extends Command {
  private Climber m_subsystem;
  private DoubleSupplier LeftTilt, RightTilt;
  
  /** Creates a new Spin. */
  public ClimberSpin(Climber subsystem, DoubleSupplier LeftTilt, DoubleSupplier RightTilt) {
    this.m_subsystem = subsystem;
    this.LeftTilt = LeftTilt;
    this.RightTilt = RightTilt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left, right;
    if(Math.abs(LeftTilt.getAsDouble()) < OperatorConstants.LEFT_Y_DEADBAND_CLIMBER) {
      left = 0;
    } else {
      left = LeftTilt.getAsDouble();
    }
    if(Math.abs(RightTilt.getAsDouble()) < OperatorConstants.RIGHT_Y_DEADBAND_CLIMBER) {
      right = 0;
    } else {
      right = RightTilt.getAsDouble();
    }

    double lspeed = (left) * ClimberConstants.SCALING_FACTOR_CLIMBER;
    double rspeed = (right) * ClimberConstants.SCALING_FACTOR_CLIMBER;

    m_subsystem.setLeftClimberSpeed(lspeed);
    m_subsystem.setRightClimberSpeed(rspeed);
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
