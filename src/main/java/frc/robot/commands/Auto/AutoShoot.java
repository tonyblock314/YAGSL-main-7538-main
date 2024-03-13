// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoShoot extends SequentialCommandGroup {
  private Intake m_intake;
  private Shooter m_shooter;
  /** Creates a new AutoInake. */
  public AutoShoot(Shooter shooter, Intake intake) {
    this.m_intake = intake;
    this.m_shooter = shooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    

    addCommands(
      new AutoShootM(m_shooter, 0.9),
      new Wait(m_intake, 0.5),
      new Wait(m_shooter, 0.5),
      new AutoIntakeM(m_intake, 1),
      new Wait(m_intake, 0.5),
      new Wait(m_shooter, 0.75),
      new AutoIntakeM(m_intake, 0),
      new AutoShootM(m_shooter, 0)
    );
  }
}
