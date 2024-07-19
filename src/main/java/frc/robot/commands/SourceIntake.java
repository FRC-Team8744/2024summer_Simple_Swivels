// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class SourceIntake extends Command {
  private final Indexer m_index;
  private final Shooter m_shooter;
  /** Creates a new SourceIntake. */
  public SourceIntake(Indexer ind, Shooter sh) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_index = ind;
    m_shooter = sh;
    addRequirements(m_index);
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.reverseShooter(Constants.MechanismConstants.shooterSpeed);
    m_index.reverseIndexer(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.motorOff();
    m_index.motorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
