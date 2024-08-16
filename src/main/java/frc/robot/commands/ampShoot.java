// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ampShoot extends Command {
  /** Creates a new shoot. */
  private final Shooter m_Shooter;
  private final Indexer m_Indexer;
  public ampShoot(Shooter runShooter, Indexer runIndexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = runShooter;
    m_Indexer = runIndexer;
    addRequirements(m_Shooter);
    // addRequirements(m_Indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Shooter.shootShooter(Constants.MechanismConstants.shooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Shooter.isAtSpeed()) {
      // m_Indexer.moveIndexer(Constants.MechanismConstants.ampShooterSpeed);
      m_Indexer.moveIndexer(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.motorOff();
    m_Indexer.motorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
