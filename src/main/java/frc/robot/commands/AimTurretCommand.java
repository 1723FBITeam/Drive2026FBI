package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurretCommand extends Command {

private final TurretSubsystem turret;

 public AimTurretCommand(TurretSubsystem turret) {
        System.out.println("Command STARTED!!!!!");
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        
        turret.aimAtTag();
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs forever as default command
    }


}

