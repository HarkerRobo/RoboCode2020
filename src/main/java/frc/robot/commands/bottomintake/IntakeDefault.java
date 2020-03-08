package frc.robot.commands.bottomintake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.OI;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.subsystems.BottomIntake;
import frc.robot.subsystems.Indexer;
import harkerrobolib.commands.IndefiniteCommand;

public class IntakeDefault extends IndefiniteCommand {

    private Command spinIndexerForward = new SpinIndexer(0.5, false);
    private Command spinIndexerBackward = new SpinIndexer(0.8, true);

    public IntakeDefault() {
        addRequirements(BottomIntake.getInstance());
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(OI.getInstance().getDriverGamepad().getRightTrigger() > OI.XBOX_TRIGGER_DEADBAND || OI.getInstance().getOperatorGamepad().getRightTrigger() > OI.XBOX_TRIGGER_DEADBAND) {
                BottomIntake.getInstance().spinIntake(0.35);
        } else if(OI.getInstance().getDriverGamepad().getLeftTrigger() > OI.XBOX_TRIGGER_DEADBAND || OI.getInstance().getOperatorGamepad().getLeftTrigger() > OI.XBOX_TRIGGER_DEADBAND) {
                BottomIntake.getInstance().spinIntake(-0.5);
        } else {
                BottomIntake.getInstance().spinIntake(0);
        }  
        if (BottomIntake.getInstance().isStalling() && !DriverStation.getInstance().isAutonomous())
            BottomIntake.getInstance().spinIntake(-0.3);
    }
   
    @Override
    public void end(boolean interrupted) {
        BottomIntake.getInstance().spinIntake(0);
    }
}