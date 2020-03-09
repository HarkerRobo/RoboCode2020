package frc.robot.commands.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Spinner;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class IndexerDefault extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER = 0.7;

    private Command spinIndexerForward = new SpinIndexer(0.5, false);
    private Command spinIndexerBackward = new SpinIndexer(0.8, true);

    public IndexerDefault() {
        addRequirements(Indexer.getInstance());
    }
    
    @Override
    public void initialize() {
        Indexer.getInstance().getAgitator().setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void execute() {
        if(OI.getInstance().getDriverGamepad().getRightTrigger() > OI.XBOX_TRIGGER_DEADBAND || OI.getInstance().getOperatorGamepad().getRightTrigger() > OI.XBOX_TRIGGER_DEADBAND) {
                Indexer.getInstance().getSolenoid().set(Indexer.CLOSED);
                spinIndexerForward.execute();
        } else if(OI.getInstance().getDriverGamepad().getLeftTrigger() > OI.XBOX_TRIGGER_DEADBAND || OI.getInstance().getOperatorGamepad().getLeftTrigger() > OI.XBOX_TRIGGER_DEADBAND) {
                Indexer.getInstance().getSolenoid().set(Indexer.OPEN);
                spinIndexerBackward.execute();
            // jamFlag = false;
        } else {
                spinIndexerBackward.end(false);
                spinIndexerForward.end(false);
        }  

        double output = MathUtil.mapJoystickOutput(OI.getInstance().getOperatorGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);
        
        // Spinner.getInstance().getSpinnerMotor().set(ControlMode.PercentOutput, OUTPUT_MULTIPLIER * output);
    }
    
    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().spinSpine(0);
        Indexer.getInstance().spinAgitator(0);
    }
}