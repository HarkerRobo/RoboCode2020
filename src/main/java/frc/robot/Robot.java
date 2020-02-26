/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.drivetrain.SwerveManualHeadingControl;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.commands.spinner.SpinnerManual;
import frc.robot.subsystems.BottomIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.util.Limelight;
import frc.robot.auto.Autons;
import frc.robot.auto.Trajectories;
    
/**
 * Has anyone heard of the team that ran out of code? (This is a real story)
 * 
 * SO had a rookie team while I was FTAAing who would come out to the field, run till about 15 seconds to the end of the match, then have their robot stop. 
 * They stepped back calmly from their controllers and just wait out the end of the game
 * I was confused, thought maybe they just were done with what they were doing in the match, and shrugged
 * 
 * they did it 3 more times before I was like "Ok somethings wierd" because they finally didnt look like they had hit end game like they wanted but the bot wasnt moving
 * 
 * I went over and was like what
 * 
 * "We ran out of code"
 * 
 * You What
 * 
 * "Oh we just ran out of code. It happens."
 * 
 * Let me see your code
 * 
 * I go in and its like 15 lines of code
 * 
 * copied and pasted as many times as they could fit on the rio
 * 
 * "Have you heard of this magical thing called a loop"
 * 
 * "A loop? Whats that?"
 * 
 * Subsystems (motor #):
 *  drive (8)
 *  climb (2)
 *  shooter (2)
 *  intake (1)
 *  indexer/indexer (2)
 *  control panel spinner (1)
 * 
 * Solenoids:
 *  Spinner (1 double solenoid)
 *  Climber (1 double solenoid)
 *  Intake (1 double solenoid)
 *  Indexer (1 single)
 *  Shooter Hood (1 single)
 *
 * @since 01/06/20
 */
public class Robot extends TimedRobot {
    private Compressor compressor;
    private boolean isTeleop;

    @Override
    public void teleopInit() {
        isTeleop = true;
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code. This is where all subsystems are instantiated and 
     * default commands are set.
     */
    @Override
    public void robotInit() {
        isTeleop = false;
        Drivetrain.getInstance().setDefaultCommand(new SwerveManualHeadingControl());
        // Spinner.getInstance().setDefaultCommand(new SpinnerManual());
        BottomIntake.getInstance();
        Indexer.getInstance();
        Shooter.getInstance();
        // Climber.getInstance();

        OI.getInstance();
        compressor = new Compressor();  

        Limelight.setLEDS(true);
        Indexer.getInstance().getSolenoid().set(Indexer.CLOSED);
        // Spinner.getInstance().getSolenoid().set(Spinner.DOWN);

        Autons.AutonCommands.BASELINE.toString();
        System.out.println(Autons.getAutonCommand());
        Autons.AutonCommands.BASELINE.toString();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putString("Robot Type", RobotMap.IS_COMP ? "Practice" : "Comp");

        SmartDashboard.putNumber("Indexer Current", Indexer.getInstance().getSpine().getOutputCurrent());
        SmartDashboard.putNumber("Shooter Current", Shooter.getInstance().getMaster().getOutputCurrent());
        SmartDashboard.putNumber("Intake Current", BottomIntake.getInstance().getTalon().getOutputCurrent());

        SmartDashboard.putNumber("distance", SpinShooterLimelight.medianFilter.calculate(Shooter.getInstance().getLimelightDistance()));

        SmartDashboard.putString("Indexer Piston", Indexer.getInstance().getSolenoid().get().toString());

        SmartDashboard.putNumber("Pigeon Heading", Drivetrain.getInstance().getPigeon().getFusedHeading());
        // SmartDashboard.putNumber("TL RisetoFall", Drivetrain.getInstance().getTopLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // SmartDashboard.putNumber("TR RisetoFall", Drivetrain.getInstance().getTopRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // SmartDashboard.putNumber("BL RisetoFall", Drivetrain.getInstance().getBackLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // SmartDashboard.putNumber("BR RisetoFall", Drivetrain.getInstance().getBackRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());

        // SmartDashboard.putString("Shooter Command", Shooter.getInstance().getCurrentCommand().getName() == null);
        // Indexer.getInstance().getSolenoid().set(Indexer.OPEN);

        // SmartDashboard.putNumber("TL Angle Error", Drivetrain.getInstance().getTopLeft().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("TR Angle Error", Drivetrain.getInstance().getTopRight().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("BL Angle Error", Drivetrain.getInstance().getBackLeft().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("BR Angle Error", Drivetrain.getInstance().getBackRight().getAngleMotor().getClosedLoopError());
        
        // SmartDashboard.putNumber("TL Drive Error", Drivetrain.getInstance().getTopLeft().getDriveMotor().getClosedLoopError() / Drivetrain.GEAR_RATIO);
        // SmartDashboard.putNumber("TR Drive Error", Drivetrain.getInstance().getTopRight().getDriveMotor().getClosedLoopError()/ Drivetrain.GEAR_RATIO);
        // SmartDashboard.putNumber("BL Drive Error", Drivetrain.getInstance().getBackLeft().getDriveMotor().getClosedLoopError() / Drivetrain.GEAR_RATIO);
        // SmartDashboard.putNumber("BR Drive Error", Drivetrain.getInstance().getBackRight().getDriveMotor().getClosedLoopError() / Drivetrain.GEAR_RATIO);

        // SmartDashboard.putNumber("TL Drive Current", Drivetrain.getInstance().getTopLeft().getDriveMotor().getStatorCurrent());

        // CommandScheduler.getInstance().schedule(Autons.getAutonCommand());
        // double pigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading()%360;
        // SmartDashboard.putNumber("PIGEON HEADING", pigeonHeading);

        // if(pigeonHeading < 90 || pigeonHeading > 270) {
        
        //     Limelight.setLEDS(false);
        // }
        // else {
        //     Limelight.setLEDS(true);
        // }

        // SmartDashboard.putString("cd color spinner current color", Spinner.getInstance().getCurrentColor().toString());
        SmartDashboard.putString("cd color spinner desired color", DriverStation.getInstance().getGameSpecificMessage());
        SmartDashboard.putBoolean("cd hood sol", Shooter.getInstance().getSolenoid().get() == Shooter.SHOOTER_HIGH_ANGLE);
        SmartDashboard.putBoolean("cd intake sol", BottomIntake.getInstance().getSolenoid().get() == BottomIntake.IN);
        SmartDashboard.putBoolean("cd indexer sol", Indexer.getInstance().getSolenoid().get() == Indexer.OPEN);
        // SmartDashboard.putBoolean("cd spinner sol", Spinner.getInstance().getSolenoid().get() == Spinner.UP);
        SmartDashboard.putNumber("cd pigeon angle", Drivetrain.getInstance().getPigeon().getFusedHeading());
        // SmartDashboard.putString("cd current auton", isTeleop ? Autons.curAuton.toString() : "Teleop Running");
        SmartDashboard.putBoolean("cd shooter isStalling", Shooter.getInstance().isStalling());
        SmartDashboard.putBoolean("cd intake isStalling", BottomIntake.getInstance().isStalling());
    }

    @Override
    public void autonomousInit() {
        Drivetrain.getInstance().getPigeon().setFusedHeading(180);
        // CommandScheduler.getInstance().schedule(Autons.getAutonCommand());
        CommandScheduler.getInstance().schedule(new SwerveDriveWithOdometryProfiling(Trajectories.Test.horizontalTrajectory, Rotation2d.fromDegrees(180)));
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();

        Limelight.setLEDS(true);
    }

    @Override
    public void disabledInit() {
        Limelight.setLEDS(false);

        CommandScheduler.getInstance().cancelAll();
    }
}