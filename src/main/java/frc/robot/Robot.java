/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.subsystems.Drivetrain;
    
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
 *  hopper/indexer (2)
 *  control panel spinner (1)
 * 
 * @since 01/06/20
 */
public class Robot extends TimedRobot {

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code. This is where all subsystems are instantiated and 
     * default commands are set.
     */
    @Override
    public void robotInit() {
        Drivetrain.getInstance().setDefaultCommand(new SwerveManual());
        // BottomIntake.getInstance();
        // Shooter.getInstance().setDefaultCommand(new SpinShooterManual());
        // Indexer.getInstance();
        // Climber.getInstance();

        OI.getInstance();
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

        SmartDashboard.putString("Robot Type", RobotMap.IS_PRACTICE ? "Practice Bot" : "Comp Bot");

        // SmartDashboard.putNumber("TL Rise to Fall", Drivetrain.getInstance().getTopLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // SmartDashboard.putNumber("TR Rise to Fall", Drivetrain.getInstance().getTopRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // SmartDashboard.putNumber("BL Rise to Fall", Drivetrain.getInstance().getBackLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // SmartDashboard.putNumber("BR Rise to Fall", Drivetrain.getInstance().getBackRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());

        SmartDashboard.putNumber("TL Angle POS", Drivetrain.getInstance().getTopLeft().getAngleMotor().getSelectedSensorPosition() * 360.0 / 4096);
        SmartDashboard.putNumber("TR Angle POS", Drivetrain.getInstance().getTopRight().getAngleMotor().getSelectedSensorPosition() * 360.0 / 4096);
        SmartDashboard.putNumber("BL Angle POS", Drivetrain.getInstance().getBackLeft().getAngleMotor().getSelectedSensorPosition() * 360.0 / 4096);
        SmartDashboard.putNumber("BR Angle POS", Drivetrain.getInstance().getBackRight().getAngleMotor().getSelectedSensorPosition() * 360.0 / 4096);

        SmartDashboard.putNumber("TL Drive POS", Drivetrain.getInstance().getTopLeft().getDriveMotor().getSelectedSensorPosition() / Drivetrain.GEAR_RATIO);
        SmartDashboard.putNumber("TR Drive POS", Drivetrain.getInstance().getTopRight().getDriveMotor().getSelectedSensorPosition() / Drivetrain.GEAR_RATIO);
        SmartDashboard.putNumber("BL Drive POS", Drivetrain.getInstance().getBackLeft().getDriveMotor().getSelectedSensorPosition() / Drivetrain.GEAR_RATIO);
        SmartDashboard.putNumber("BR Drive POS", Drivetrain.getInstance().getBackRight().getDriveMotor().getSelectedSensorPosition() / Drivetrain.GEAR_RATIO);

        // SmartDashboard.putNumber("TL Angle Error", Drivetrain.getInstance().getTopLeft().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("TR Angle Error", Drivetrain.getInstance().getTopRight().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("BL Angle Error", Drivetrain.getInstance().getBackLeft().getAngleMotor().getClosedLoopError());
        // SmartDashboard.putNumber("BR Angle Error", Drivetrain.getInstance().getBackRight().getAngleMotor().getClosedLoopError());
        
        // SmartDashboard.putNumber("TL Drive Error", Drivetrain.getInstance().getTopLeft().getDriveMotor().getClosedLoopError() / Drivetrain.GEAR_RATIO);
        // SmartDashboard.putNumber("TR Drive Error", Drivetrain.getInstance().getTopRight().getDriveMotor().getClosedLoopError()/ Drivetrain.GEAR_RATIO);
        // SmartDashboard.putNumber("BL Drive Error", Drivetrain.getInstance().getBackLeft().getDriveMotor().getClosedLoopError() / Drivetrain.GEAR_RATIO);
        // SmartDashboard.putNumber("BR Drive Error", Drivetrain.getInstance().getBackRight().getDriveMotor().getClosedLoopError() / Drivetrain.GEAR_RATIO);

        SmartDashboard.putNumber("Pigeon Heading", Drivetrain.getInstance().getPigeon().getFusedHeading());
        SmartDashboard.putNumber("Pigeon Yaw", Drivetrain.getInstance().getPigeon().getYaw());
        SmartDashboard.putNumber("Pigeon Compass", Drivetrain.getInstance().getPigeon().getAbsoluteCompassHeading());
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
    }
}