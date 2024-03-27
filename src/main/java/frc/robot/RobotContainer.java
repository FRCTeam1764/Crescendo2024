package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.AutoGroundPickUp;
import frc.robot.commands.AutoCommands.AutoGroundPickUpShort;
import frc.robot.commands.AutoCommands.AutoShoot;
import frc.robot.commands.AutoCommands.LimeLightAuto;
import frc.robot.commands.AutoCommands.LockOnAprilTagAuto;
import frc.robot.commands.ComplexCommands.AmpIntakeCommand;
import frc.robot.commands.ComplexCommands.ClimbDownCommand;
import frc.robot.commands.ComplexCommands.GoToAmpPositionCommand;
import frc.robot.commands.ComplexCommands.GroundPickup;
import frc.robot.commands.ComplexCommands.ScoreAmpCommand;
import frc.robot.commands.ComplexCommands.ScoreTrapCommand;
import frc.robot.commands.ComplexCommands.Shoot;
import frc.robot.commands.ComplexCommands.ShootRamp;
import frc.robot.commands.ComplexCommands.ShootRampShoot;
import frc.robot.commands.ComplexCommands.SpitOutNoteCommand;
import frc.robot.commands.ComplexCommands.indexRingCommand;
import frc.robot.commands.ComplexCommands.returnGroundPickUp;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TeleopDrive;
import frc.robot.commands.SimpleCommands.ClimberCommand;
import frc.robot.commands.SimpleCommands.IntakeCommand;
import frc.robot.commands.SimpleCommands.RollerCommand;
import frc.robot.commands.SimpleCommands.ShooterCommand;
import frc.robot.commands.SimpleCommands.ShooterSpecial;
import frc.robot.commands.SimpleCommands.TestWrist;
import frc.robot.commands.SimpleCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.SwerveConstantsYAGSL;
import frc.robot.subsystems.*;
import frc.robot.libraries.external.control.Path;
import frc.robot.libraries.external.control.Trajectory;
import frc.robot.libraries.external.robot.input.JoystickAxis;
import frc.robot.state.RobotState;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final Joystick secondaryController = new Joystick(1); 

    /* Drive Controls */

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickAxis LeftTrigger = new JoystickAxis(driver, XboxController.Axis.kLeftTrigger.value);

    private final JoystickButton SpeakerLimelight = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton RingLimelight = new JoystickButton(driver, XboxController.Button.kB.value);

    /* CoPilot Buttons */

    private final JoystickButton shoot = new JoystickButton(secondaryController,
            XboxController.Button.kRightBumper.value);
        
    private final JoystickButton groundPickup = new JoystickButton(secondaryController,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton shootAmp = new JoystickButton(secondaryController, XboxController.Button.kA.value);
    private final JoystickButton shootTrap = new JoystickButton(secondaryController, XboxController.Button.kY.value);
    private final JoystickButton spitOut = new JoystickButton(secondaryController, XboxController.Button.kB.value);

    private final JoystickButton index = new JoystickButton(secondaryController, XboxController.Button.kX.value);

    private final POVButton climbRight = new POVButton(secondaryController, 90);
    private final POVButton climbLeft = new POVButton(secondaryController, 270);
    private final POVButton climbCenter = new POVButton(secondaryController, 0);
    private final POVButton climbDown = new POVButton(secondaryController, 180);

    private final JoystickButton ZeroLeftArm = new JoystickButton(secondaryController,
            XboxController.Button.kBack.value);
    private final JoystickButton ZeroRightArm = new JoystickButton(secondaryController,
            XboxController.Button.kStart.value);

    /* Subsystems */

    public RobotState robotState = new RobotState(driver);

  
/*
 * hiya, head programmer at frc 1764 here
 * thanks for chceking out the code further generations/other teams
 * i do NOT recommend using this as a refrence for anything, pls check out 5013 or 9410's github instead (i stole their code)
 *  i wanna quit 
 */

    // private final Swerve s_Swerve = new Swerve();
     private  SendableChooser<Command> autoChooser;
private final Music THEMUSIC = new Music();
    private final Superstructure superstructure = new Superstructure();
    
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(THEMUSIC);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(robotState.intakeState);
    private final Shooter shooter = new Shooter(THEMUSIC);

      private final SwerveSubsystem s_Swerve = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve/falcon"),THEMUSIC);
    // Limelights
    // 3 is front intake
    //2 is back shooter
    //4,7,15,16,14,12,11,13,6,5 - tags that 2 should see
    private  LimelightSubsystem limelight3 = new LimelightSubsystem("limelight-three", 1,s_Swerve);
    private  LimelightSubsystem limelight2 = new LimelightSubsystem("limelight-two",0,s_Swerve);

    private Trajectory[] trajectories;

    public RobotContainer() {

        // teleop drive for yagsl
    limelight3.setPipeline(1);

        s_Swerve.setDefaultCommand(
                new TeleopDrive(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> !robotCentric.getAsBoolean()));

        configAutoCommands();
        configurePilotButtonBindings();
        configureCoPilotButtonBindings();

         autoChooser =  AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
        
    }

    private void configurePilotButtonBindings() {

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        // limelighs
        //a
        SpeakerLimelight.whileTrue(new LockOnAprilTag(s_Swerve, limelight2, 0, driver,true));
        //b
        RingLimelight.whileTrue(new LockOnAprilTag(s_Swerve, limelight3, 1, driver,false));

    }

    private void configureCoPilotButtonBindings() {
        //a button
        /* 
        shootAmp.whileTrue(new GoToAmpPositionCommand(robotState.intakeState, intakeSubsystem, shooter));
        shootAmp.onFalse(new AmpIntakeCommand(shooter,intakeSubsystem,robotState.intakeState));
        //y button
        shootTrap.whileTrue(new ShootRamp(shooter, intakeSubsystem));
        shootTrap.onFalse(new ShootRampShoot(shooter,intakeSubsystem));
        // left bumper
        groundPickup.whileTrue(new GroundPickup(shooter, intakeSubsystem,
                robotState.intakeState));
        groundPickup.onFalse(new returnGroundPickUp(intakeSubsystem, shooter,
                robotState.intakeState));

        // right bumper
        shoot.onTrue(new Shoot(shooter, intakeSubsystem));

        // x button
        index.whileTrue(new indexRingCommand(shooter, intakeSubsystem));

        // b button
         spitOut.whileTrue(new SpitOutNoteCommand(shooter, intakeSubsystem,
         robotState.intakeState));

            */

            shoot.whileTrue(new TestWrist(intakeSubsystem,.1,false));

        // dpad (bane of humanity) 1 = left 2 = right
        climbLeft.toggleOnTrue(new ClimberCommand(climberSubsystem, -140, -100));
        climbRight.toggleOnTrue(new ClimberCommand(climberSubsystem, -100, -140));
        climbCenter.toggleOnTrue(new ClimberCommand(climberSubsystem, -100, -100));
        // replcae
        climbDown.toggleOnTrue(new ClimbDownCommand(climberSubsystem));

        // climbDown.whileTrue(new testClimberLeft(climberSubsystem,.2));
        ZeroRightArm.whileTrue(new testClimberRight(climberSubsystem, .2));
        ZeroLeftArm.whileTrue(new testClimberLeft(climberSubsystem, .2));

/* 
        ZeroRightArm.whileTrue(new testClimberRight(climberSubsystem, .2));
        ZeroLeftArm.whileTrue(new testClimberLeft(climberSubsystem, .2));
//test commands - send climber back up 
        climbDown.whileTrue( new testClimberRight(climberSubsystem, -.2));
         climbCenter.whileTrue(new testClimberLeft(climberSubsystem, -.2));
         */

    }

    public void configAutoCommands() {
        /* 
        //oopsie, better practice next year or something :D
        NamedCommands.registerCommand("AutoScore", new AutoShoot(shooter,intakeSubsystem));
        NamedCommands.registerCommand("GroundPickUpAuto", new AutoGroundPickUp(s_Swerve,intakeSubsystem,robotState.intakeState,shooter));
        NamedCommands.registerCommand("GroundPickUpAutoShort", new AutoGroundPickUpShort(s_Swerve,intakeSubsystem,robotState.intakeState,shooter));
        NamedCommands.registerCommand("LimeLightRing", new LimeLightAuto(s_Swerve, limelight3, 1));
        NamedCommands.registerCommand("LimeLightSpeaker", new LimeLightAuto(s_Swerve, limelight2, 0));
        NamedCommands.registerCommand("IndexRing",     new ParallelDeadlineGroup(new simpleWaitCommand(.2),     new indexRingCommand(shooter, intakeSubsystem)));
        NamedCommands.registerCommand("WristDown", new WristCommand(intakeSubsystem,robotState.intakeState, CommandConstants.INTAKE_DOWN_ENCODERVALUE,true,false));
        NamedCommands.registerCommand("WristUp", new WristCommand(intakeSubsystem,robotState.intakeState, CommandConstants.INTAKE_UP_ENCODERVALUE,true,false));
        NamedCommands.registerCommand("SetInPeace", new SequentialCommandGroup(new WristCommand(intakeSubsystem,robotState.intakeState, CommandConstants.INTAKE_UP_ENCODERVALUE,true,false),new ParallelDeadlineGroup(new simpleWaitCommand(.2),     new indexRingCommand(shooter, intakeSubsystem))));
        */
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }


    public SwerveSubsystem getDrivetrainSubsystem(){
     return s_Swerve;
     }
    public double getPercentFromBattery(double speed){
        return speed * 12 / RobotController.getBatteryVoltage();
}

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public Joystick getsecondaryController() {
        return secondaryController;
    }

    public Joystick getPrimaryController() {
        return driver;
    }


    public Trajectory[] getTrajectories() {
        return trajectories;
    }
}
