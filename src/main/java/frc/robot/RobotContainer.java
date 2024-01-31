package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.ComplexCommands.ClimbToPosition;
import frc.robot.commands.ComplexCommands.GoToAmpPositionCommand;
import frc.robot.commands.ComplexCommands.GroundPickup;
import frc.robot.commands.ComplexCommands.ScoreAmpCommand;
import frc.robot.commands.ComplexCommands.Shoot;
import frc.robot.commands.ComplexCommands.returnGroundPickUp;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TeleopDrive;
import frc.robot.commands.SimpleCommands.ClimberCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.*;
import frc.robot.libraries.external.control.Path;
import frc.robot.libraries.external.control.Trajectory;
import frc.robot.libraries.external.robot.input.JoystickAxis;
import frc.robot.state.RobotState;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickAxis LeftTrigger = new JoystickAxis(driver,XboxController.Axis.kLeftTrigger.value);

    private final JoystickButton SpeakerLimelight = new JoystickButton(driver, XboxController.Button.kA.value);
     private final JoystickButton RingLimelight = new JoystickButton(driver, XboxController.Button.kB.value);
private final JoystickButton AmpPhotonVision = new JoystickButton(driver, XboxController.Button.kX.value);
    
    /* CoPilot Buttons */

    private final JoystickButton Shoot = new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value);
    private final JoystickButton groundPickup = new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton climb = new JoystickButton(secondaryController, XboxController.Button.kY.value);

    private final JoystickButton scoreAmp = new JoystickButton(secondaryController, XboxController.Button.kX.value);

    private final POVButton climbRight = new POVButton(secondaryController,90);
    private final POVButton climbLeft = new POVButton(secondaryController,270);
    private final POVButton climbCenter = new POVButton(secondaryController,0);
    private final POVButton climbDown = new POVButton(secondaryController,180);


    /* Subsystems */
    public RobotState robotState = new RobotState(driver);
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
    private final Superstructure superstructure = new Superstructure();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(null);
    private final Shooter shooter = new Shooter();
    //Hunter was here
//Limelights
    private final LimelightSubsystem limelight3 = new LimelightSubsystem("Limelight3");
     private final LimelightSubsystem limelight2 = new LimelightSubsystem("Limelight2");
private final LimelightSubsystem thePi =  new LimelightSubsystem("TopCam");


    private Trajectory[] trajectories;


    public RobotContainer() {
  



        s_Swerve.setDefaultCommand(
            new TeleopDrive(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        configurePilotButtonBindings();
        configureCoPilotButtonBindings();
    }


    private void configurePilotButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        //limelighs
        SpeakerLimelight.whileTrue(new LockOnAprilTag(s_Swerve,limelight2,0));
        RingLimelight.whileTrue(new LockOnAprilTag(s_Swerve,limelight3,0));
        AmpPhotonVision.whileTrue(new LockOnAprilTag(s_Swerve,thePi,0));

    }
   private void configureCoPilotButtonBindings() {

        //left bumper
        groundPickup.whileTrue(new GroundPickup(shooter, intakeSubsystem, robotState.intakeState));
        groundPickup.onFalse(new returnGroundPickUp(intakeSubsystem, shooter, robotState.intakeState));
        //right bumper
        Shoot.onTrue(new Shoot(shooter,intakeSubsystem));
        //x button
        scoreAmp.whileTrue(new GoToAmpPositionCommand(robotState.intakeState,intakeSubsystem,shooter));
        scoreAmp.onFalse(new ScoreAmpCommand(intakeSubsystem, robotState.intakeState));
        
        //dpad (bane of humanity) 1 = left 2 = right 
        climbLeft.toggleOnTrue(new ClimbToPosition(climberSubsystem,100000,50000)
         );
        climbRight.toggleOnTrue( new ClimbToPosition(climberSubsystem,50000,100000)
         );
        climbCenter.toggleOnTrue(new ClimbToPosition(climberSubsystem,50000,50000)
        );
         climbDown.toggleOnTrue( new ClimbToPosition(climberSubsystem,0,0));
    }

     public Command getAutonomousCommand() {

         return null; 

     }


     public SwerveSubsystem getDrivetrainSubsystem() {
         return s_Swerve;
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
