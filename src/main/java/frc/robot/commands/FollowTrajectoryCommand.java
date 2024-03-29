// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.libraries.external.control.Trajectory;

// public class FollowTrajectoryCommand extends Command {
//   private DrivetrainSubsystem drivetrain;
//   private Trajectory trajectory;
  
//   public FollowTrajectoryCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory) {
//     this.drivetrain = drivetrain;
//     this.trajectory = trajectory;

//     addRequirements(drivetrain);
//   }

//   @Override
//   public void initialize() {
//     drivetrain.getFollower().follow(trajectory);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.getFollower().cancel();
//   }

//   @Override
//   public boolean isFinished() {
//     return drivetrain.getFollower().getCurrentTrajectory().isEmpty();
//   }
// }