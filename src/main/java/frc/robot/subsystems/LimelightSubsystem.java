package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
//import webblib.util.RectanglePoseArea;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */
  // public static final RectanglePoseArea field =
  // new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private double horizontal_offset = 0;




  public LimelightSubsystem(String name) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    this.table = NetworkTableInstance.getDefault().getTable(name);
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tv = table.getEntry("tv");

  }

  public LimelightSubsystem(String name,double offset) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    this.table = NetworkTableInstance.getDefault().getTable(name);
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tv = table.getEntry("tv");
    this.horizontal_offset = offset;

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getHorizontalAngleOfErrorDegrees(){
    return getTx().getDouble(0.0) +horizontal_offset;

  }

  public double getVerticalAngleOfErrorDegrees(){
    return getTy().getDouble(0.0) +0;
  }


 public NetworkTableEntry getTx() {
    return tx;
  }

  public NetworkTableEntry getTy() {
    return ty;
  }

  public NetworkTableEntry getTa() {
    return ta;
  }

  public void setPipeline(int pipe){

table.getEntry("pipeline").setNumber(pipe);



  }

  public double getTxAngleRadians() {
    return Units.degreesToRadians(tx.getDouble(0));
  }

  public double getTargetAngleRadians() {
    return getTxAngleRadians();
  }
  public boolean hasTarget(){
    return tv.getDouble(0) != 0;
  }
}