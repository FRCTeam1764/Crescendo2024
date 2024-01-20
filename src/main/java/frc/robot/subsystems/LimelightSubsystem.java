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
  private boolean aprilTagViable;
  private RobotContainer m_robotContainer;
  Alliance alliance;
  private Boolean enable = true;
  private Pose2d botpose;
  public LimelightSubsystem(String name) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    this.table = NetworkTableInstance.getDefault().getTable(name);

    this.aprilTagViable = aprilTagViable;

    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tv = table.getEntry("tv");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     //read values periodically
    double x = this.tx.getDouble(0.0);
    double y = this.ty.getDouble(0.0);
    double area = this.ta.getDouble(0.0);


    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    
  }


  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }
  public double getHorizontalAngleOfErrorDegrees(){
    //+1 is a fudge factor cor camera mounting
    return getTx().getDouble(0.0) + Constants.HORIZONTAL_OFFSET;
  }

  public double getVerticalAngleOfErrorDegrees(){
    //+1 is a fudge factor cor camera mounting
    return getTy().getDouble(0.0) +Constants.VERTICAL_OFFSET;
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

  public double getTxAngleRadians() {
    return Units.degreesToRadians(tx.getDouble(0));
  }

  public double getTargetAngleRadians() {
    return getTxAngleRadians();//+m_robotContainer.getDrivetrain().getHeadingRadians();
  }
  public boolean hasTarget(){
    SmartDashboard.putNumber("tv; ", tv.getDouble(0));
    return tv.getDouble(0) != 0;
  }
}