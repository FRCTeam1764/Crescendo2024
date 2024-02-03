package frc.robot.constants;

import frc.robot.common.CanPort;

public final class Constants {
    
    public static final String CANIVORE_NAME = "1764 canivore";
    public static final int PRESSURE_SENSOR_PORT =   0;


    public static final CanPort CLIMBER_MOTOR1 =     new CanPort(10); //right
    public static final CanPort CLIMBER_MOTOR2 =     new CanPort(20); //left

    public static final CanPort FLYWHEEL_MOTOR1 =     new CanPort(8); //right
    public static final CanPort FLYWHEEL_MOTOR2 =     new CanPort(36); //left
    public static final CanPort HOLDER_MOTOR =     new CanPort(33); //index, climber, roller

    public static final CanPort INTAKE_MOTOR =     new CanPort(21);
    public static final CanPort WRIST_MOTOR1 =     new CanPort(35);
    public static final CanPort WRIST_MOTOR2 =     new CanPort(26);


    public static final int HOLDER_BREAK_BEAM = 0;
    public static final int INTAKE_BREAK_BEAM = 9;

    public static final int CLIMBER_LIMITSWITCH_LEFT = 6;
    public static final int CLIMBER_LIMITSWITCH_RIGHT = 7;

    
    
 
    // public static final CanPort SIDE_INTAKE_MOTOR =     new CanPort(22);
    // public static final CanPort INTAKE_OPENER_MOTOR =   new CanPort(23);


}