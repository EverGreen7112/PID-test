package frc.robot;

/**
 * RobotMap
 */
public interface RobotMap {
    public interface MotorPorts{
        public static final int
        leftChassisUp = 0,            
        leftChassisBack = 1,
        leftChassisFront = 2,
        rightChassisUp = 3,
        rightChassisBack = 14,
        rightChassisFront = 15;
        
    }
    public interface AnalogPorts{
        public static final int
        gyro = 2;
    }
    
    public interface DigitalPorts {
        public static final int
            rightEncoderA = 0,
            rightEncoderB = 1,
            leftEncoderA = 2,
            leftEncoderB = 3;
        
    }
    
}