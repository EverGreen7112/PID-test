package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * PIDAction
 */
public class PIDAction extends PIDController {
    public PIDType m_type;
    public PIDAction(double goal,PIDType type) {
        
        super(type.m_kP.get(), type.m_kI.get(), type.m_kP.get());
        m_type = type;
        super.setTolerance(type.m_tolerance.get());
        super.setSetpoint(goal);
    // TODO Auto-generated constructor stub
    }
    
    public enum PIDType{
    DISPLACEMENT(
    //put here constructor argument
    
    ),
    ROTATION(
   //put here constructor argument    
   
    ),
    SHOOTING(
    //put here constructor argument
    );
    public Supplier<Double> m_kP;
    public Supplier<Double> m_kI;
    public Supplier<Double> m_kD;
    public Supplier<Double> m_tolerance;
    public Supplier<Double> m_input;
    public Consumer<Double> m_output;
        private PIDType(Supplier<Double> kP, Supplier<Double> kI,Supplier<Double> kD,Supplier<Double> tolerance,Supplier<Double>  input,Consumer<Double> output){
            m_kP = kP;
            m_kI = kI;
            m_kD = kD;
            m_tolerance = tolerance;
            m_input =input;
            m_output = output;
        }
    }

    
}