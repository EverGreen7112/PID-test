package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Trajectory
 */
public class Trajectory extends CommandBase {
    private PIDAction m_action;
    private PIDAction[] m_actions;
    private boolean m_finished = false;
    public Trajectory(PIDAction action, PIDAction... actions){
        m_action = action;
        actions = actions;
    }
       
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        while(m_action.calculate(m_action.m_type.m_input.get())!=0){
            m_action.m_type.m_output.accept(m_action.calculate(m_action.m_type.m_input.get()));   
        }
        for (int i = 0; i < m_actions.length; i++) {
            while(m_actions[i].calculate(m_actions[i].m_type.m_input.get())!=0){
                m_actions[i].m_type.m_output.accept(m_actions[i].calculate(m_actions[i].m_type.m_input.get()));
            }
        }
        m_finished =true;
        }
        @Override
        public boolean isFinished() {
            return m_finished;
        }
        
    }


    
