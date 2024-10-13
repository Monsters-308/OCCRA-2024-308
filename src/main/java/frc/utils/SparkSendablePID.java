package frc.utils;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SparkSendablePID implements Sendable, AutoCloseable {
    private static int instances = 0;
    
    private final SparkPIDController m_PIDController;

    private final ControlType m_controlType;

    private double m_p;

    private double m_i;

    private double m_d;

    private double m_iz;

    private double m_reference = 0;

    /**
     * Creates a PID controller that implements {@link Sendable}, allowing 
     * you to tune the SparkMax PID controllers from Shuffleboard.
     * @param PIDController The SparkMax PID controller.
     * @param controlType The control type to use when setting the controller from Shuffleboard.
     */
    public SparkSendablePID(SparkPIDController PIDController, ControlType controlType){
        m_PIDController = PIDController;
        m_controlType = controlType;

        m_p = m_PIDController.getP();
        m_i = m_PIDController.getI();
        m_d = m_PIDController.getD();
        m_iz = m_PIDController.getIZone();

        // No clue what this is for
        instances++;
        SendableRegistry.addLW(this, "SparkPIDController", instances);
    }

    public double getP() {
        return m_p;
    }

    public double getI() {
        return m_i;
    }

    public double getD() {
        return m_d;
    }

    public void setP(double p) {
        m_PIDController.setP(p);
        m_p = p;
    }

    public void setI(double i) {
        m_PIDController.setI(i);
        m_i = i;
    }

    public void setD(double d) {
        m_PIDController.setP(d);
        m_d = d;
    }

    /** Note: this returns the last setpoint used on the dashboard. */
    public double getSetpoint() {
        return m_reference;
    }

    public void setSetpoint(double setpoint) {
        m_reference = setpoint;
        m_PIDController.setReference(setpoint, m_controlType);
    }

    public double getIZone() {
        return m_iz;
    }

    /** Note: all negative values will be ignored. */
    public void setIZone(double iZone) {
        if(iZone >= 0){
            m_PIDController.setIZone(iZone);
            m_iz = iZone;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("izone", this::getIZone, this::setIZone);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }
}
