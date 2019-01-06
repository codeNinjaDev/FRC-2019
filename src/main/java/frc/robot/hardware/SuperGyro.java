package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSourceType;

/*** Encoder Class that keeps track of total angle even when reset unless hard reset*/
public class SuperGyro extends AHRS {
    private double offsetAngle;
    private PIDSourceType m_pidSource;

    public SuperGyro(I2C.Port port) {
        super(port);
        offsetAngle = 0;
        m_pidSource = PIDSourceType.kDisplacement;

    }

    @Override
    public void reset() {
        offsetAngle = super.getAngle();
    }

    public void hardReset() {
        super.reset();
    }

    @Override
    public double getAngle() {
        return (super.getAngle() - offsetAngle);
    }

    /*** Get total angle from init */
    public double getTotalAngle() {
        return super.getAngle();
    }

    /**
     * Set which parameter of the encoder you are using as a process control
     * variable. The encoder class supports the rate and distance parameters.
     *
     * @param pidSource An enum to select the parameter.
     */
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        m_pidSource = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return m_pidSource;
    }

    /**
     * Implement the PIDSource interface.
     *
     * @return The current value of the selected source parameter.
     */
    @Override
    public double pidGet() {
        switch (m_pidSource) {
        case kDisplacement:
            return getAngle();
        case kRate:
            return super.getRate();
        default:
            return 0.0;
        }
    }

}