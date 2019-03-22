/*** Encoder Class that keeps track of total ticks even when reset */
package frc.robot.hardware;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SuperEncoder extends Encoder{
    private int offsetTicks;
    private double offsetDistance;
    private PIDSourceType m_pidSource;

    public SuperEncoder(int positionPort, int directionPort) {
        super(positionPort, directionPort);
        offsetTicks = 0;
        offsetDistance = 0;
        m_pidSource = PIDSourceType.kDisplacement;

    }

    @Override
    public void reset() {
        offsetTicks = super.get();
        offsetDistance = super.getDistance();
    }

    @Override 
    public double getDistance() {
        return (super.getDistance() - offsetDistance);
    }

    @Override 
    /** Gets current amount of ticks - offset */
    public int get() {
        return (super.get() - offsetTicks);
    }
    /*** Get total ticks from init */
    public int getTotalTicks() {
        return super.get();
    }

    /*** Get total distance from init */
    public double getTotalDistance() {
        return super.getDistance();
    }

    /**
   * Set which parameter of the encoder you are using as a process control variable. The encoder
   * class supports the rate and distance parameters.
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
        return getDistance();
      case kRate:
        return super.getRate();
      default:
        return 0.0;
    }
  }

}