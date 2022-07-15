package yu.evan.finger;

import us.ihmc.euclid.interfaces.Settable;

public class JointSpaceSensorData implements Settable<JointSpaceSensorData>
{
    private double intermediateAngle;
    private double intermediateVelocity;
    private double intermediateTorque;
    private double proximalAngle;
    private double proximalVelocity;
    private double proximalTorque;

    public JointSpaceSensorData()
    {

    }

    public void set( double proximalAngle, double proximalVelocity, double proximalTorque,
                     double intermediateAngle, double intermediateVelocity, double intermediateTorque)
    {
        this.intermediateAngle = intermediateAngle;
        this.intermediateVelocity = intermediateVelocity;
        this.intermediateTorque = intermediateTorque;

        this.proximalAngle = proximalAngle;
        this.proximalVelocity = proximalVelocity;
        this.proximalTorque = proximalTorque;
    }

    public double getIntermediateAngle()
    {
        return intermediateAngle;
    }

    public double getIntermediateVelocity()
    {
        return intermediateVelocity;
    }

    public double getIntermediateTorque()
    {
        return intermediateTorque;
    }

    public double getProximalAngle()
    {
        return proximalAngle;
    }

    public double getProximalVelocity()
    {
        return proximalVelocity;
    }

    public double getProximalTorque()
    {
        return proximalTorque;
    }

    @Override
    public void set(JointSpaceSensorData other)
    {
        this.intermediateAngle = other.intermediateAngle;
        this.intermediateVelocity = other.intermediateVelocity;
        this.intermediateTorque = other.intermediateTorque;

        this.proximalAngle = other.proximalAngle;
        this.proximalVelocity = other.proximalVelocity;
        this.proximalTorque = other.proximalTorque;
    }
}
