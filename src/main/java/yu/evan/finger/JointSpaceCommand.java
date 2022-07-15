package yu.evan.finger;

import us.ihmc.euclid.interfaces.Settable;

public class JointSpaceCommand implements Settable<JointSpaceCommand>
{
    private JointSpaceControlMode controlMode;
    private double proximalInput;
    private double intermediateInput;

    public JointSpaceCommand()
    {

    }

    public void setCommand(JointSpaceControlMode controlMode, double proximalInput, double intermediateInput)
    {
        this.controlMode = controlMode;
        this.proximalInput = proximalInput;
        this.intermediateInput = intermediateInput;
    }

    public JointSpaceControlMode getControlMode()
    {
        return controlMode;
    }

    public double getIntermediateInput()
    {
        return intermediateInput;
    }

    public double getProximalInput()
    {
        return proximalInput;
    }

    @Override
    public void set(JointSpaceCommand other) {
        this.controlMode = other.getControlMode();
        this.intermediateInput = other.getIntermediateInput();
        this.proximalInput = other.getProximalInput();
    }
}
