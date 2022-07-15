package yu.evan.finger.communication;

import us.ihmc.robotics.time.CountdownTimer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import yu.evan.finger.*;

public class TestSerialRead
{
    SerialCommunicationInterface communicationInterface;
    String comPort = "COM7";
    int baud = 115200;
    JointSpaceSensorData sensorData;

    CountdownTimer printTimer;

    public TestSerialRead()
    {
        printTimer = new CountdownTimer("countdown", new YoRegistry("serialTest"));
        printTimer.startCountdown(0.01);
        sensorData = new JointSpaceSensorData();
        communicationInterface = new SerialCommunicationInterface(comPort, baud);
        while(true)
        {
            if (printTimer.isRinging())
            {
                printTimer.restartCountdown();
                sensorData.set(communicationInterface.getJointSensorData());
                System.out.println(sensorData.getProximalAngle() + ", "
                        + sensorData.getProximalVelocity() + ", "
                        + sensorData.getProximalTorque() + ", "
                        + sensorData.getIntermediateAngle() + ", "
                        + sensorData.getIntermediateVelocity() + ", "
                        + sensorData.getIntermediateTorque());
//                System.out.println(Integer.toString((int) sensorData.getProximalAngle(), 2) + ", "
//                        + Integer.toString((int) sensorData.getProximalVelocity(), 2) + ", "
//                        + Integer.toString((int) sensorData.getProximalTorque(), 2) + ", "
//                        + Integer.toString((int) sensorData.getIntermediateAngle(), 2) + ", "
//                        + Integer.toString((int) sensorData.getIntermediateVelocity(), 2) + ", "
//                        + Integer.toString((int) sensorData.getIntermediateTorque(), 2));
            }
        }
    }

    public static void main(String[] args)
    {
        new TestSerialRead();
    }
}
