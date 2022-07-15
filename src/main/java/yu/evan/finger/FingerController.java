package yu.evan.finger;

import us.ihmc.commons.MathTools;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import yu.evan.finger.communication.SerialCommunicationInterface;
import yu.evan.finger.kinematics.FingerForwardKinematics;

public class FingerController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private FingerForwardKinematics forwardKinematics;
   private final String prefix = "ForwardKinematics";

   private final YoDouble proximalAngleInput;
   private final YoDouble intermediateAngleInput;
   private final YoDouble xPositionOutput;
   private final YoDouble yPositionOutput;
   private final YoDouble distalAngleOutput;

   private final FingerRobot robot;

   private final SerialCommunicationInterface serial;
   String comPort = "COM7";
   int baud = 115200;
   JointSpaceSensorData sensorData;

   boolean useSerial = false;

   public FingerController(FingerRobot robot, FingerParameters parameters)
   {
      this.robot = robot;

      forwardKinematics = new FingerForwardKinematics(parameters);
      proximalAngleInput = new YoDouble(prefix + "ProximalAngleInput", registry);
      intermediateAngleInput = new YoDouble(prefix + "IntermediateAngleInput", registry);
      xPositionOutput = new YoDouble(prefix + "XPositionOutput", registry);
      yPositionOutput = new YoDouble(prefix + "YPositionOutput", registry);
      distalAngleOutput = new YoDouble(prefix + "DistalAngleOutput", registry);

      robot.setDynamic(false);

      sensorData = new JointSpaceSensorData();
      serial = new SerialCommunicationInterface(comPort, baud);
   }

   public void setUseSerial(boolean useSerial)
   {
      this.useSerial = useSerial;
   }

   @Override
   public void doControl()
   {
      sensorData.set(serial.getJointSensorData());
      intermediateAngleInput.set(sensorData.getProximalAngle() * Math.PI / 180.0);
      proximalAngleInput.set(sensorData.getIntermediateAngle() * Math.PI / 180.0);

      forwardKinematics.setInputAngles(proximalAngleInput.getDoubleValue(), intermediateAngleInput.getDoubleValue());
      forwardKinematics.computeIfNeeded();
      xPositionOutput.set(forwardKinematics.getOutputX());
      yPositionOutput.set(forwardKinematics.getOutputY());
      distalAngleOutput.set(forwardKinematics.getDistalAngle());

      robot.setProximalAngle(proximalAngleInput.getDoubleValue());
      robot.setIntermediateAngle(intermediateAngleInput.getDoubleValue());
      robot.setDistalAngle(distalAngleOutput.getDoubleValue());
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
