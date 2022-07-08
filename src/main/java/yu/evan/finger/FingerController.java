package yu.evan.finger;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
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
   }

   @Override
   public void doControl()
   {
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
