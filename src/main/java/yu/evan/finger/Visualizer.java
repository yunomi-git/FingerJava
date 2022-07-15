package yu.evan.finger;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import yu.evan.finger.kinematics.FingerForwardKinematics;

public class Visualizer
{
   private SimulationConstructionSet sim;
   public Visualizer()
   {
      FingerParameters parameters = new FingerParameters();
//      FingerForwardKinematics forwardKinematics = new FingerForwardKinematics(parameters);
//      forwardKinematics.setInputAngles(0.0349066, 0);
//      forwardKinematics.computeIfNeeded();
//      System.out.println(forwardKinematics.getOutputX());
//      System.out.println(forwardKinematics.getOutputY());
//      System.out.println(forwardKinematics.getDistalAngle());

      FingerRobot fingerRobot = new FingerRobot(parameters);
      FingerController controller = new FingerController(fingerRobot, parameters);
      controller.setUseSerial(true);
      fingerRobot.setController(controller);

      sim = new SimulationConstructionSet(fingerRobot);
      sim.setGroundVisible(false);
      sim.setDT(0.001, 400);
      sim.setCameraFix(0.0, 0.0, 0.0);
      sim.setCameraPosition(0.0, 2.0, 0.0);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new Visualizer();
   }
}
