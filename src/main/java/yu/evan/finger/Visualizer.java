package yu.evan.finger;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class Visualizer
{
   private SimulationConstructionSet sim;
   public Visualizer()
   {
      FingerRobot fingerRobot = new FingerRobot();

      sim = new SimulationConstructionSet(fingerRobot);
      sim.setGroundVisible(false);
      sim.setDT(0.001, 400);
      sim.setCameraFix(0.0, 0.0, 0.0);
      sim.setCameraPosition(0.0, 2.0, 0.0);

//      sim.setCameraTracking(true, true, true, false);
//      sim.setCameraDolly(false, true, true, false);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new Visualizer();
   }
}
