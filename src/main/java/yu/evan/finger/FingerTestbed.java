package yu.evan.finger;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * In this example simulation, we will create a simple 7-DoF robot arm {@code RobotArmOne} which is
 * controlled with simple PD controllers on each joint to track sine-wave trajectories, see
 * {@code RobotArmOneController}.
 * <p>
 * This is the main class that sets up and starts the simulation environment.
 * </p>
 */
public class FingerTestbed
{
//   private ArduinoCommunication arduinoCommunication;
//   public FingerTestbed()
//   {
//      arduinoCommunication = new ArduinoCommunication();
//      FingerRobot fingerRobot = new FingerRobot();
//      FingerController fingerController = new FingerController();
//      fingerController.initialize();
//      fingerRobot.setController(fingerController);
//
//
//      // Creating the simulation.
//      SimulationConstructionSet scs = new SimulationConstructionSet(fingerRobot);
//      // As this example simulation is rather simple, let's prevent SCS from
//      // simulating faster than real-time.
//      scs.setSimulateNoFasterThanRealTime(true);
//      // Defining the simulation DT and the frequency at which data is logged.
//      scs.setDT(1.0e-4, 10);
//      // Defining the buffer size to ensure a minimum simulation duration before
//      // filling the graphs in the simulator.
//      scs.changeBufferSize(65536);
//      // Launch the simulator.
//      scs.startOnAThread();
//   }
//
//   public static void main(String[] args)
//   {
//      new FingerTestbed();
//   }
}
