package yu.evan.finger.kinematics;

import yu.evan.finger.Visualizer;

public class LookupTableTest
{
   public LookupTableTest()
   {
      Discrete1DTableTest discreteTable = new Discrete1DTableTest();
      LookupTable2D<Arithmetic1D> lookupTable = new LookupTable2D<>(discreteTable, 0.0, 1.0, 0.0, 1.0);
      lookupTable.setInput(0,0);
      lookupTable.compute();
      Arithmetic1D output = lookupTable.getOutput();
      System.out.println(output.get());
   }

   public static void main(String[] args)
   {
      new LookupTableTest();
   }
}
