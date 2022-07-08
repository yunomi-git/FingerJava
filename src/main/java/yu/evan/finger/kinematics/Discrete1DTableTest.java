package yu.evan.finger.kinematics;

public class Discrete1DTableTest implements DiscreteLookupTable2D<Arithmetic1D>
{

   @Override
   public Arithmetic1D createDefaultValue()
   {
      return new Arithmetic1D(0.0);
   }

   @Override
   public int getNumDivisions()
   {
      return 1;
   }

   @Override
   public Arithmetic1D getValueAtIndex(int i0, int i1)
   {
      return dataTable[i0][i1];
   }

   private Arithmetic1D[][] dataTable = {{new Arithmetic1D(0.0), new Arithmetic1D(1.0)}, {new Arithmetic1D(2.0), new Arithmetic1D(3.0)}};
}
