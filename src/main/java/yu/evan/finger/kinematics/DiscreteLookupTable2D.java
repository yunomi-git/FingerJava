package yu.evan.finger.kinematics;

public interface DiscreteLookupTable2D<T extends Arithmetic>
{
   public T createDefaultValue();
   public int getNumDivisions();
   public T getValueAtIndex(int i0, int i1);
}
