package yu.evan.finger;

public class FingerParameters // eventually need to redo this
{
   private final double distalLength = 0.022; //mm
   private final double intermediateLength = 0.025; //mm
   private final double proximalLength = 0.043; //mm

   public double getDistalLength()
   {
      return distalLength;
   }

   public double getIntermediateLength()
   {
      return intermediateLength;
   }

   public double getProximalLength()
   {
      return proximalLength;
   }
}
