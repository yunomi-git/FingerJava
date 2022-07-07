package yu.evan.finger;

public class MathUtil
{
   public static double lawOfCosine(double opposingLength, double adjacentLength1, double adjacentLength2)
   {
      double cosine = (opposingLength*opposingLength - adjacentLength1*adjacentLength1 - adjacentLength2*adjacentLength2)/(-2*adjacentLength1*adjacentLength2);
      return java.lang.Math.acos(cosine);
   }
}
