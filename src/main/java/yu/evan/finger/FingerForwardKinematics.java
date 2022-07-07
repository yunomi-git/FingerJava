package yu.evan.finger;

import us.ihmc.euclid.tuple2D.Vector2D;

public class FingerForwardKinematics
{
   private double proximalAngle;
   private double intermediateAngle;
   private ArithmeticVector2D arithmeticOutput;
   private double outputX;
   private double outputY;
   private double distalAngle;
   private boolean outputIsCurrent = false;
   LookupTable2D<ArithmeticVector2D> lookupTable2D;

   // For garbage free calculations
   private Vector2D outputPosition;
   private Vector2D outputXYromIntermediate;
   private Vector2D proximalPosition;
   private Vector2D intermediatePosition;
   private double proximalLength;
   private double intermediateLength;
   private double distalLength;

   private final Vector2D proximalDefaultPosition = new Vector2D(proximalLength, 0);
   private final Vector2D intermediateDefaultPosition = new Vector2D(intermediateLength, 0);


   public void setInputAngles(double proximalAngle, double intermediateAngle)
   {
      this.proximalAngle = proximalAngle;
      this.intermediateAngle = intermediateAngle;
      outputIsCurrent = false;
   }

   public void computeIfNeeded()
   {
      if (!outputIsCurrent)
      {
         lookupTable2D.setInput(proximalAngle, intermediateAngle);
         lookupTable2D.compute();
         arithmeticOutput = lookupTable2D.getOutput();

         outputX = arithmeticOutput.get1();
         outputY = arithmeticOutput.get2();
         distalAngle = computeDistalAngle(proximalAngle, intermediateAngle, outputX, outputY);

         outputIsCurrent = true;
      }
   }

   public double getOutputX()
   {
      computeIfNeeded();
      return outputX;
   }

   public double getOutputY()
   {
      computeIfNeeded();
      return outputY;
   }

   public double getDistalAngle()
   {
      computeIfNeeded();
      return distalAngle;
   }

   // Given proximal angle, intermediate angle, and output position,
   // calculate the distal angle
   public double computeDistalAngle(double proximalAngle, double intermediateAngle, double outputX, double outputY)
   {
      proximalPosition = rotaionMatrix(proximalAngle) * proximalDefaultPosition;
      intermediatePosition = rotationMatrix(intermediateAngle) * intermediateDefaultPosition + proximalPosition;
      outputPosition.setX(outputX);
      outputPosition.setY(outputY);
      outputXYromIntermediate = outputPosition - intermediatePosition;

      distalAngle = MathUtil.lawOfCosine(outputXYromIntermediate.getY(), outputXYromIntermediate.getX(), distalLength);

      return distalAngle;
   }



}
