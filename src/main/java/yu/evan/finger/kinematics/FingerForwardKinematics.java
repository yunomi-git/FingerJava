package yu.evan.finger.kinematics;

import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import yu.evan.finger.FingerParameters;
import yu.evan.finger.MathUtil;

//        int[][] values = {{1,2},{3,4}};
//        System.out.println(values[0][1]); -> 2
public class FingerForwardKinematics
{

//   private final YoDouble proximalAngleInput;
//   private final YoDouble intermediateAngleInput;
//   private final YoDouble xPositionOutput;
//   private final YoDouble yPositionOutput;
//   private final YoDouble distalAngleOutput;

   private double proximalAngle;
   private double intermediateAngle;
   private ArithmeticVector2D arithmeticOutput;
   private double outputX;
   private double outputY;
   private double distalAngle;
   private boolean outputIsCurrent = false;
   private LookupTable2D<ArithmeticVector2D> lookupTable2D;

   // For garbage free calculations
   private Vector2D outputPosition;
   private Vector2D outputXYromIntermediate;
   private Vector2D proximalPosition;
   private Vector2D intermediatePosition;

   private double proximalLength;
   private double intermediateLength;
   private double distalLength;

   private final Vector2D proximalDefaultPosition;
   private final Vector2D intermediateDefaultPosition;

   public FingerForwardKinematics(FingerParameters parameters)
   {
      distalLength = parameters.getDistalLength();
      intermediateLength = parameters.getIntermediateLength();
      proximalLength = parameters.getProximalLength();

      proximalDefaultPosition = new Vector2D(proximalLength, 0);
      intermediateDefaultPosition = new Vector2D(intermediateLength, 0);

      lookupTable2D = new LookupTable2D<>(new DiscreteLookupTable2DKinematicsV1(), 0, Math.PI/2, 0, Math.PI/2);

      outputPosition = new Vector2D();
      outputXYromIntermediate = new Vector2D();
      proximalPosition = new Vector2D();
      intermediatePosition = new Vector2D();
   }


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
      RotationMatrixTools.applyYawRotation(proximalAngle, proximalDefaultPosition, proximalPosition);
      RotationMatrixTools.applyYawRotation(intermediateAngle + proximalAngle, intermediateDefaultPosition, intermediatePosition);
      intermediatePosition.add(proximalPosition);

      outputPosition.setX(outputX); // outputPosition is only used here for gf operation
      outputPosition.setY(outputY);

      outputXYromIntermediate.set(outputPosition);
      outputXYromIntermediate.sub(intermediatePosition);

      boolean xIsPos = outputXYromIntermediate.getX() > 0;
      boolean yIsPos = outputXYromIntermediate.getY() > 0;

      double altAngle = MathUtil.lawOfCosine(Math.abs(outputXYromIntermediate.getY()), Math.abs(outputXYromIntermediate.getX()), distalLength);
      double distalAngle;

      if (xIsPos && yIsPos)
      {
         distalAngle = altAngle - proximalAngle - intermediateAngle;
      }
      else if (!xIsPos && yIsPos)
      {
         distalAngle = Math.PI - altAngle - proximalAngle - intermediateAngle;
      }
      else if (!xIsPos && !yIsPos)
      {
         distalAngle = Math.PI + altAngle - proximalAngle - intermediateAngle;
      }
      else
      {
         distalAngle = altAngle - proximalAngle - intermediateAngle;
      }

      return distalAngle;
   }



}
