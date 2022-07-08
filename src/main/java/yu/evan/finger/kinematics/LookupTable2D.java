package yu.evan.finger.kinematics;

/*
 * Table is 2d -> 1d
 * Input are 2 doubles
 * Output is an arithmetic data type. Output is interpolated along discrete input values
 */
public class LookupTable2D<T extends Arithmetic>
{
   // assuming values
   //    - increasing
   //    - evenlySpaced

   private final DiscreteLookupTable2D discreteTable;
   private final double min0;
   private final double min1;
   private final double max0;
   private final double max1;
   private final double stepSize0;
   private final double stepSize1;

   private double input0;
   private double input1;

   private T output00;
   private T output01;
   private T output10;
   private T output11;

   private T output0;
   private T output1;

   private T output;

   public LookupTable2D(DiscreteLookupTable2D discreteTable, double min0, double max0, double min1, double max1)
   {
      this.discreteTable = discreteTable;
      this.min0 = min0;
      this.max0 = max0;
      this.min1 = min1;
      this.max1 = max1;
      int numDivisions = discreteTable.getNumDivisions();
      this.stepSize0 = (max0 - min0) / numDivisions;
      this.stepSize1 = (max1 - min1) / numDivisions;

      output00 = (T) discreteTable.createDefaultValue();
      output01 = (T) discreteTable.createDefaultValue();
      output10 = (T) discreteTable.createDefaultValue();
      output11 = (T) discreteTable.createDefaultValue();
      output0 = (T) discreteTable.createDefaultValue();
      output1 = (T) discreteTable.createDefaultValue();
      output = (T) discreteTable.createDefaultValue();
   }

   public void setInput(double input1, double input2)
   {
      this.input0 = input1;
      this.input1 = input2;
   }

   // 00 -0---- 01
   //  ...|......
   //  ...|......
   //  ...x......
   //  ...|......
   // 10 -1-----11
   public void compute()
   {
      // get the appropriate index. scale is position in-between indices
      int index0 = (int) ((this.input0 - this.min0)/this.stepSize0);
      double scale0 = (this.input0 - index0 * this.stepSize0) / this.stepSize0;
      int index1 = (int) ((this.input1 - this.min1)/this.stepSize1);
      double scale1 = (this.input1 - index1 * this.stepSize1) / this.stepSize1;

      output00.copy(discreteTable.getValueAtIndex(index0, index1));
      output01.copy(discreteTable.getValueAtIndex(index0, index1 + 1));
      output10.copy(discreteTable.getValueAtIndex(index0 + 1, index1));
      output11.copy(discreteTable.getValueAtIndex(index0 + 1, index1 + 1));

      output00.setScale(1.0 - scale1, output00);
      output01.setScale(scale1, output01);
      output0.setAddition(output00, output01);
      output10.setScale(1.0 - scale1, output10);
      output11.setScale(scale1, output11);
      output1.setAddition(output10, output11);

      output0.setScale(1.0 - scale0, output0);
      output1.setScale(scale0, output1);
      output.setAddition(output0, output1);
   }

   public T getOutput()
   {
      return output;
   }
}
