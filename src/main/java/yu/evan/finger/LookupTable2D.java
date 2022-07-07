package yu.evan.finger;

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

   private T[][] outputTable;
   private double input1;
   private double input2;

   private T output00;
   private T output01;
   private T output10;
   private T output11;

   private T output0;
   private T output1;

   private T output;

   public void setInput(double input1, double input2)
   {
      this.input1 = input1;
      this.input2 = input2;
   }

   // 00 -0---- 10
   //  ...|......
   //  ...|......
   //  ...x......
   //  ...|......
   // 01 -1-----11
   public void compute()
   {
      int index0 = 0;
      double scale0 = 1.0;
      int index1 = 0;
      double scale1 = 1.0;

      output00.copy(outputTable[index0][index1]);
      output01.copy(outputTable[index0][index1 + 1]);
      output10.copy(outputTable[index0 + 1][index1]);
      output11.copy(outputTable[index0 + 1][index1 + 1]);

      output0.setAddition(output00, output10);
      output0.setScale(scale0, output0);
      output1.setAddition(output01, output11);
      output1.setScale(scale0, output1);

      output.setAddition(output0, output1);
      output.setScale(scale1, output);
   }

   public T getOutput()
   {
      return output;
   }
}
