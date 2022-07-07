package yu.evan.finger;

public class ArithmeticVector2D implements Arithmetic<ArithmeticVector2D>
{
   private double x1;
   private double x2;

   public ArithmeticVector2D(double x1, double x2)
   {
      this.x1 = x1;
      this.x2 = x2;
   }

   @Override
   public void setAddition(ArithmeticVector2D add1, ArithmeticVector2D add2)
   {
      this.set1(add1.get1() + add2.get1());
      this.set2(add1.get2() + add2.get2());
   }

   @Override
   public void setScale(double val, ArithmeticVector2D mult)
   {
      this.set1(mult.get1() * val);
      this.set2(mult.get2() * val);
   }

   @Override
   public void copy(ArithmeticVector2D other)
   {
      this.set1(other.get1());
      this.set2(other.get2());
   }

   public void set1(double x1)
   {
      this.x1 = x1;
   }

   public void set2(double x2)
   {
      this.x2 = x2;
   }

   public double get1()
   {
      return x1;
   }

   public double get2()
   {
      return x2;
   }
}
