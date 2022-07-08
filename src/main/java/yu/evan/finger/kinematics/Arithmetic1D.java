package yu.evan.finger.kinematics;

public class Arithmetic1D implements Arithmetic<Arithmetic1D>
{
   private double x;

   public Arithmetic1D(double x)
   {
      this.x = x;
   }

   @Override
   public void setAddition(Arithmetic1D add1, Arithmetic1D add2)
   {
      this.set(add1.get() + add2.get());
   }

   @Override
   public void setScale(double val, Arithmetic1D mult)
   {
      this.set(mult.get() * val);
   }

   @Override
   public void copy(Arithmetic1D other)
   {
      this.set(other.get());
   }

   public void set(double x)
   {
      this.x = x;
   }

   public double get()
   {
      return x;
   }

}
