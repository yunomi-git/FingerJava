package yu.evan.finger;

public interface Arithmetic<T>
{
   public void setAddition(T add1, T add2);
   public void setScale(double val, T mult);
   public void copy(T other);
}
