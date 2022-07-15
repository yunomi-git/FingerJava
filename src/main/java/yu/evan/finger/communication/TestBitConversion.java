package yu.evan.finger.communication;

public class TestBitConversion
{
    public TestBitConversion()
    {
    //  byte[] readBuffer = {(byte) 0b10000000, (byte) 0b00000000};
    //  byte[] readBuffer = {(byte) 0b00001011, (byte) 0b00010111}; // 3.9
        byte[] readBuffer = {(byte) 0b00001010, (byte) 0b11111111}; // 3.8
        int intBits = convertReadBufferToIntFromIndex(readBuffer, 0);
        System.out.println(intBits);
        System.out.println(Integer.toString(intBits, 2));
    }

    private int convertReadBufferToIntFromIndex(byte[] readBuffer, int i)
    {
        int intBits = (((readBuffer[i]& 0xff) << 8 ) | ((readBuffer[i+1])& 0xff));
//        int intBits =  Byte.toUnsignedInt((readBuffer[i+1]));
//        int intBits =  ((byte) readBuffer[i+1]) & 0xff;
        return intBits;
    }

    public static void main(String[] args)
    {
        TestBitConversion test = new TestBitConversion();
    }
}
