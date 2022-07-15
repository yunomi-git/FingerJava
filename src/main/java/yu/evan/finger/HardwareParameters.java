package yu.evan.finger;

public class HardwareParameters
{
    public double MAX_READ_TORQUE = 0.4; // Nm
    public double MIN_READ_TORQUE = -MAX_READ_TORQUE;

    public double MAX_READ_VELOCITY = 0.5; // deg/sec
    public double MIN_READ_VELOCITY = -MAX_READ_VELOCITY;

    public double MIN_READ_ANGLE = 0.0;
    public double MAX_READ_ANGLE = 90.0; // deg

    public double SERIAL_WRITE_SCALE = 65536.0; // 16 bits...just fill out 2 bytes

}
