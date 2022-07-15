package yu.evan.finger.communication;

import com.fazecast.jSerialComm.SerialPort;
import com.fazecast.jSerialComm.SerialPortDataListener;
import com.fazecast.jSerialComm.SerialPortEvent;
import yu.evan.finger.HardwareParameters;
import yu.evan.finger.JointSpaceCommand;
import yu.evan.finger.JointSpaceSensorData;

public class SerialCommunicationInterface
{
   SerialPort comPort;
   private final int readBytes = 12;
   private final int writeBytes = 9;
   private byte[] readBuffer = new byte[readBytes];
   private byte[] writeBuffer = new byte[writeBytes];

   JointSpaceCommand command;
   JointSpaceSensorData sensorData;
   HardwareParameters hp = new HardwareParameters();

   public SerialCommunicationInterface(String comPortID, int baudRate)
   {
      this.comPort = SerialPort.getCommPort(comPortID);
      this.comPort.setComPortParameters(baudRate, 8, 1, SerialPort.NO_PARITY);

      comPort.setComPortTimeouts(SerialPort.TIMEOUT_NONBLOCKING, 0, 0); // TODO
      comPort.openPort(5000);

      // Read event
      comPort.addDataListener(new SerialPortDataListener()
      {
         @Override
         public int getListeningEvents()
         {
            return SerialPort.LISTENING_EVENT_DATA_AVAILABLE;
         }

         @Override
         public void serialEvent(SerialPortEvent event)
         {
            if (event.getEventType() != SerialPort.LISTENING_EVENT_DATA_AVAILABLE)
               return;
            comPort.readBytes(readBuffer, readBytes);
         }
      });

      sensorData = new JointSpaceSensorData();

   }

   private void convertReadBufferToSensorData()
   {
      double proximalPosition = convertReadBufferToFloatFromIndex(readBuffer,0, hp.MIN_READ_ANGLE, hp.MAX_READ_ANGLE);
      double proximalVelocity = convertReadBufferToFloatFromIndex(readBuffer, 2, hp.MIN_READ_VELOCITY, hp.MAX_READ_VELOCITY);
      double proximalTorque = convertReadBufferToFloatFromIndex(readBuffer, 4, hp.MIN_READ_TORQUE, hp.MAX_READ_TORQUE);
      double intermediatePosition = convertReadBufferToFloatFromIndex(readBuffer,6, hp.MIN_READ_ANGLE, hp.MAX_READ_ANGLE);
      double intermediateVelocity = convertReadBufferToFloatFromIndex(readBuffer,8, hp.MIN_READ_VELOCITY, hp.MAX_READ_VELOCITY);
      double intermediateTorque = convertReadBufferToFloatFromIndex(readBuffer,10, hp.MIN_READ_TORQUE, hp.MAX_READ_TORQUE);
      sensorData.set(proximalPosition, proximalVelocity, proximalTorque,
              intermediatePosition, intermediateVelocity, intermediateTorque);
   }

   private double convertReadBufferToFloatFromIndex(byte[] readBuffer, int i, double min, double max)
   {
      int intBits = (((readBuffer[i]& 0xff) << 8) | ((readBuffer[i+1])& 0xff));
      double scale = intBits / hp.SERIAL_WRITE_SCALE;
      return min + scale * (max - min);
//      return (double) intBits;
   }

   private void writeFloatToWriteBufferFromIndex(float val, byte[] writeBuffer, int i)
   {
      int intBits = Float.floatToIntBits(val);
      writeBuffer[i] = (byte) (intBits >> 24);
      writeBuffer[i+1] = (byte) (intBits >> 16);
      writeBuffer[i+2] = (byte) (intBits >> 8);
      writeBuffer[i+3] = (byte) (intBits);
   }

   public JointSpaceSensorData getJointSensorData()
   {
      convertReadBufferToSensorData();
      return sensorData;
   }

   public void setCommand(JointSpaceCommand command)
   {
      this.command.set(command);
   }

   public void write()
   {
      int mode = command.getControlMode().ordinal();
      float proximalInput = (float) command.getProximalInput();
      float intermediateInput = (float) command.getIntermediateInput();

      writeBuffer[0] = (byte) mode;
      writeFloatToWriteBufferFromIndex(proximalInput, writeBuffer, 1);
      writeFloatToWriteBufferFromIndex(intermediateInput, writeBuffer, 5);
      comPort.writeBytes(writeBuffer, writeBytes);
   }
}
