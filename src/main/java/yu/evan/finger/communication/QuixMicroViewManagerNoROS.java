//package yu.evan.finger.communication;
//
//import com.fazecast.jSerialComm.SerialPort;
//import com.fazecast.jSerialComm.SerialPortDataListener;
//import com.fazecast.jSerialComm.SerialPortEvent;
//import controller_msgs.msg.dds.*;
//import us.ihmc.commons.Conversions;
//import us.ihmc.communication.IHMCRealtimeROS2Publisher;
//import us.ihmc.communication.ROS2Tools;
//import us.ihmc.communication.controllerAPI.CommandInputManager;
//import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
//import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
//import us.ihmc.log.LogTools;
//import us.ihmc.quix.controlModules.QuixMotions;
//import us.ihmc.quix.controlModules.stepGeneration.GeneratorType;
//import us.ihmc.realtime.MonotonicTime;
//import us.ihmc.realtime.PeriodicParameters;
//import us.ihmc.realtime.PriorityParameters;
//import us.ihmc.realtime.RealtimeThread;
//import us.ihmc.ros2.RealtimeROS2Node;
//import us.ihmc.simulationconstructionset.gui.YoSliderpanel;
//import us.ihmc.simulationconstructionset.util.RobotController;
//import us.ihmc.yoVariables.listener.YoVariableChangedListener;
//import us.ihmc.yoVariables.registry.YoRegistry;
//import us.ihmc.yoVariables.variable.YoBoolean;
//import us.ihmc.yoVariables.variable.YoInteger;
//import us.ihmc.yoVariables.variable.YoVariable;
//
//import java.io.IOException;
//import java.io.InputStream;
//import java.io.PrintWriter;
//import java.nio.charset.StandardCharsets;
//
//public class QuixMicroViewManagerNoROS extends RealtimeThread
//{
//   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
//
//   private static final long period = Conversions.millisecondsToNanoseconds(2);
//   private static final PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
//   private static final PeriodicParameters controllerPeriod = new PeriodicParameters(new MonotonicTime(0, period));
//
//   private SerialPort comPort;
//
//   private static final byte MSG_OUT_START_BYTE = 120;
//   private static final byte MSG_OUT_END_BYTE = 121;
//   private byte[] msgToCrutch = new byte[9]; // above plus below
//   private YoInteger yoUrgentInfo = new YoInteger("crutchThreadUrgentInfo", registry);
//   private YoInteger yoStartupState = new YoInteger("crutchThreadStartupState", registry);
//   private YoInteger yoPreviousMotionState = new YoInteger("crutchThreadPreviousMotionState", registry);
//   private YoInteger yoCurrentMotionState = new YoInteger("crutchThreadCurrentMotionState", registry);
//   private YoInteger yoSitDownState = new YoInteger("crutchThreadSitDownState", registry);
//   private YoInteger yoStandUpState = new YoInteger("crutchThreadStandUpState", registry);
//   private YoInteger yoSideStepSwingOutStatus = new YoInteger("crutchThreadSideStepSwingOutStatus", registry);
//
//   private byte[] msgFromCrutch = new byte[11];
//   private YoBoolean yoUserEnable = new YoBoolean("crutchThreadUserEnable", registry);
//   private YoBoolean yoReWiggle = new YoBoolean("crutchThreadReWiggle", registry);
//   private YoBoolean yoStartBehavior = new YoBoolean("crutchThreadStartBehavior", registry);
//   private YoInteger yoRequestedMotionState = new YoInteger("crutchThreadRequestedMotionState", registry);
//   private YoBoolean yoExecuteBehavior = new YoBoolean("crutchThreadExecuteBehavior", registry);
//   private YoBoolean yoContinuousWalking = new YoBoolean("crutchThreadContinuousWalking", registry);
//   private YoInteger yoFlatStepType = new YoInteger("crutchThreadFlatStepType", registry);
//   private YoInteger yoStairsStepType = new YoInteger("crutchThreadStairsStepType", registry);
//   private YoInteger yoSideStepDirection = new YoInteger("crutchThreadSideStepDirection", registry);
//   private YoInteger yoSlopeStepType = new YoInteger("crutchThreadSlopeStepType", registry);
//   private YoInteger yoForceSwingSide = new YoInteger("crutchThreadForceSwingSide", registry);
//
//   private final CommandInputManager commandInputManager;
//   private final StatusMessageOutputManager controllerStatusOutputManager;
//
//   private controller_msgs.msg.dds.QuixCrutchMessage crutchMsg = new QuixCrutchMessage();
//
//   public QuixMicroViewManagerNoROS(String comPortID,
//                                    CommandInputManager commandInputManager,
//                                    StatusMessageOutputManager controllerStatusOutputManager,
//                                    String robotName)
//   {
//      super(controllerPriority, controllerPeriod);
//
//      this.commandInputManager = commandInputManager;
//      this.controllerStatusOutputManager = controllerStatusOutputManager;
//
//      /*
//       * Setup serial comm port
//       */
//      this.comPort = SerialPort.getCommPort(comPortID);
//      //for the love of god speed it up
//      this.comPort.setComPortParameters(9600, 8, 1, SerialPort.NO_PARITY);
//      //Set this to non-blocking?
//      comPort.setComPortTimeouts(SerialPort.TIMEOUT_NONBLOCKING, 0, 0); // TODO
//      comPort.openPort(5000);
//
//      comPort.addDataListener(new SerialPortDataListener()
//      {
//         @Override
//         public int getListeningEvents()
//         {
//            return SerialPort.LISTENING_EVENT_DATA_AVAILABLE;
//         }
//
//         @Override
//         public void serialEvent(SerialPortEvent event)
//         {
//            if (event.getEventType() != SerialPort.LISTENING_EVENT_DATA_AVAILABLE)
//               return;
//            comPort.readBytes(msgFromCrutch, msgFromCrutch.length);
//            //System.out.println(msgFromCrutch);
//         }
//      });
//
//      /*
//       * Setup ROS comms
//       */
//      // setup publishers
//
//      // setup subscribers
//
//      StatusMessageListener<QuixUrgentUserInfoMessage> urgentUserInfoListener = (userInfo) ->
//      {
//         msgToCrutch[1] = userInfo.getUrgentUserInfoName();
//         yoUrgentInfo.set(msgToCrutch[1]);
//      };
//
//      StatusMessageListener<QuixStartupStateMessage> startupStateListener = (startupState) ->
//      {
//         msgToCrutch[2] = startupState.getStartupStateName();
//         yoStartupState.set(msgToCrutch[2]);
//      };
//
//      StatusMessageListener<QuixMotionStateMessage> motionStateListener = (motionState) ->
//      {
//         msgToCrutch[3] = msgToCrutch[4]; // set previous to current
//         msgToCrutch[4] = motionState.getMotionStateName(); // update current
//
//         yoPreviousMotionState.set(msgToCrutch[3]);
//         yoCurrentMotionState.set(msgToCrutch[4]);
//      };
//
//      StatusMessageListener<QuixSitDownStateMessage> sitDownStateListener = (sitDownState) ->
//      {
//         msgToCrutch[5] = sitDownState.getSitDownStateName();
//         yoSitDownState.set(msgToCrutch[5]);
//      };
//
//      StatusMessageListener<QuixStandUpStateMessage> standUpStateListener = (standUpState) ->
//      {
//         msgToCrutch[6] = standUpState.getStandUpStateName();
//         yoStandUpState.set(msgToCrutch[6]);
//      };
//
//      StatusMessageListener<QuixSideStepSwingOutMessage> sideStepSwingOutListener = (sideStepSwingOut) ->
//      {
//         msgToCrutch[7] = sideStepSwingOut.getSideStepInSwingOut() ? (byte) 1 : 0;
//         yoSideStepSwingOutStatus.set(msgToCrutch[7]);
//      };
//
//      this.controllerStatusOutputManager.attachStatusMessageListener(QuixUrgentUserInfoMessage.class, urgentUserInfoListener);
//      this.controllerStatusOutputManager.attachStatusMessageListener(QuixStartupStateMessage.class, startupStateListener);
//      this.controllerStatusOutputManager.attachStatusMessageListener(QuixMotionStateMessage.class, motionStateListener);
//      this.controllerStatusOutputManager.attachStatusMessageListener(QuixSitDownStateMessage.class, sitDownStateListener);
//      this.controllerStatusOutputManager.attachStatusMessageListener(QuixStandUpStateMessage.class, standUpStateListener);
//      this.controllerStatusOutputManager.attachStatusMessageListener(QuixSideStepSwingOutMessage.class, sideStepSwingOutListener);
//
//      createListeners();
//      System.out.println("finishedListeners");
//   }
//
//   private void createListeners()
//   {
//      yoUserEnable.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.setUserEnable(yoUserEnable.getBooleanValue());
//            commandInputManager.submitMessage(crutchMsg);
//            System.out.println("user Enabling");
//         }
//      });
//
//      yoReWiggle.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.setRewiggle(yoReWiggle.getBooleanValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      yoStartBehavior.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.setStartBehavior(yoStartBehavior.getBooleanValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      yoRequestedMotionState.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.getRequestedMotionState().setMotionStateName((byte) yoRequestedMotionState.getIntegerValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      yoExecuteBehavior.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.setExecuteBehavior(yoExecuteBehavior.getBooleanValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      yoContinuousWalking.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.setContinuousWalking(yoContinuousWalking.getBooleanValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      yoFlatStepType.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.getFlatStepType().setFlatStepTypeName((byte) yoFlatStepType.getIntegerValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      yoStairsStepType.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.getStairsStepType().setStairsStepTypeName((byte) yoStairsStepType.getIntegerValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      yoSideStepDirection.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.getSideStepDirection().setStepDirection((byte) yoSideStepDirection.getIntegerValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      yoSlopeStepType.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.getSlopeStepType().setSlopeStepTypeName((byte) yoSlopeStepType.getIntegerValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      yoForceSwingSide.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            crutchMsg.setForceSwingSide((byte) yoForceSwingSide.getIntegerValue());
//            commandInputManager.submitMessage(crutchMsg);
//         }
//      });
//
//      ////////////////////////////////////////////
//
//      yoUrgentInfo.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            sendMsgToCrutch();
//         }
//      });
//
//      yoStartupState.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            sendMsgToCrutch();
//         }
//      });
//
//      yoPreviousMotionState.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            sendMsgToCrutch();
//         }
//      });
//
//      yoCurrentMotionState.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            sendMsgToCrutch();
//         }
//      });
//
//      yoSitDownState.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            sendMsgToCrutch();
//         }
//      });
//
//      yoStandUpState.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            sendMsgToCrutch();
//         }
//      });
//
//      yoSideStepSwingOutStatus.addListener(new YoVariableChangedListener()
//      {
//         @Override
//         public void changed(YoVariable source)
//         {
//            sendMsgToCrutch();
//         }
//      });
//   }
//
//   public void initialize()
//   {
//      msgToCrutch[0] = MSG_OUT_START_BYTE;
//      msgToCrutch[msgToCrutch.length - 1] = MSG_OUT_END_BYTE;
//
//      super.start();
//   }
//
//   @Override
//   public void run()
//   {
//      try
//      {
//         while (true)
//         {
//            //            byte[] ascii_bytes = new byte[11];
//            //            for(int i = 0; i < 11; i++) {
//            //               ascii_bytes[i] = (byte) (msgFromCrutch[i] +  0x30);
//            //            }
//            //            System.out.println(new String(ascii_bytes, StandardCharsets.US_ASCII));
//            yoUserEnable.set(msgFromCrutch[0] == 1);
//            yoReWiggle.set(msgFromCrutch[1] == 1);
//            yoStartBehavior.set(msgFromCrutch[2] == 1);
//            yoRequestedMotionState.set(msgFromCrutch[3]);
//            yoExecuteBehavior.set(msgFromCrutch[4] == 1);
//            yoContinuousWalking.set(msgFromCrutch[5] == 1);
//            yoFlatStepType.set(msgFromCrutch[6]);
//            yoStairsStepType.set(msgFromCrutch[7]);
//            yoSideStepDirection.set(msgFromCrutch[8]);
//            yoSlopeStepType.set(msgFromCrutch[9]);
//            yoForceSwingSide.set(msgFromCrutch[10]);
//            //System.out.println("still running");
//            waitForNextPeriod();
//         }
//      }
//      catch (Throwable e)
//      {
//         e.printStackTrace();
//      }
//   }
//
//   public void sendMsgToCrutch()
//   {
//      comPort.writeBytes(msgToCrutch, msgToCrutch.length);
//   }
//
//   public YoRegistry getRegistry()
//   {
//      return registry;
//   }
//}