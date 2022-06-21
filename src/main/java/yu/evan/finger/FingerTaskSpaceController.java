package yu.evan.finger;

public class FingerTaskSpaceController
{
   private TaskSpaceCommand taskSpaceCommandProvided;
   private JointSpaceCommand jointSpaceCommandComputed;

   public void getSensorData()
   {

   }

   public void submitDesiredTaskSpaceCommand(TaskSpaceCommand taskSpaceCommand)
   {

   }

   public void getJointSpaceCommand()
   {

   }

   public void submitSensorData(SensorData sensorData)
   {
      JointState measuredJointState = sensorData.getJointState();
      measuredJointState.torques.add(feedforwardTorque(measuredJointState.positions))
   }

   public void read()
   {
      measuredTaskPosition = forwardKinematics(measuredJointState.positions);
      measuredTaskVelocities = jacobian(measuredJointState.positions) * (measuredJointState.velocities);
      measuredTaskForces = invJacobian(measuredJointState.positions) * measuredJointState.torques;

      measuredTaskState.set(measuredTaskPosition, measuredTaskVelocities, measuredTaskForces);
   }

   public void computeControl(double dt)
   {
      // do poisition, torque, or impedance control
      // if position or torque control, transparent
      // if impedance control, use error in position to decide force

      desiredJointPositions = inverseKinematics(desiredTaskPositions);
      desiredJointTorques = jacobian(desiredJointPositions) * desiredTaskForces;
      desiredJointTorques.add(feedforwardTorque(desiredJointPositions))

      jointSpaceCommandComputed.set(desiredJointPositions, desiredJointTorques);
   }

}
