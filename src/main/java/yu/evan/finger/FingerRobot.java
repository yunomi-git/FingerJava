package yu.evan.finger;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.RigidJoint;
import us.ihmc.simulationconstructionset.Robot;

public class FingerRobot extends Robot
{
   private final double distalLength = 0.022; //mm
   private final double intermediateLength = 0.025; //mm
   private final double proximalLength = 0.043; //mm
   private final double DEFAULT_MASS = 1.0;

   private FingerForwardKinematics forwardKinematics;

   public FingerRobot()
   {
      super("Finger");

      RigidJoint rootJoint = new RigidJoint("CeilingJoint", new Vector3D(), this);

      Link ceiling = new Link("link1");
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(0, 0, 0);

      ceiling.setLinkGraphics(linkGraphics);
      rootJoint.setLink(ceiling);

      PinJoint proximalJoint = new PinJoint("jointProximal", new Vector3D(0.0, 0.0, 0.0), this, Axis3D.Y);
      PinJoint intermediateJoint = new PinJoint("jointIntermediate", new Vector3D(proximalLength, 0.0, 0.0), this, Axis3D.Y);
      PinJoint distalJoint = new PinJoint("jointDistal", new Vector3D(intermediateLength, 0.0, 0.0), this, Axis3D.Y);

      proximalJoint.setLink(createLink("proximalLink", proximalLength));
      rootJoint.addJoint(proximalJoint);
      intermediateJoint.setLink(createLink("intermediateLink", intermediateLength));
      proximalJoint.addJoint(intermediateJoint);
      distalJoint.setLink(createLink("distalLink", distalLength));
      intermediateJoint.addJoint(distalJoint);

      proximalJoint.setInitialState(0.0, 0.0);
      intermediateJoint.setInitialState(0, 0.0);
      distalJoint.setInitialState(0, 0.0);

      this.addRootJoint(rootJoint);


   }

   private Link createLink(String name, double length)
   {
      Link ret = new Link(name);
      ret.setMass(DEFAULT_MASS);
      ret.setComOffset(0.0, 0.0, length);
      double ixx = DEFAULT_MASS/3.0 * (length*length);
      double iyy = ixx;
      double izz = 0.0;
      ret.setMomentOfInertia(ixx, iyy, izz);
      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(new YawPitchRoll(0, Math.PI / 2, 0));
      linkGraphics.addCylinder(length, 0.005, YoAppearance.BlueViolet());
      linkGraphics.addSphere(0.006, YoAppearance.BlueViolet());
      linkGraphics.translate(0.0, 0.0, length);
      linkGraphics.addSphere(0.006, YoAppearance.BlueViolet());


      // associate the linkGraphics object with the link object
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }


}
