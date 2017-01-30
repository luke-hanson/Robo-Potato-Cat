package us.ihmc.exampleSimulations.simplePendulum;

import us.ihmc.simulationconstructionset.Robot;

import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;

import us.ihmc.robotics.Axis;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import javax.vecmath.Vector3d;


public class SimplePendulum extends Robot // SimplePendulumRobot inherits some properties and methods from Robot class
{
   /*
     Pendulum constants and initial values
      - Lengths are in meters (m)
      - Masses are in kilograms (kg)
   */

    public static final double FULCRUM_RADIUS = 0.02;

    public static final double ROD_LENGTH = 1.0;
    public static final double ROD_RADIUS = 0.01;
    public static final double ROD_MASS = 0.01;

    public static final double BALL_RADIUS = 0.05;
    public static final double BALL_MASS = 1.0;

    // I = mrˆ2 pendulum's resistance to changes to its rotation in kg.mˆ2
    public static final double FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y = ROD_LENGTH * ROD_LENGTH * BALL_MASS;

   /*
      Initial state of the pendulum
   */

    private double fulcrumInitialPositionDegrees = 90.0;
    private double fulcrumInitialPositionRadians = fulcrumInitialPositionDegrees * Math.PI / 180.0;
    private double fulcrumInitialVelocity = 0.0;

    /*
        Constructor creates an instance of the class SimplePendulumRobot
    */
    public SimplePendulumRobot()
    {
        // a. Call parent class "Robot" constructor. The string "SimplePendulum" will be the name of the robot.
        super("SimplePendulum");

        // b. Create a link
        Link pendulumLink = new Link("PendulumLink");
        pendulumLink.setMass(BALL_MASS);
        pendulumLink.setComOffset(0.0, 0.0, -ROD_LENGTH);
        pendulumLink.setMomentOfInertia(0.0, FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y, 0.0);

        // c. Create a pin joint attached to the link
        PinJoint fulcrumPinJoint = new PinJoint("FulcrumPinJoint", new Vector3d(0.0, 0.0, 1.5), this, Axis.Y);
        fulcrumPinJoint.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);
        fulcrumPinJoint.setDamping(0.3);
        fulcrumPinJoint.setLink(pendulumLink);

        // d. Add fulcrumPinJoint as the root joint of the robot
        this.addRootJoint(fulcrumPinJoint);

        // e. Add 3D objects
        Graphics3DObject pendulumGraphics = new Graphics3DObject();
        pendulumGraphics.addSphere(FULCRUM_RADIUS, YoAppearance.BlueViolet());
        pendulumGraphics.translate(0.0, 0.0, -ROD_LENGTH);
        pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, YoAppearance.Black());
        pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
        pendulumLink.setLinkGraphics(pendulumGraphics);
    }
}