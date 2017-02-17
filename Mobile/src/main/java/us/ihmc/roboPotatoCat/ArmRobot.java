package us.ihmc.roboPotatoCat;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

import javax.vecmath.Vector3d;

/**
 *
 * In this tutorial, lengths are expressed in meters (m), masses in kilograms (kg)
 *
 */
public class ArmRobot extends Robot
{
    /*
       Define the parameters of the robot
    */
    public static final double ROD_LENGTH = 1.0;
    public static final double ROD_RADIUS = 0.01;
    public static final double ROD_MASS = 0.00;

    public static final double FULCRUM_RADIUS = 0.02;

    public static final double BALL_RADIUS = 0.05;
    public static final double BALL_MASS = 1.0;

    public static final double FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y =
            ROD_LENGTH * ROD_LENGTH * BALL_MASS; // I = mrˆ2 pendulum's resistance to changes to its rotation in  kg.mˆ2

   /*
      Initial state of the pendulum
   */

    private double fulcrumInitialPositionDegrees = 90.0;
    private double fulcrumInitialPositionRadians = fulcrumInitialPositionDegrees * Math.PI / 180.0;
    private double fulcrumInitialVelocity = 0.0;

    /* Some joint state variables */
    private DoubleYoVariable tau_fulcrum, q_fulcrum, qd_fulcrum; // Respectively Torque, Position, Velocity
    private DoubleYoVariable tau_Second, q_Second, qd_Second; // Respectively Torque, Position, Velocity

    /*
       Define its constructor
     */
    public ArmRobot()
    {
        //` a. Call parent class "Robot" constructor. The string "SimplePendulum" will be the name of the robot.
        super("pendulum");

        // b. Add a joint to the robot
        PinJoint fulcrumPinJoint = new PinJoint("FulcrumPin", new Vector3d(0.0, 0.0, 1.0), this, Axis.Y);
        PinJoint secondPinJoint = new PinJoint("SecondPin", new Vector3d(0.0, 0.0, -ROD_LENGTH), this, Axis.Y);

        fulcrumPinJoint.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);
        Link secondLink = pendulumLink();
        fulcrumPinJoint.setLink(secondLink);// pendulumLink() method defined next.
        fulcrumPinJoint.setDamping(0.3);

        secondPinJoint.setInitialState(-fulcrumInitialPositionRadians, fulcrumInitialVelocity);
        secondPinJoint.setLink(pendulumLink());// pendulumLink() method defined next.
        //secondPinJoint.setDamping(0.3);

        q_fulcrum = fulcrumPinJoint.getQ();
        qd_fulcrum = fulcrumPinJoint.getQD();
        tau_fulcrum = fulcrumPinJoint.getTau();

        q_Second = secondPinJoint.getQ();
        qd_Second = secondPinJoint.getQD();
        tau_Second = secondPinJoint.getTau();

        fulcrumPinJoint.addJoint(secondPinJoint);

        this.addRootJoint(fulcrumPinJoint);

        //this.addRootJoint(secondPinJoint);
    }

    /**
     * Fulcrum's angular position in radians
     * @return angular position in radians
     */
    public double getFulcrumAngularPosition()
    {
        return q_fulcrum.getDoubleValue();
    }

    /**
     * Fulcrum's angular velocity in radians per seconds
     * @return angular velocity in radians per seconds
     */
    public double getFulcrumAngularVelocity()
    {
        return qd_fulcrum.getDoubleValue();
    }

    /**
     * Fulcrum's torque in Newton meter
     * @return Torque in Newton meter
     */
    public double getFulcrumTorque()
    {
        return tau_fulcrum.getDoubleValue();
    }

    /**
     * Set Fulcrum's torque in Newton meter
     * @return Torque in Newton meter
     */
    public void setFulcrumTorque(double tau)
    {
        this.tau_fulcrum.set(tau);
    }

    public void setSecondTorque(double tau)
    {
        this.tau_Second.set(tau);
    }

    /**
     * Create the first link for the DoublePendulumRobot.
     */
    private Link pendulumLink()
    {
        Link pendulumLink = new Link("PendulumLink");
        pendulumLink.setMomentOfInertia(0.0, FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y, 0.0);
        pendulumLink.setMass(BALL_MASS);
        pendulumLink.setComOffset(0.0, 0.0, -ROD_LENGTH);

        Graphics3DObject pendulumGraphics = new Graphics3DObject();
        pendulumGraphics.addSphere(FULCRUM_RADIUS, YoAppearance.BlueViolet());
        pendulumGraphics.translate(0.0, 0.0, -ROD_LENGTH);
        pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, YoAppearance.Black());
        pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
        pendulumLink.setLinkGraphics(pendulumGraphics);

        return pendulumLink;
    }
//    private Link secondLink()
//    {
//        Link pendulumLink = new Link("PendulumLink");
//        pendulumLink.setMomentOfInertia(0.0, FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y, 0.0);
//        pendulumLink.setMass(BALL_MASS);
//        pendulumLink.setComOffset(0.0, 0.0, -ROD_LENGTH);
//
//        Graphics3DObject pendulumGraphics = new Graphics3DObject();
//        pendulumGraphics.addSphere(FULCRUM_RADIUS, YoAppearance.BlueViolet());
//        pendulumGraphics.translate(0.0, 0.0, -ROD_LENGTH);
//        pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, YoAppearance.Black());
//        pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
//        pendulumLink.setLinkGraphics(pendulumGraphics);
//
//        return pendulumLink;
//    }

}