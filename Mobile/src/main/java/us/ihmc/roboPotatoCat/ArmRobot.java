package us.ihmc.roboPotatoCat;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.WavyGroundProfile;

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

    public static final double FULCRUM_MOMENT_OF_INERTIA_ABOUT_X =
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
        super("JD");

        FloatingJoint rootJoint = new FloatingJoint("FulcrumPin", new Vector3d(), this);
        rootJoint.setPosition(0, 0,3);
        //PinJoint rootJoint = new PinJoint("rootJoint", new Vector3d(2.0, 0.0, 2.3125), this, Axis.X);

        PinJoint rightShoulderRotator = new PinJoint("q1", new Vector3d(2.0, 0.0, 2.3125), this, Axis.X);
        PinJoint leftShoulderRotator = new PinJoint("q2", new Vector3d(-2.0, 0.0, 2.3125), this, Axis.X);
        PinJoint leftHip = new PinJoint("q3", new Vector3d(-0.875, 0.0, -2.4375), this, Axis.X);
        PinJoint rightHip = new PinJoint("q4", new Vector3d(0.875, 0.0, -2.4375), this, Axis.X);


        rightShoulderRotator.setDamping(0.3);
        leftShoulderRotator.setDamping(0.3);
        leftHip.setDamping(0.3);
        rightHip.setDamping(0.3);


        Link rightShoulderRotatorLink = secondLink();
        Link leftShoulderRotatorLink = secondLink();
        Link leftHipLink = secondLink();
        Link rightHipLink = secondLink();

        rootJoint.setLink(rightShoulderRotatorLink);
        rootJoint.setLink(leftShoulderRotatorLink);
        rootJoint.setLink(leftHipLink);
        rootJoint.setLink(rightHipLink);

        rightShoulderRotator.setLink(secondLink());
        leftShoulderRotator.setLink(secondLink());
        leftHip.setLink(secondLink());
        rightHip.setLink(secondLink());

        rootJoint.addJoint(rightShoulderRotator);
        rootJoint.addJoint(leftShoulderRotator);
        rootJoint.addJoint(leftHip);
        rootJoint.addJoint(rightHip);
        rightShoulderRotator.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);

        rootJoint.setLink(coreGraphic());
        this.addRootJoint(rootJoint);
        // b. Add a joint to the robot
//        PinJoint fulcrumPinJoint = new PinJoint("FulcrumPin", new Vector3d(0.0, 0.0, 1.25), this, Axis.Y);
//        PinJoint secondPinJoint = new PinJoint("SecondPin", new Vector3d(0.0, 0.0, -ROD_LENGTH), this, Axis.Y);
//        PinJoint zPinJoint = new PinJoint("zPin", new Vector3d(0.0, 0.0, 0.0), this, Axis.X);
//
//        fulcrumPinJoint.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);
//        fulcrumPinJoint.setDamping(0.3);
//
//        zPinJoint.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);
//
//        Link secondLink = secondLink();
//        fulcrumPinJoint.setLink(secondLink);// pendulumLink() method defined next.
//
//        Link coreGraphic = coreGraphic();
//        fulcrumPinJoint.setLink(coreGraphic);// pendulumLink() method defined next.
//
//
//
//        secondPinJoint.setInitialState(-fulcrumInitialPositionRadians, fulcrumInitialVelocity);
//        secondPinJoint.setLink(pendulumLink());// pendulumLink() method defined next.
//        secondPinJoint.setDamping(0.3);
//
//        zPinJoint.setInitialState(-fulcrumInitialPositionRadians, fulcrumInitialVelocity);
//        zPinJoint.setLink(pendulumLink());// pendulumLink() method defined next.
//        zPinJoint.setDamping(0.3);
//
//        q_fulcrum = fulcrumPinJoint.getQ();
//        qd_fulcrum = fulcrumPinJoint.getQD();
//        tau_fulcrum = fulcrumPinJoint.getTau();
//
//        q_Second = secondPinJoint.getQ();
//        qd_Second = secondPinJoint.getQD();
//        tau_Second = secondPinJoint.getTau();
//
//        fulcrumPinJoint.addJoint(secondPinJoint);
//
//        fulcrumPinJoint.addJoint(zPinJoint);
//
//        this.addRootJoint(fulcrumPinJoint);

        GroundContactPoint groundContactPoint = new GroundContactPoint("rootJointGcp", this);
        rightShoulderRotator.addGroundContactPoint(groundContactPoint);

        GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0,
                this.getRobotsYoVariableRegistry());

        GroundProfile3D profile = new FlatGroundProfile();
        groundModel.setGroundProfile3D(profile);
        this.setGroundContactModel(groundModel);
    }

    /**
     * Fulcrum's angular position in radians
     * @return angular position in radians
     */
    public double getFulcrumAngularPosition()
    {
        return q_fulcrum.getDoubleValue();
    }

    public double getSecondAngularPosition()
    {
        return q_Second.getDoubleValue();
    }

    /**
     * Fulcrum's angular velocity in radians per seconds
     * @return angular velocity in radians per seconds
     */
    public double getFulcrumAngularVelocity()
    {
        return qd_fulcrum.getDoubleValue();
    }
    public double getSecondAngularVelocity()
    {
        return qd_Second.getDoubleValue();
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
        pendulumLink.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        pendulumLink.setMass(BALL_MASS);
        pendulumLink.setComOffset(0.0, 0.0, -ROD_LENGTH);

        Graphics3DObject pendulumGraphics = new Graphics3DObject();
        pendulumGraphics.addSphere(FULCRUM_RADIUS, YoAppearance.Crimson());
        pendulumGraphics.translate(0.0, 0.0, -ROD_LENGTH);
        pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, YoAppearance.Black());
        pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
        pendulumLink.setLinkGraphics(pendulumGraphics);

        return pendulumLink;
    }
    private Link secondLink()
    {
        Link pendulumLink = new Link("FulcrumPin");
        pendulumLink.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        pendulumLink.setMass(BALL_MASS);
        //pendulumLink.setComOffset(0.0, 0.0, -ROD_LENGTH);

        Graphics3DObject pendulumGraphics = new Graphics3DObject();
        //pendulumGraphics.addSphere(FULCRUM_RADIUS, YoAppearance.BlueViolet());
        //pendulumGraphics.translate(0.0, 0.0, 0.0);
        pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, YoAppearance.AliceBlue());
        pendulumGraphics.rotate((Math.PI/2), Axis.Y);
        pendulumGraphics.addCylinder(ROD_LENGTH*.1, ROD_RADIUS*8, YoAppearance.Black());
        //pendulumGraphics.addCube(ROD_LENGTH, ROD_RADIUS, ROD_RADIUS, YoAppearance.LemonChiffon());


        //pendulumGraphics.addCoordinateSystem(1);
        //pendulumGraphics.addTeaPot(YoAppearance.PapayaWhip());
        //pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());


        pendulumLink.setLinkGraphics(pendulumGraphics);

        return pendulumLink;
    }
    private Link coreGraphic()
    {
        Link pendulumLink = new Link("PendulumLink");
        pendulumLink.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        pendulumLink.setMass(BALL_MASS);
        pendulumLink.setComOffset(0.0, 0.0, -ROD_LENGTH);

        Graphics3DObject pendulumGraphics = new Graphics3DObject();
        pendulumGraphics.addSphere(BALL_RADIUS*5, YoAppearance.Magenta());
        pendulumGraphics.translate(0.0, 0.0, -ROD_LENGTH);
        //pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, YoAppearance.Black());
        //pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
        pendulumLink.setLinkGraphics(pendulumGraphics);

        return pendulumLink;
    }
}