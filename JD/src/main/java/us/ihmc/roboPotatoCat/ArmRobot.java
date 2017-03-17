package us.ihmc.roboPotatoCat;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

import javax.vecmath.Vector3d;

/**
 *
 * lengths are expressed in meters (m), masses in kilograms (kg)
 *
 */
public class ArmRobot extends Robot
{
    /*
       Define the parameters of the robot
    */

    public static final double SERVO_JOINT_LENGTH = 1; //1 = 60mm everything is extrapolated from that
    public static final double ROD_RADIUS = 0.01;
    public static final double ROD_MASS = 0.00;

    public static final double FULCRUM_RADIUS = 0.02;

    public static final double BALL_RADIUS = 0.05;
    public static final double BALL_MASS = 1.0;

    public static final double FULCRUM_MOMENT_OF_INERTIA_ABOUT_X =
            SERVO_JOINT_LENGTH * SERVO_JOINT_LENGTH * BALL_MASS; // I = mrˆ2 pendulum's resistance to changes to its rotation in  kg.mˆ2

    private double fulcrumInitialPositionDegrees = 90.0;
    private double fulcrumInitialPositionRadians = fulcrumInitialPositionDegrees * Math.PI / 180.0;
    private double fulcrumInitialVelocity = 0.0;

    /* Some joint state variables */
    ////NEED TO ADD OUR JOINTS SO THAT WE CAN MESS WITH THTEM IN THE SIM
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
        rootJoint.setPosition(0, 0,4);

        //instansiate new joints here - the vector3d is the point in space that the new part exists(i think)
        PinJoint rightShoulderRotator = new PinJoint("rightShoulderRotator", new Vector3d(.78, 0.0, .75), this, Axis.X);
        PinJoint leftShoulderRotator = new PinJoint("leftShoulderRotator", new Vector3d(-0.78, 0.0, 0.75), this, Axis.X);
        PinJoint leftHip = new PinJoint("leftHip", new Vector3d(-0.5, 0.0, 0), this, Axis.X);
        PinJoint rightHip = new PinJoint("rightHip", new Vector3d(0.5, 0.0, 0), this, Axis.X);


        //damping = how tight the joints are
        rightShoulderRotator.setDamping(0.3);
        leftShoulderRotator.setDamping(0.3);
        leftHip.setDamping(0.3);
        rightHip.setDamping(0.3);

        //assign a graphic
        Link rightShoulderRotatorLink = servoPinAxisGraphic();
        Link leftShoulderRotatorLink = servoPinAxisGraphic();
        Link leftHipLink = servoPinAxisGraphic();
        Link rightHipLink = servoPinAxisGraphic();

        //set the root joint
        rootJoint.setLink(rightShoulderRotatorLink);
        rootJoint.setLink(leftShoulderRotatorLink);
        rootJoint.setLink(leftHipLink);
        rootJoint.setLink(rightHipLink);

        //set graphics a different way or it breaks?
        rightShoulderRotator.setLink(servoPinAxisGraphic());
        leftShoulderRotator.setLink(servoPinAxisGraphic());
        leftHip.setLink(servoPinAxisGraphic());
        rightHip.setLink(servoPinAxisGraphic());

        //im not sure that i remember this bit
        rootJoint.addJoint(rightShoulderRotator);
        rootJoint.addJoint(leftShoulderRotator);
        rootJoint.addJoint(leftHip);
        rootJoint.addJoint(rightHip);

        //initial positions of joints
        rightShoulderRotator.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);

        rootJoint.setLink(coreGraphic());
        this.addRootJoint(rootJoint);

        //ground contact modeling bit
        //each new contact point needs a new GroundContactPoint as below
        GroundContactPoint groundContactPoint = new GroundContactPoint("rootJointGcp", this);
        //and it will also need to be attached to a joint or link as below
        rightShoulderRotator.addGroundContactPoint(groundContactPoint);
        GroundContactPoint groundContactPointLS = new GroundContactPoint("leftShoulder", this);
        leftShoulderRotator.addGroundContactPoint(groundContactPointLS);
        GroundContactPoint groundContactPointRH = new GroundContactPoint("rightHip", this);
        rightHip.addGroundContactPoint(groundContactPointRH);
        GroundContactPoint groundContactPointLH = new GroundContactPoint("leftHip", this);
        leftHip.addGroundContactPoint(groundContactPointLH);

        //leave this bit alone!
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
        pendulumLink.setComOffset(0.0, 0.0, -SERVO_JOINT_LENGTH);

        Graphics3DObject pendulumGraphics = new Graphics3DObject();
        pendulumGraphics.addSphere(FULCRUM_RADIUS, YoAppearance.Crimson());
        pendulumGraphics.translate(0.0, 0.0, -SERVO_JOINT_LENGTH);
        pendulumGraphics.addCylinder(SERVO_JOINT_LENGTH, ROD_RADIUS, YoAppearance.Black());
        pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
        pendulumLink.setLinkGraphics(pendulumGraphics);

        return pendulumLink;
    }
    private Link servoPinAxisGraphic()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();
        servoHeadGraphics.translate(-0.0835, 0.0, 0.0);//0.0835 is one half of .167(the cylinders height) setting this value in the x pos negative centers the graphic on the center of the virtual object
        servoHeadGraphics.rotate((Math.PI/2), Axis.Y);
        servoHeadGraphics.addCylinder(.167, .084, YoAppearance.Black());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }
    private Link coreGraphic()
    {
        Link body = new Link("body");
        body.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        body.setMass(BALL_MASS);

        Graphics3DObject bodyGraphics = new Graphics3DObject();
        bodyGraphics.translate(0.0, 0.0, 0);
        bodyGraphics.addCube(1.585, SERVO_JOINT_LENGTH, SERVO_JOINT_LENGTH, YoAppearance.AntiqueWhite());//x=width y=depth z=height looking at the robot
        body.setLinkGraphics(bodyGraphics);

        return body;
    }
}