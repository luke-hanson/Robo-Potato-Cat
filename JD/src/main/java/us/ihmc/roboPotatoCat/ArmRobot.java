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
    public static final double INCH_TO_MILLIMETER = (1 * 25.4)/60;
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
    //YES
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

        //instantiate new joints here - the vector3d is the point in space that the new part exists(i think)
        PinJoint rightShoulderRotator = new PinJoint("rightShoulderRotator", new Vector3d(3*INCH_TO_MILLIMETER, 0.0, .75), this, Axis.X);//make sure to measure jd and adjust these Zs
        PinJoint leftShoulderRotator = new PinJoint("leftShoulderRotator", new Vector3d(-3*INCH_TO_MILLIMETER, 0.0, 0.75), this, Axis.X);
        PinJoint rightShoulderFlapper = new PinJoint("rightShoulderFlapper", new Vector3d(0.0625*INCH_TO_MILLIMETER, 0.0, 0), this, Axis.Y);
        PinJoint leftShoulderFlapper = new PinJoint("leftShoulderFlapper", new Vector3d(-0.0625*INCH_TO_MILLIMETER, 0.0, 0), this, Axis.Y);
        PinJoint rightElbow = new PinJoint("rightElbow", new Vector3d(2.75*INCH_TO_MILLIMETER, 0.0, 0), this, Axis.Y);
        PinJoint leftElbow = new PinJoint("leftElbow", new Vector3d(-2.75*INCH_TO_MILLIMETER, 0.0, 0), this, Axis.Y);
        PinJoint rightHand = new PinJoint("rightHand", new Vector3d(2.875*INCH_TO_MILLIMETER, 0.0, 0), this, Axis.X);
        PinJoint leftHand = new PinJoint("leftHand", new Vector3d(-2.875*INCH_TO_MILLIMETER, 0.0, 0), this, Axis.X);

        PinJoint rightHip = new PinJoint("rightHip", new Vector3d(0.875*INCH_TO_MILLIMETER, 0.0, -1.125*INCH_TO_MILLIMETER), this, Axis.X);
        PinJoint leftHip = new PinJoint("leftHip", new Vector3d(-0.875*INCH_TO_MILLIMETER, 0.0, -1.125*INCH_TO_MILLIMETER), this, Axis.X);
        PinJoint rightKnee = new PinJoint("rightKnee", new Vector3d(0, 0, -2.375*INCH_TO_MILLIMETER), this, Axis.X);
        PinJoint leftKnee = new PinJoint("leftKnee", new Vector3d(0, 0, -2.375*INCH_TO_MILLIMETER), this, Axis.X);
        PinJoint rightAnkle = new PinJoint("rightAnkle", new Vector3d(0, 0.0, -2.125*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint leftAnkle = new PinJoint("leftAnkle", new Vector3d(0, 0.0, -2.125*INCH_TO_MILLIMETER), this, Axis.Y);
        //might need to add more joints for ground contact points on the feet, or maybe we can just offset GCPs from the ankles?
        //PinJoint TEST = new PinJoint("test", new Vector3d(-0.5,0,-1), this, Axis.X);

        //damping = how tight the joints are
        rightShoulderRotator.setDamping(0.3);
        leftShoulderRotator.setDamping(0.3);
        rightShoulderFlapper.setDamping(0.3);
        leftShoulderFlapper.setDamping(0.3);
        rightElbow.setDamping(0.3);
        leftElbow.setDamping(0.3);
        rightHand.setDamping(0.3);
        leftHand.setDamping(0.3);

        rightHip.setDamping(0.3);
        leftHip.setDamping(0.3);
        rightKnee.setDamping(0.3);
        leftKnee.setDamping(0.3);
        rightAnkle.setDamping(0.3);
        leftAnkle.setDamping(0.3);

        //assign a graphic
        rightShoulderRotator.setLink(servoPinAxisGraphic());
        leftShoulderRotator.setLink(servoPinAxisGraphic());
        rightShoulderFlapper.setLink(servoPinAxisGraphicY());
        leftShoulderFlapper.setLink(servoPinAxisGraphicY());
        rightElbow.setLink(servoPinAxisGraphicY());
        leftElbow.setLink(servoPinAxisGraphicY());
        rightHand.setLink(servoPinAxisGraphic());
        leftHand.setLink(servoPinAxisGraphic());

        rightHip.setLink(servoPinAxisGraphic());
        leftHip.setLink(servoPinAxisGraphic());
        rightKnee.setLink(servoPinAxisGraphic());
        leftKnee.setLink(servoPinAxisGraphic());
        rightAnkle.setLink(servoPinAxisGraphicY());
        leftAnkle.setLink(servoPinAxisGraphicY());
        //TEST.setLink(servoPinAxisGraphic());

        //attach joints to each other
        rootJoint.addJoint(rightShoulderRotator);
        rootJoint.addJoint(leftShoulderRotator);
        rootJoint.addJoint(leftHip);
        rootJoint.addJoint(rightHip);

        rightShoulderRotator.addJoint(rightShoulderFlapper);
        leftShoulderRotator.addJoint(leftShoulderFlapper);
        rightShoulderFlapper.addJoint(rightElbow);
        leftShoulderFlapper.addJoint(leftElbow);
        rightElbow.addJoint(rightHand);
        leftElbow.addJoint(leftHand);

        rightHip.addJoint(rightKnee);
        leftHip.addJoint(leftKnee);
        rightKnee.addJoint(rightAnkle);
        leftKnee.addJoint(leftAnkle);
        //rightAnkle.addJoint(TEST);

        //initial positions of joints
        //rightShoulderRotator.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);
        //rightHip.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);
        //rightKnee.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);

        rootJoint.setLink(coreGraphic());
        this.addRootJoint(rootJoint);

        //ground contact modeling bit  no contacts for just links?
        //each new contact point needs a new GroundContactPoint as below
        GroundContactPoint groundContactPointRSR = new GroundContactPoint("rightShoulderRotator", this);
        //and it will also need to be attached to a joint or link as below
        rightShoulderRotator.addGroundContactPoint(groundContactPointRSR);
        //so ONE ground contact point starts here
        GroundContactPoint groundContactPointLSR = new GroundContactPoint("leftShoulderRotator", this);
        leftShoulderRotator.addGroundContactPoint(groundContactPointLSR);
        //and ends here
        GroundContactPoint groundContactPointRSF = new GroundContactPoint("rightShoulderFlapper", this);
        rightShoulderFlapper.addGroundContactPoint(groundContactPointRSF);

        GroundContactPoint groundContactPointLSF = new GroundContactPoint("leftShoulderFlapper", this);
        leftShoulderFlapper.addGroundContactPoint(groundContactPointLSF);

        GroundContactPoint groundContactPointRE = new GroundContactPoint("rightElbow", this);
        rightElbow.addGroundContactPoint(groundContactPointRE);

        GroundContactPoint groundContactPointLE = new GroundContactPoint("leftElbow", this);
        leftElbow.addGroundContactPoint(groundContactPointLE);

        GroundContactPoint groundContactPointRHn = new GroundContactPoint("rightHand", this);
        rightHand.addGroundContactPoint(groundContactPointRHn);

        GroundContactPoint groundContactPointLHn = new GroundContactPoint("leftHand", this);
        leftHand.addGroundContactPoint(groundContactPointLHn);


        GroundContactPoint groundContactPointRH = new GroundContactPoint("rightHip", this);
        rightHip.addGroundContactPoint(groundContactPointRH);

        GroundContactPoint groundContactPointLH = new GroundContactPoint("leftHip", this);
        leftHip.addGroundContactPoint(groundContactPointLH);

        GroundContactPoint groundContactPointRK = new GroundContactPoint("rightKnee", this);
        rightKnee.addGroundContactPoint(groundContactPointRK);

        GroundContactPoint groundContactPointLK = new GroundContactPoint("leftKnee", this);
        leftKnee.addGroundContactPoint(groundContactPointLK);

        GroundContactPoint groundContactPointRA = new GroundContactPoint("rightAnkle", this);
        rightAnkle.addGroundContactPoint(groundContactPointRA);

        GroundContactPoint groundContactPointLA = new GroundContactPoint("leftAnkle", this);
        leftAnkle.addGroundContactPoint(groundContactPointLA);

//        GroundContactPoint groundContactPointTEST = new GroundContactPoint("test", this);
//        TEST.addGroundContactPoint(groundContactPointTEST);

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

    //graphics bits are self-documenting :D
    private Link legSegment()
    {
        Link legSegment = new Link("legSegment");
        legSegment.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        legSegment.setMass(BALL_MASS);
        //legSegment.setComOffset(0.0, 0.0, -1);

        Graphics3DObject legSegmentGraphics = new Graphics3DObject();
        legSegmentGraphics.translate(-.5, 0, -1);// this translation helps place the leg segment where it "belongs"
        legSegmentGraphics.addCube(.3,.3,.6, YoAppearance.DarkMagenta());
        legSegment.setLinkGraphics(legSegmentGraphics);

        return legSegment;
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
    //made this for the Y position, but it seems to mess with positioning
    private Link servoPinAxisGraphicY()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();
        servoHeadGraphics.translate(-0.0835, 0.0, 0.0);//0.0835 is one half of .167(the cylinders height) setting this value in the x pos negative centers the graphic on the center of the virtual object
        servoHeadGraphics.rotate((Math.PI/2), Axis.Y);
        servoHeadGraphics.rotate((Math.PI/2), Axis.X);
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
        bodyGraphics.addCube(3.75*INCH_TO_MILLIMETER, SERVO_JOINT_LENGTH, 2.625*INCH_TO_MILLIMETER, YoAppearance.AntiqueWhite());//x=width y=depth z=height looking at the robot
        body.setLinkGraphics(bodyGraphics);

        return body;
    }
}
