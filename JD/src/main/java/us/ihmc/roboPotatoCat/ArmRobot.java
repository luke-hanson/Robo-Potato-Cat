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

import static us.ihmc.simulationconstructionset.FloatingPlanarJoint.XY;
import static us.ihmc.simulationconstructionset.FloatingPlanarJoint.XZ;

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
    ////NEED TO ADD OUR JOINTS SO THAT WE CAN MESS WITH THEM IN THE SIM
    //YES
    private DoubleYoVariable tau_LRotator, q_LRotator, qd_LRotator; // Respectively Torque, Position, Velocity
    private DoubleYoVariable tau_RRotator, q_RRotator, qd_RRotator;
    private DoubleYoVariable tau_LFlapper, q_LFlapper, qd_LFlapper;
    private DoubleYoVariable tau_RFlapper, q_RFlapper, qd_RFlapper;
    private DoubleYoVariable tau_LElbow, q_LElbow, qd_LElbow;
    private DoubleYoVariable tau_RElbow, q_RElbow, qd_RElbow;
    private DoubleYoVariable tau_LHip, q_LHip, qd_LHip;
    private DoubleYoVariable tau_RHip, q_RHip, qd_RHip;
    private DoubleYoVariable tau_LKnee, q_LKnee, qd_LKnee;
    private DoubleYoVariable tau_RKnee, q_RKnee, qd_RKnee;
    private DoubleYoVariable tau_LAnkle, q_LAnkle, qd_LAnkle;
    private DoubleYoVariable tau_RAnkle, q_RAnkle, qd_RAnkle;



    /*
       Define its constructor
     */
    public ArmRobot()
    {

        //` a. Call parent class "Robot" constructor. The string "SimplePendulum" will be the name of the robot.
        super("JD");

        //brought back the old one because he can stand now!
        FloatingJoint rootJoint = new FloatingJoint("FulcrumPin", new Vector3d(), this);
        rootJoint.setPosition(0, 0,3);

//        FloatingPlanarJoint rootJoint = new FloatingPlanarJoint("FulcrumPin", this, XZ);
//        rootJoint.changeOffsetVector(0, 0, 3); //2.65 is right at ground

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

        //for foot contact points
        PinJoint r1 = new PinJoint("r1", new Vector3d(-1*INCH_TO_MILLIMETER, -3*INCH_TO_MILLIMETER, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint r2 = new PinJoint("r2", new Vector3d(-1*INCH_TO_MILLIMETER, 0.0, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint r3 = new PinJoint("r3", new Vector3d(-1*INCH_TO_MILLIMETER, 1.5*INCH_TO_MILLIMETER, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint r4 = new PinJoint("r4", new Vector3d(1.75*INCH_TO_MILLIMETER, 1.75*INCH_TO_MILLIMETER, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint r5 = new PinJoint("r5", new Vector3d(1.75*INCH_TO_MILLIMETER, 0.0, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint r6 = new PinJoint("r6", new Vector3d(1*INCH_TO_MILLIMETER, -3*INCH_TO_MILLIMETER, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);

        PinJoint l1 = new PinJoint("l1", new Vector3d(1*INCH_TO_MILLIMETER, -3*INCH_TO_MILLIMETER, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint l2 = new PinJoint("l2", new Vector3d(1*INCH_TO_MILLIMETER, 0.0, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint l3 = new PinJoint("l3", new Vector3d(1*INCH_TO_MILLIMETER, 1.5*INCH_TO_MILLIMETER, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint l4 = new PinJoint("l4", new Vector3d(-1.75*INCH_TO_MILLIMETER, 1.75*INCH_TO_MILLIMETER, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint l5 = new PinJoint("l5", new Vector3d(-1.75*INCH_TO_MILLIMETER, 0.0, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);
        PinJoint l6 = new PinJoint("l6", new Vector3d(-1*INCH_TO_MILLIMETER, -3*INCH_TO_MILLIMETER, -0.5*INCH_TO_MILLIMETER), this, Axis.Y);

        leftShoulderFlapper.setLimitStops(-Math.PI/2,Math.PI/2,10,50);
        leftElbow.setLimitStops(-Math.PI/2,Math.PI/2,10,50);
        rightShoulderFlapper.setLimitStops(-Math.PI/2,Math.PI/2,10,50);
        rightElbow.setLimitStops(-Math.PI/2,Math.PI/2,10,50);

        rightHip.setLimitStops(-Math.PI/2,Math.PI/2, 10, 50);
        leftHip.setLimitStops(-Math.PI/2,Math.PI/2, 10, 50);
        rightKnee.setLimitStops(-Math.PI/2,Math.PI/2, 10, 100);
        leftKnee.setLimitStops(-Math.PI/2,Math.PI/2, 10, 100);


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
        rightShoulderRotator.setLink(servoPinAxisGraphicR());
        leftShoulderRotator.setLink(servoPinAxisGraphicL());
        rightShoulderFlapper.setLink(testSphereArmThighR());
        leftShoulderFlapper.setLink(testSphereArmThighL());
        rightElbow.setLink(testSphereForearmR());
        leftElbow.setLink(testSphereForearmL());
        rightHand.setLink(testSphereHand());
        leftHand.setLink(testSphereHand());

        rightHip.setLink(testSphereThighR());
        leftHip.setLink(testSphereThighL());
        rightKnee.setLink(testSphereLeg());
        leftKnee.setLink(testSphereLeg());
        rightAnkle.setLink(testSphereFootR());
        leftAnkle.setLink(testSphereFootL());

        r1.setLink(footsies());
        r2.setLink(footsies());
        r3.setLink(footsies());
        r4.setLink(footsies());
        r5.setLink(footsies());
        r6.setLink(footsies());

        l1.setLink(footsies());
        l2.setLink(footsies());
        l3.setLink(footsies());
        l4.setLink(footsies());
        l5.setLink(footsies());
        l6.setLink(footsies());

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

        rightAnkle.addJoint(r1);
        rightAnkle.addJoint(r2);
        rightAnkle.addJoint(r3);
        rightAnkle.addJoint(r4);
        rightAnkle.addJoint(r5);
        rightAnkle.addJoint(r6);

        leftAnkle.addJoint(l1);
        leftAnkle.addJoint(l2);
        leftAnkle.addJoint(l3);
        leftAnkle.addJoint(l4);
        leftAnkle.addJoint(l5);
        leftAnkle.addJoint(l6);

        //sets the position of the joint from the controllers
        q_LRotator = leftShoulderRotator.getQ();
        qd_LRotator = leftShoulderRotator.getQD();
        tau_LRotator = leftShoulderRotator.getTau();

        q_RRotator = rightShoulderRotator.getQ();
        qd_RRotator = rightShoulderRotator.getQD();
        tau_RRotator = rightShoulderRotator.getTau();

        q_LFlapper = leftShoulderFlapper.getQ();
        qd_LFlapper = leftShoulderFlapper.getQD();
        tau_LFlapper = leftShoulderFlapper.getTau();

        q_RFlapper = rightShoulderFlapper.getQ();
        qd_RFlapper = rightShoulderFlapper.getQD();
        tau_RFlapper = rightShoulderFlapper.getTau();

        q_LElbow = leftElbow.getQ();
        qd_LElbow = leftElbow.getQD();
        tau_LElbow= leftElbow.getTau();

        q_RElbow = rightElbow.getQ();
        qd_RElbow = rightElbow.getQD();
        tau_RElbow = rightElbow.getTau();

        q_LHip = leftHip.getQ();
        qd_LHip = leftHip.getQD();
        tau_LHip = leftHip.getTau();

        q_RHip = rightHip.getQ();
        qd_RHip = rightHip.getQD();
        tau_RHip = rightHip.getTau();

        q_LKnee = leftKnee.getQ();
        qd_LKnee = leftKnee.getQD();
        tau_LKnee = leftKnee.getTau();

        q_RKnee = rightKnee.getQ();
        qd_RKnee = rightKnee.getQD();
        tau_RKnee = rightKnee.getTau();

        q_LAnkle = leftAnkle.getQ();
        qd_LAnkle = leftAnkle.getQD();
        tau_LAnkle = leftAnkle.getTau();

        q_RAnkle = rightAnkle.getQ();
        qd_RAnkle = rightAnkle.getQD();
        tau_RAnkle = rightAnkle.getTau();



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

        GroundContactPoint groundContactPointR1 = new GroundContactPoint("r1", this);
        r1.addGroundContactPoint(groundContactPointR1);
        GroundContactPoint groundContactPointR2 = new GroundContactPoint("r2", this);
        r2.addGroundContactPoint(groundContactPointR2);
        GroundContactPoint groundContactPointR3 = new GroundContactPoint("r3", this);
        r3.addGroundContactPoint(groundContactPointR3);
        GroundContactPoint groundContactPointR4 = new GroundContactPoint("r4", this);
        r4.addGroundContactPoint(groundContactPointR4);
        GroundContactPoint groundContactPointR5 = new GroundContactPoint("r5", this);
        r5.addGroundContactPoint(groundContactPointR5);
        GroundContactPoint groundContactPointR6 = new GroundContactPoint("r6", this);
        r6.addGroundContactPoint(groundContactPointR6);

        GroundContactPoint groundContactPointL1 = new GroundContactPoint("l1", this);
        l1.addGroundContactPoint(groundContactPointL1);
        GroundContactPoint groundContactPointL2 = new GroundContactPoint("l2", this);
        l2.addGroundContactPoint(groundContactPointL2);
        GroundContactPoint groundContactPointL3 = new GroundContactPoint("l3", this);
        l3.addGroundContactPoint(groundContactPointL3);
        GroundContactPoint groundContactPointL4 = new GroundContactPoint("l4", this);
        l4.addGroundContactPoint(groundContactPointL4);
        GroundContactPoint groundContactPointL5 = new GroundContactPoint("l5", this);
        l5.addGroundContactPoint(groundContactPointL5);
        GroundContactPoint groundContactPointL6 = new GroundContactPoint("l6", this);
        l6.addGroundContactPoint(groundContactPointL6);

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
//    public double getFulcrumAngularPosition()
//    {
//        return q_fulcrum.getDoubleValue();
//    }
//    public double getSecondAngularPosition()
//    {
//        return q_Second.getDoubleValue();
//    }

    public double getLRotatorAngularPosition() { return q_LRotator.getDoubleValue(); }
    public double getRRotatorAngularPosition() { return q_RRotator.getDoubleValue(); }
    public double getLFlapperAngularPosition()
    {
        return q_LFlapper.getDoubleValue();
    }
    public double getRFlapperAngularPosition() { return q_RFlapper.getDoubleValue(); }
    public double getLElbowAngularPosition()
    {
        return q_LElbow.getDoubleValue();
    }
    public double getRElbowAngularPosition() { return q_RElbow.getDoubleValue(); }
    public double getLHipAngularPosition()
    {
        return q_LHip.getDoubleValue();
    }
    public double getRHipAngularPosition()
    {
        return q_RHip.getDoubleValue();
    }
    public double getLKneeAngularPosition()
    {
        return q_LKnee.getDoubleValue();
    }
    public double getRKneeAngularPosition()
    {
        return q_RKnee.getDoubleValue();
    }
    public double getLAnkleAngularPosition()
    {
        return q_LAnkle.getDoubleValue();
    }
    public double getRAnkleAngularPosition()
    {
        return q_RAnkle.getDoubleValue();
    }

    /*public double getSecondAngularPosition()
    {
        return q_Second.getDoubleValue();
    }*/

    /**
     * Fulcrum's angular velocity in radians per seconds
     * @return angular velocity in radians per seconds
     */
//    public double getFulcrumAngularVelocity()
//    {
//        return qd_fulcrum.getDoubleValue();
//    }
//    public double getSecondAngularVelocity()
//    {
//        return qd_Second.getDoubleValue();
//    }

    public double getLRotatorAngularVelocity() { return qd_LRotator.getDoubleValue(); }
    public double getRRotatorAngularVelocity() { return qd_RRotator.getDoubleValue(); }
    public double getLFlapperAngularVelocity()
    {
        return qd_LFlapper.getDoubleValue();
    }
    public double getRFlapperAngularVelocity() { return qd_RFlapper.getDoubleValue(); }
    public double getLElbowAngularVelocity()
    {
        return qd_LElbow.getDoubleValue();
    }
    public double getRElbowAngularVelocity() { return qd_RElbow.getDoubleValue(); }
    public double getLHipAngularVelocity()
    {
        return qd_LHip.getDoubleValue();
    }
    public double getRHipAngularVelocity()
    {
        return qd_RHip.getDoubleValue();
    }
    public double getLKneeAngularVelocity()
    {
        return qd_LKnee.getDoubleValue();
    }
    public double getRKneeAngularVelocity()
    {
        return qd_RKnee.getDoubleValue();
    }
    public double getLAnkleAngularVelocity()
    {
        return qd_LAnkle.getDoubleValue();
    }
    public double getRAnkleAngularVelocity()
    {
        return qd_RAnkle.getDoubleValue();
    }
    /**
     * Fulcrum's torque in Newton meter
     * @return Torque in Newton meter
     */
    //public double getFulcrumTorque() { return tau_fulcrum.getDoubleValue(); }
    //if something breaks, try uncommenting this
    /**
     * Set Fulcrum's torque in Newton meter
     * @return Torque in Newton meter
     */
//    public void setFulcrumTorque(double tau)
//    {
//        this.tau_fulcrum.set(tau);
//    }
//    public void setSecondTorque(double tau)
//    {
//        this.tau_Second.set(tau);
//    }

    public void setLRotatorTorque(double tau)
    {
        this.tau_LRotator.set(tau);
    }
    public void setRRotatorTorque(double tau)
    {
        this.tau_RRotator.set(tau);
    }
    public void setLFlapperTorque(double tau) { this.tau_LFlapper.set(tau); }
    public void setRFlapperTorque(double tau)
    {
        this.tau_RFlapper.set(tau);
    }
    public void setLElbowTorque(double tau)
    {
        this.tau_LElbow.set(tau);
    }
    public void setRElbowTorque(double tau)
    {
        this.tau_RElbow.set(tau);
    }
    public void setLHipTorque(double tau)
    {
        this.tau_LHip.set(tau);
    }
    public void setRHipTorque(double tau)
    {
        this.tau_RHip.set(tau);
    }
    public void setLKneeTorque(double tau)
    {
        this.tau_LKnee.set(tau);
    }
    public void setRKneeTorque(double tau)
    {
        this.tau_RKnee.set(tau);
    }
    public void setLAnkleTorque(double tau)
    {
        this.tau_LAnkle.set(tau);
    }
    public void setRAnkleTorque(double tau)
    {
        this.tau_RAnkle.set(tau);
    }





    //graphics bits are self-documenting :D
    //no they're not D:
    //individual methods for everything AAAAHHHH!!!

    private Link servoPinAxisGraphicL()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.rotate((Math.PI/2), Axis.Y);
        servoHeadGraphics.translate(0.0, 0.0, 0.0);//0.0835 is one half of .167(the cylinders height) setting this value in the x pos negative centers the graphic on the center of the virtual object
        servoHeadGraphics.addCylinder(.6, .15, YoAppearance.Black());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link servoPinAxisGraphicR()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.rotate((Math.PI/2), Axis.Y);
        servoHeadGraphics.translate(0.0, 0.0, -.5);//0.0835 is one half of .167(the cylinders height) setting this value in the x pos negative centers the graphic on the center of the virtual object
        servoHeadGraphics.addCylinder(.6, .15, YoAppearance.Black());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link coreGraphic()
    {
        Link body = new Link("body");
        body.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        body.setMass(BALL_MASS);

        Graphics3DObject bodyGraphics = new Graphics3DObject();
        bodyGraphics.addCube(3.75*INCH_TO_MILLIMETER, SERVO_JOINT_LENGTH, 2.625*INCH_TO_MILLIMETER, YoAppearance.White());//x=width y=depth z=height looking at the robot
        bodyGraphics.rotate((Math.PI/2), Axis.Z);
        bodyGraphics.translate(0.0, 0.875*INCH_TO_MILLIMETER, -.5);
        bodyGraphics.addCylinder(.5, .13, YoAppearance.Black());
        bodyGraphics.translate(0.0, -0.875*INCH_TO_MILLIMETER*2, 0.0);
        bodyGraphics.addCylinder(.5, .13, YoAppearance.Black());
        bodyGraphics.translate(0.0, 0.875*INCH_TO_MILLIMETER, 0.5);
        bodyGraphics.translate(0.0, 0.0, 1);
        bodyGraphics.addCylinder(.5, .13, YoAppearance.Black());
        //changing this translate affects the head
        bodyGraphics.translate(0.0, 0.0, 1);
        bodyGraphics.addSphere(.6, YoAppearance.White());//the actual head
        bodyGraphics.rotate((Math.PI*0.2), Axis.Z);
        bodyGraphics.rotate((Math.PI*0.2), Axis.Y);
        bodyGraphics.translate(0.0, 0.05, .44);
        bodyGraphics.addCylinder(.7, .4, YoAppearance.Black());
        bodyGraphics.addCylinder(.05, .7, YoAppearance.Black());
        body.setLinkGraphics(bodyGraphics);

        return body;
    }

    private Link testSphereThighR()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.addSphere(0.25, YoAppearance.White());
        servoHeadGraphics.translate(0.0, 0.0, -2.375*INCH_TO_MILLIMETER);
        servoHeadGraphics.addCylinder(2.375*INCH_TO_MILLIMETER, 0.12, YoAppearance.Black());
        servoHeadGraphics.addSphere(.2, YoAppearance.Chartreuse());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link testSphereThighL()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.addSphere(0.25, YoAppearance.White());
        //adding the groin bar
        servoHeadGraphics.rotate((Math.PI/2), Axis.Y);
        servoHeadGraphics.addCylinder(.6, 0.12, YoAppearance.Black());
        servoHeadGraphics.rotate(-(Math.PI/2), Axis.Y);
        servoHeadGraphics.translate(0.0, 0.0, -2.375*INCH_TO_MILLIMETER);
        servoHeadGraphics.addCylinder(2.375*INCH_TO_MILLIMETER, 0.12, YoAppearance.Black());
        servoHeadGraphics.addSphere(.2, YoAppearance.Chartreuse());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link testSphereLeg()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.addSphere(.27, YoAppearance.White());

        servoHeadGraphics.translate(0.0, 0.0, -2.125*INCH_TO_MILLIMETER);
        servoHeadGraphics.addCylinder(2.125*INCH_TO_MILLIMETER, 0.12, YoAppearance.Black());
        servoHeadGraphics.addSphere(.2, YoAppearance.Chartreuse());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link testSphereFootR()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.addSphere(.25, YoAppearance.Gray());
        servoHeadGraphics.translate(0.12, -0.22, -0.625*INCH_TO_MILLIMETER);
        servoHeadGraphics.addHemiEllipsoid(0.4, 0.7, 0.4, YoAppearance.White());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link testSphereFootL()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.addSphere(.25, YoAppearance.Gray());
        servoHeadGraphics.translate(-0.12, -0.22, -0.625*INCH_TO_MILLIMETER);
        servoHeadGraphics.addHemiEllipsoid(.4, .7, .4, YoAppearance.White());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link testSphereArmThighL()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.rotate((Math.PI/2), Axis.Y);
        servoHeadGraphics.addSphere(.4, YoAppearance.White());
        servoHeadGraphics.translate(0.0, 0.0, -2.75*INCH_TO_MILLIMETER);
        servoHeadGraphics.addCylinder(2.75*INCH_TO_MILLIMETER, .2, YoAppearance.Black());
        servoHeadGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link testSphereArmThighR()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.rotate((Math.PI/2), Axis.Y);
        servoHeadGraphics.addSphere(.4, YoAppearance.White());
        servoHeadGraphics.translate(0.0, 0.0, 0);
        servoHeadGraphics.addCylinder(2.75*INCH_TO_MILLIMETER, .2, YoAppearance.Black());
        servoHeadGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link testSphereForearmR()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.rotate((Math.PI/2), Axis.Y);
        servoHeadGraphics.addSphere(.3, YoAppearance.White());
        servoHeadGraphics.translate(0.0, 0.0, 0.0);
        servoHeadGraphics.addCylinder(2.75*INCH_TO_MILLIMETER, .2, YoAppearance.Black());
        servoHeadGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link testSphereForearmL()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.rotate((Math.PI/2), Axis.Y);
        servoHeadGraphics.addSphere(.3, YoAppearance.White());
        servoHeadGraphics.translate(0.0, 0.0, -2.75*INCH_TO_MILLIMETER);
        servoHeadGraphics.addCylinder(2.75*INCH_TO_MILLIMETER, .2, YoAppearance.Black());
        servoHeadGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link testSphereHand()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(BALL_MASS);

        Graphics3DObject servoHeadGraphics = new Graphics3DObject();

        servoHeadGraphics.addSphere(.32, YoAppearance.White());
        servoHeadGraphics.addCylinder(.05, .4, YoAppearance.Black());
        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }

    private Link footsies()
    {
        Link servo = new Link("servoPin");
        servo.setMomentOfInertia(FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X, FULCRUM_MOMENT_OF_INERTIA_ABOUT_X);
        servo.setMass(1);

//        Graphics3DObject servoHeadGraphics = new Graphics3DObject();
//
//        servoHeadGraphics.addSphere(.1, YoAppearance.White());
//        servoHeadGraphics.addCylinder(.05, .4, YoAppearance.Black());
//        servo.setLinkGraphics(servoHeadGraphics);

        return servo;
    }
}
