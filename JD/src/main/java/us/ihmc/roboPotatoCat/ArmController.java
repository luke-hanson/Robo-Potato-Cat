package us.ihmc.roboPotatoCat;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;


public class ArmController implements RobotController
{
    // A name for this controller
    private final String name = "pendulumController";

    // This line instantiates a registry that will contain relevant controller variables that will be accessible from the simulation panel.
    private final YoVariableRegistry registry = new YoVariableRegistry("RobotController");

    // This is a reference to the SimplePendulumRobot that enables the controller to access this robot's variables.
    private ArmRobot robot;

   /* Control variables: */

    // Target angle
    private DoubleYoVariable desiredPositionRadians;

    // Controller parameter variables
    private DoubleYoVariable p_gain, d_gain, i_gain;

    private IntegerYoVariable RRotUp;
    private IntegerYoVariable LRotUp;
    private IntegerYoVariable crouch;

    private int iR = 0;
    private int iL = 0;
    private DoubleYoVariable iLCrouchHip;
    private DoubleYoVariable iRCrouchHip;
    private DoubleYoVariable iLCrouchKnee;
    private DoubleYoVariable iRCrouchKnee;

    private double theNumberWeAreCountingUpTo = 5000;

//    private DoubleYoVariable p_LRot, d_LRot, i_LRot;
//    private DoubleYoVariable p_RRot, d_RRot, i_RRot;
//    private DoubleYoVariable p_LFlap, d_LFlap, i_LFlap;
//    private DoubleYoVariable p_RFlap, d_RFlap, i_RFlap;
//    private DoubleYoVariable p_LBow, d_LBow, i_LBow;
//    private DoubleYoVariable p_RBow, d_RBow, i_RBow;
//    private DoubleYoVariable p_LHip, d_LHip, i_LHip;
//    private DoubleYoVariable p_RHip, d_RHip, i_RHip;
//    private DoubleYoVariable p_LKnee, d_LKnee, i_LKnee;
//    private DoubleYoVariable p_RKnee, d_RKnee, i_RKnee;
//    private DoubleYoVariable p_LAnk, d_LAnk, i_LAnk;
//    private DoubleYoVariable p_RAnk, d_RAnk, i_RAnk;



    // This is the desired torque that we will apply to the fulcrum joint (PinJoint)
    private double torque;

    /* Constructor:
       Where we instantiate and initialize control variables
    */
    public ArmController(ArmRobot robot)
    {
        this.robot = robot;
        desiredPositionRadians = new DoubleYoVariable("DesiredPosRad", registry);
        desiredPositionRadians.set(Math.PI);

        p_gain = new DoubleYoVariable("ProportionalGain", registry);
        p_gain.set(250.0);
        d_gain = new DoubleYoVariable("DerivativeGain", registry);
        d_gain.set(100.0);
        i_gain = new DoubleYoVariable("IntegralGain", registry);
        i_gain.set(10.0);

        RRotUp = new IntegerYoVariable("RRotUp", registry);
        RRotUp.set(0);
        LRotUp = new IntegerYoVariable("LRotUp", registry);
        LRotUp.set(0);
        crouch = new IntegerYoVariable("crouch", registry);
        crouch.set(0);

        iLCrouchHip = new DoubleYoVariable("iLCrouchHip", registry);
        iLCrouchHip.set(1);
        iRCrouchHip = new DoubleYoVariable("iRCrouchHip", registry);
        iRCrouchHip.set(1);
        iLCrouchKnee = new DoubleYoVariable("iLCrouchKnee", registry);
        iLCrouchKnee.set(1);
        iRCrouchKnee = new DoubleYoVariable("iRCrouchKnee", registry);
        iRCrouchKnee.set(1);
    }

    public void initialize()
    {

    }

    private double positionError = 0;
    private double integralError = 0;

    public void doControl()
    {
        LRotatorController();
        RRotatorController();
        LFlapperController();
        RFlapperController();
        LElbowController();
        RElbowController();
        LHipController();
        RHipController();
        LKneeController();
        RKneeController();
        LAnkleController();
        RAnkleController();
    }

    public void LRotatorController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        if(LRotUp.getIntegerValue() != 0)
        {
            positionError = (0) - robot.getLRotatorAngularPosition();
        }
        else
        {
            positionError = (desiredPositionRadians.getDoubleValue()) - robot.getLRotatorAngularPosition();
        }
        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getLRotatorAngularVelocity());

        robot.setLRotatorTorque(torque);
    }
    public void RRotatorController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        if(RRotUp.getIntegerValue() != 0)
        {
            positionError = -1*(desiredPositionRadians.getDoubleValue()) - robot.getRRotatorAngularPosition();
        }
        else
        {
            positionError = (0) - robot.getRRotatorAngularPosition();
        }

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getRRotatorAngularVelocity());

        robot.setRRotatorTorque(torque);
    }
    public void LFlapperController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getLFlapperAngularPosition();

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getLFlapperAngularVelocity());

        robot.setLFlapperTorque(torque);
    }
    public void RFlapperController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getRFlapperAngularPosition();

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getRFlapperAngularVelocity());

        robot.setRFlapperTorque(torque);
    }
    public void LElbowController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        if(LRotUp.getIntegerValue() != 0)
        {
            if (iL == 1000)
            {
                iL = 2000;
            }
            else if (iL == 1001)
            {
                iL = 0;
            }
            if (iL > 1000)
            {

                positionError = (desiredPositionRadians.getDoubleValue()*0.25) - (robot.getLElbowAngularPosition());
                iL--;
            }
            if (iL < 1000)
            {
                positionError = (desiredPositionRadians.getDoubleValue()*0.5) - (robot.getLElbowAngularPosition());
                iL++;
            }
        }
        else
        {
            positionError = (desiredPositionRadians.getDoubleValue() * 0.25) - robot.getLElbowAngularPosition();
        }

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getLElbowAngularVelocity());

        robot.setLElbowTorque(torque);
    }
    public void RElbowController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        if(RRotUp.getIntegerValue() != 0)
        {
            if (iR == 1000)
            {
                iR = 2000;
            }
            else if (iR == 1001)
            {
                iR = 0;
            }
            if (iR > 1000)
            {

                positionError = (desiredPositionRadians.getDoubleValue()*.25) - (robot.getRElbowAngularPosition());
                iR--;
            }
            if (iR < 1000)
            {
                positionError = (desiredPositionRadians.getDoubleValue()*0.5) - (robot.getRElbowAngularPosition());
                iR++;
            }
        }
        else
        {
            positionError = (desiredPositionRadians.getDoubleValue() * 0.25) - robot.getRElbowAngularPosition();
        }

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getRElbowAngularVelocity());

        robot.setRElbowTorque(torque);
    }
    public void LHipController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        if(crouch.getIntegerValue() != 0)
        {
            if(iLCrouchHip.getDoubleValue() < theNumberWeAreCountingUpTo)
            {
                positionError = -(((desiredPositionRadians.getDoubleValue())*(iLCrouchHip.getDoubleValue()/theNumberWeAreCountingUpTo))*0.75) - robot.getLHipAngularPosition();
                iLCrouchHip.add(1);
            }
            else
            {
                positionError = -(desiredPositionRadians.getDoubleValue()*0.75) - robot.getLHipAngularPosition();
            }
        }
        else
        {
            positionError = (desiredPositionRadians.getDoubleValue()*0) - robot.getLHipAngularPosition();
            iLCrouchHip.set(0);
        }

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getLHipAngularVelocity());

        robot.setLHipTorque(torque);
    }
    public void RHipController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        if(crouch.getIntegerValue() != 0)
        {
            if(iRCrouchHip.getDoubleValue() < theNumberWeAreCountingUpTo)
            {
                positionError = -(((desiredPositionRadians.getDoubleValue())*(iRCrouchHip.getDoubleValue()/theNumberWeAreCountingUpTo))*0.75) - robot.getRHipAngularPosition();
                iRCrouchHip.add(1);
            }
            else
            {
                positionError = -(desiredPositionRadians.getDoubleValue()*0.75) - robot.getRHipAngularPosition();

            }
        }
        else
        {
            positionError = (desiredPositionRadians.getDoubleValue()*0) - robot.getRHipAngularPosition();
            iRCrouchHip.set(0);
        }

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getRHipAngularVelocity());

        robot.setRHipTorque(torque);
    }
    public void LKneeController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        if(crouch.getIntegerValue() != 0)
        {
            if(iLCrouchKnee.getDoubleValue() < theNumberWeAreCountingUpTo)
            {
                positionError = (((desiredPositionRadians.getDoubleValue()*((iLCrouchKnee.getDoubleValue()/theNumberWeAreCountingUpTo)))/2)) - robot.getLKneeAngularPosition();
                iLCrouchKnee.add(1);
            }
            else
            {
                positionError = (desiredPositionRadians.getDoubleValue()*0.5) - robot.getLKneeAngularPosition();

            }
        }
        else
        {
            positionError = (desiredPositionRadians.getDoubleValue()*0) - robot.getLKneeAngularPosition();
            iLCrouchKnee.set(0);
        }

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getLKneeAngularVelocity());

        robot.setLKneeTorque(torque);
    }
    public void RKneeController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        if(crouch.getIntegerValue() != 0)
        {
            if(iRCrouchKnee.getDoubleValue() < theNumberWeAreCountingUpTo)
            {
                positionError = ((desiredPositionRadians.getDoubleValue()*(((iRCrouchKnee.getDoubleValue()/theNumberWeAreCountingUpTo)))/2)) - robot.getLKneeAngularPosition();
                iRCrouchKnee.add(1);
            }
            else
            {
                positionError = (desiredPositionRadians.getDoubleValue()*0.5) - robot.getLKneeAngularPosition();

            }
        }
        else
        {
            positionError = (desiredPositionRadians.getDoubleValue()*0) - robot.getLKneeAngularPosition();
            iRCrouchKnee.set(0);
        }

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getRKneeAngularVelocity());

        robot.setRKneeTorque(torque);
    }
    public void LAnkleController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        positionError = (0) - robot.getLAnkleAngularPosition();

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getLAnkleAngularVelocity());

        robot.setLAnkleTorque(torque);
    }
    public void RAnkleController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        positionError = (0) - robot.getRAnkleAngularPosition();

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getRAnkleAngularVelocity());

        robot.setRAnkleTorque(torque);
    }

    public YoVariableRegistry getYoVariableRegistry()
    {
        return registry;
    }

    public String getName()
    {
        return name;
    }

    public String getDescription()
    {
        return name;
    }
}
