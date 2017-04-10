package us.ihmc.roboPotatoCat;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
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

    public double temp = 10;

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getLRotatorAngularPosition();

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getRRotatorAngularPosition();

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getLElbowAngularPosition();

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getRElbowAngularPosition();

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getLHipAngularPosition();

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getRHipAngularPosition();

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getLKneeAngularPosition();

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getRKneeAngularPosition();

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getLAnkleAngularPosition();

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
        positionError = (desiredPositionRadians.getDoubleValue() * .25) - robot.getRAnkleAngularPosition();

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
