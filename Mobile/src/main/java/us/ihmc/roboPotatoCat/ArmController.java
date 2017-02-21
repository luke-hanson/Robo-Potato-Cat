package us.ihmc.roboPotatoCat;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;


public class ArmController implements RobotController
{
    // A name for this controller
    private final String name = "pendulumController";

    // This line instantiates a registry that will contain relevant controller variables that will be accessible from the simulation panel.
    private final YoVariableRegistry registry = new YoVariableRegistry("PendulumController");
    private final YoVariableRegistry secondRegistry = new YoVariableRegistry("SecondController");

    // This is a reference to the SimplePendulumRobot that enables the controller to access this robot's variables.
    private ArmRobot robot;

   /* Control variables: */

    // Target angle
    private DoubleYoVariable desiredPositionRadians;

    // Controller parameter variables
    private DoubleYoVariable p_gain, d_gain, i_gain;
    private DoubleYoVariable p_sec, d_sec, i_sec;

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

        p_sec = new DoubleYoVariable("ProportionalGain", secondRegistry);
        p_sec.set(250.0);
        d_sec = new DoubleYoVariable("DerivativeGain", secondRegistry);
        d_sec.set(100.0);
        i_sec = new DoubleYoVariable("IntegralGain", secondRegistry);
        i_sec.set(10.0);
    }

    public void initialize()
    {

    }

    private double positionError = 0;
    private double integralError = 0;

    public void doControl()
    {
        fulcrumController();
        secondController();
    }

    public void fulcrumController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        positionError = desiredPositionRadians.getDoubleValue() - robot.getFulcrumAngularPosition();

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_gain.getDoubleValue() * positionError +
                i_gain.getDoubleValue() * integralError +
                d_gain.getDoubleValue() * (0 - robot.getFulcrumAngularVelocity());


        robot.setFulcrumTorque(torque);
    }

    public void secondController()
    {
        // ERROR term: Compute the difference between the desired position the pendulum and its current position
        positionError = 0 - robot.getSecondAngularPosition();

        // INTEGRAL term: Compute a simple numerical integration of the position error
        integralError += positionError * ArmSimulation.DT;   //

        // P.I.D
        torque = p_sec.getDoubleValue() * positionError +
                i_sec.getDoubleValue() * integralError +
                d_sec.getDoubleValue() * (0 - robot.getSecondAngularVelocity());

        robot.setSecondTorque(torque);
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
