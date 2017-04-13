package us.ihmc.roboPotatoCat;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;


public class ArmSimulation
{

    public static final double DT = 0.001;
    private SimulationConstructionSet sim;

    public ArmSimulation()
    {
        ArmRobot robot = new ArmRobot();
        robot.setController(new ArmController(robot));

        SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
        parameters.setDataBufferSize(32000);

        sim = new SimulationConstructionSet(robot, parameters);
        sim.setDT(DT, 20);
        sim.setGroundVisible(true);
        sim.setCameraPosition(0, -50.0, 8);
        sim.setCameraFix(0.0, 0.0, 0.70);

        sim.setSimulateDuration(60.0); // sets the simulation duration to 60 seconds

        // x, y, z = red, white, blue
        Graphics3DObject coordinateSystem = new Graphics3DObject();
        coordinateSystem.addCoordinateSystem(0.5);
        sim.addStaticLinkGraphics(coordinateSystem);

        Thread myThread = new Thread(sim);
        myThread.start();
    }

    public static void main(String[] args)
    {
        new ArmSimulation();
    }
}

