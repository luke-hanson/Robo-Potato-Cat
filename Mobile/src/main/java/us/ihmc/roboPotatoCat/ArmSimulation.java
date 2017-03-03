package us.ihmc.roboPotatoCat;

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
        sim.setGroundVisible(false);
        sim.setCameraPosition(0, -9.0, 0.6);
        sim.setCameraFix(0.0, 0.0, 0.70);

        sim.setSimulateDuration(60.0); // sets the simulation duration to 60 seconds

        Thread myThread = new Thread(sim);
        myThread.start();
    }

    public static void main(String[] args)
    {
        new ArmSimulation();
    }
}

