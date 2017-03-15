package us.ihmc.exampleSimulations.linkExamples;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.MeshDataGenerator;
import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.vecmath.Point2d;
import java.util.ArrayList;


public class LinkExamplesSimulation
{
    private SimulationConstructionSet sim;

    private static final double SPHERE_R = 0.15;

    private static final double ELLIPSOID_RX = 0.1, ELLIPSOID_RY = 0.2, ELLIPSOID_RZ = 0.3;

    private static final double CYLINDER_H = 0.4, CYLINDER_R = 0.05;

    private static final double ARC_TORUS_START_ANG = 0.0, ARC_TORUS_END_ANG = 1.5 * Math.PI;
    private static final double ARC_TORUS_MAJ_RAD = 0.2, ARC_TORUS_MIN_RAD = 0.05;

    private static final double OFFSET = 2.5, COORD_LENGTH = 0.5;

    private static final double WEDGE_X = 0.4, WEDGE_Y = 0.3, WEDGE_Z = 0.2;

    public LinkExamplesSimulation()
    {
        Robot nullRob = null;
        sim = new SimulationConstructionSet(nullRob);
        // position the camera to view links
        sim.setCameraPosition(10.0, 6.0, 3.0);
        sim.setCameraFix(0.5, 0.5, 0.0);
        Link exampleShapes = exampleShapes();
        sim.addStaticLink(exampleShapes);
        sim.setGroundVisible(false);

        Thread myThread = new Thread(sim);
        myThread.start();
    }

    private Link exampleShapes()
    {
        Link ret = new Link("example shapes");
        Graphics3DObject linkGraphics = new Graphics3DObject();

        // Sphere
        linkGraphics.translate(OFFSET, 0.0, 0.0);
        linkGraphics.addCoordinateSystem(COORD_LENGTH);
        linkGraphics.addSphere(SPHERE_R, YoAppearance.Black());

        // Ellipsoid
        linkGraphics.translate(OFFSET, 0.0, 0.0);
        linkGraphics.addCoordinateSystem(COORD_LENGTH);
        linkGraphics.addEllipsoid(ELLIPSOID_RX,
                ELLIPSOID_RY,
                ELLIPSOID_RZ,
                YoAppearance.Black());

        // Cylinder
        linkGraphics.translate(-1 * OFFSET, 1.0, 0.0);
        linkGraphics.addCoordinateSystem(COORD_LENGTH);
        linkGraphics.addCylinder(CYLINDER_H, CYLINDER_R, YoAppearance.Black());

        // ArcTorus
        linkGraphics.translate(OFFSET, 0.0, 0.0);
        linkGraphics.addCoordinateSystem(COORD_LENGTH);
        linkGraphics.addArcTorus( ARC_TORUS_START_ANG,
                ARC_TORUS_END_ANG,
                ARC_TORUS_MAJ_RAD,
                ARC_TORUS_MIN_RAD,
                YoAppearance.Black());

        // Extruded Polygon
        linkGraphics.translate(-1 * OFFSET, 1.0, 0.0);
        linkGraphics.addCoordinateSystem(COORD_LENGTH);
        ArrayList<Point2d> polygonPoints = new ArrayList<Point2d>();
        polygonPoints.add(new Point2d());
        polygonPoints.add(new Point2d(0.4, 0.0));
        polygonPoints.add(new Point2d(0.3, 0.3));
        double height = 0.25;
        linkGraphics.addExtrudedPolygon(polygonPoints, height, YoAppearance.Black());

        // Mesh Data
        linkGraphics.translate(OFFSET, 0.0, 0.0);
        linkGraphics.addCoordinateSystem(COORD_LENGTH);
        MeshDataHolder meshData = MeshDataGenerator.Wedge(WEDGE_X, WEDGE_Y, WEDGE_Z);
        AppearanceDefinition meshAppearance = YoAppearance.Black();
        linkGraphics.addMeshData(meshData, meshAppearance );

        ret.setLinkGraphics(linkGraphics);

        return ret;
    }

    public static void main(String[] args)
    {
        new LinkExamplesSimulation();
    }
}
