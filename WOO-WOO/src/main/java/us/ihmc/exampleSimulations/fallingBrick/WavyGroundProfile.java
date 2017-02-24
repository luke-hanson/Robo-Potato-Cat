package us.ihmc.exampleSimulations.fallingBrick;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.HeightMapWithNormals;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class WavyGroundProfile implements GroundProfile3D, HeightMapWithNormals
{
    private double xMin = -2.0, xMax = 2.0, yMin = -2.0, yMax = 2.0, zMin = -10.0, zMax = 10.0;

    private BoundingBox3d boundingBox = new BoundingBox3d(new Point3d(xMin, yMin, zMin), new Point3d(xMax, yMax, zMax));
    public WavyGroundProfile()
    {
    }
    public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
    {
        double heightAt = heightAt(x, y, z);
        surfaceNormalAt(x, y, heightAt, normalToPack);
        return heightAt;
    }

    public HeightMapWithNormals getHeightMapIfAvailable()
    {
        return this;
    }

    public double heightAt(double x, double y, double z)
    {
        if ((x > xMin) && (x < xMax) && (y > yMin) && (y < yMax))
            return 1.0 * Math.exp(-Math.abs(2.0 * x)) * Math.exp(-Math.abs(2.0 * y)) * Math.sin(2.0 * Math.PI * 0.7 * x);
        else
            return 0.0;
    }
    public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
    {
        normal.x = 0.0;
        normal.y = 0.0;
        normal.z = 1.0;
    }
    public void closestIntersectionTo(double x, double y, double z, Point3d point)
    {
        point.x = x;
        point.y = y;
        point.z = heightAt(x, y, z);
    }
    public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d point, Vector3d normal)
    {
        closestIntersectionTo(x, y, z, point);
        surfaceNormalAt(x, y, z, normal);
    }
    public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
    {
        closestIntersectionTo(x, y, z, intersectionToPack);
        surfaceNormalAt(x, y, z, normalToPack);

        return (z < intersectionToPack.getZ());
    }

    public boolean isClose(double x, double y, double z)
    {
        return boundingBox.isInside(x, y, z);
    }
    public BoundingBox3d getBoundingBox()
    {
        return boundingBox;
    }

}