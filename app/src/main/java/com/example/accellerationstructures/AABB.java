package com.example.accellerationstructures;

/**
 * AABB - Axis-Aligned Bounding Box.
 *
 * A 3D box whose edges are aligned with the X, Y, Z axes (no rotation).
 * Defined by just two corners:
 *   - min: the corner with the smallest x, y, z values
 *   - max: the corner with the largest x, y, z values
 *
 * Think of it like a shoebox sitting straight on a table — no tilting.
 *
 * Uses the SLAB METHOD from slide 3 for ray intersection.
 */
public class AABB implements Hittable{

    // The two opposite corners that define the box
    public Vec3 min; // bottom-left-front corner
    public Vec3 max; // top-right-back corner

    public AABB(Vec3 min, Vec3 max) {
        this.min = min;
        this.max = max;
    }

    /**
     * INTERSECT - Tests if a ray hits this box using the Slab Method (slide 3).
     *
     * The idea is simple:
     * The box is the OVERLAP of 3 "slabs" (one per axis).
     * A slab is just the space between two parallel planes.
     *   - X slab: between x = min.x and x = max.x
     *   - Y slab: between y = min.y and y = max.y
     *   - Z slab: between z = min.z and z = max.z
     *
     * For each axis, we find WHEN (at what t) the ray enters and exits that slab.
     * Then we check if all three intervals overlap.
     *
     * Steps:
     *   1. For each axis: compute tMin and tMax (entry and exit t values)
     *   2. tEnter = max(tXmin, tYmin, tZmin)  → latest entry across all axes
     *   3. tExit  = min(tXmax, tYmax, tZmax)  → earliest exit across all axes
     *   4. If tEnter <= tExit → HIT (the intervals overlap)
     *      If tEnter >  tExit → MISS (the ray passes outside the box)
     *
     * @param ray the ray to test
     * @return float array [tEnter, tExit] if hit, null if miss
     */
    public float[] intersect(Ray ray) {

        // --- X AXIS ---
        // When does the ray reach x = min.x and x = max.x?
        // From P(t) = O + tD, solving for t on the x component:
        //   t = (target_x - O.x) / D.x
        float tXmin = (min.x - ray.origin.x) / ray.direction.x;
        float tXmax = (max.x - ray.origin.x) / ray.direction.x;

        // If ray goes in negative x direction, swap so tXmin < tXmax
        if (tXmin > tXmax) { float temp = tXmin; tXmin = tXmax; tXmax = temp; }

        // --- Y AXIS --- (same logic)
        float tYmin = (min.y - ray.origin.y) / ray.direction.y;
        float tYmax = (max.y - ray.origin.y) / ray.direction.y;

        if (tYmin > tYmax) { float temp = tYmin; tYmin = tYmax; tYmax = temp; }

        // --- Z AXIS --- (same logic)
        float tZmin = (min.z - ray.origin.z) / ray.direction.z;
        float tZmax = (max.z - ray.origin.z) / ray.direction.z;

        if (tZmin > tZmax) { float temp = tZmin; tZmin = tZmax; tZmax = temp; }

        // tEnter = the LATEST entry point (the last axis the ray enters)
        // We need to be inside ALL 3 slabs, so we take the max of the entries.
        float tEnter = Math.max(tXmin, Math.max(tYmin, tZmin));

        // tExit = the EARLIEST exit point (the first axis the ray leaves)
        // Once we leave ANY slab, we're outside the box, so take the min of exits.
        float tExit = Math.min(tXmax, Math.min(tYmax, tZmax));

        // If tEnter > tExit → the intervals DON'T overlap → ray misses the box
        // If tExit < 0 → the box is entirely behind the ray origin
        if (tEnter > tExit || tExit < 0) {
            return null;
        }

        return new float[]{tEnter, tExit};
    }

    @Override
    public String toString() {
        return "AABB(min=" + min + ", max=" + max + ")";
    }

    @Override
    public Intersection intersect(Ray ray, int objectIndex) {
        //Step 1 - reuse existing logic
        float ts[] = intersect(ray);

        //Step 2 - missed entirely
        if(ts == null) return null;

        //Step 3 - pick closest positive t
        // ts[0] = tenter , ts[1] = tExit
        float t;
        if (ts[0] >= 0){
            t = ts[0];  //entry point is in front -> use it
        } else if (ts[1] >=0){
            t = ts[1]; //ray origin is inside the box -> use exit point
        } else {
            return null;
        }

        // Step 4 - compute hitPoint
        Vec3 hitPoint = ray.origin.add(ray.direction.scale(t));

        //Step 5 - compute normal
        Vec3 normal = computeNormal(hitPoint);

        return new Intersection(t,hitPoint,objectIndex,normal);

    }

    /**
     * Figures out which face of the box was hit
     * by checking which face the hitPoint is closest to.
     * Each face has a fixed normal direction:
     *   +X face → normal (1,0,0)
     *   -X face → normal (-1,0,0)
     *   +Y face → normal (0,1,0)  etc.
     */
    private Vec3 computeNormal(Vec3 hitPoint) {
        float epsilon = 0.0001f;
        if (Math.abs(hitPoint.x - min.x) < epsilon) return new Vec3(-1, 0, 0);
        if (Math.abs(hitPoint.x - max.x) < epsilon) return new Vec3(1, 0, 0);
        if (Math.abs(hitPoint.y - min.y) < epsilon) return new Vec3(0, -1, 0);
        if (Math.abs(hitPoint.y - max.y) < epsilon) return new Vec3(0, 1, 0);
        if (Math.abs(hitPoint.z - min.z) < epsilon) return new Vec3(0, 0, -1);
        return new Vec3(0, 0, 1); // +Z face as default
    }

    /**
     * Returns true if this AABB overlaps another AABB.
     * Used by UniformGrid to figure out which cells an object belongs to.
     */
    public boolean overlaps(AABB other) {
        return min.x <= other.max.x && max.x >= other.min.x &&
                min.y <= other.max.y && max.y >= other.min.y &&
                min.z <= other.max.z && max.z >= other.min.z;
    }

    /**
     * Returns the center point of this AABB.
     * Used by Octree to find the split point when subdividing into 8 octants.
     *
     * Example:
     *   min = (0, 0, 0)
     *   max = (6, 6, 6)
     *   center = (3, 3, 3)
     */
    public Vec3 getCenter() {
        return new Vec3(
                (min.x + max.x) / 2,
                (min.y + max.y) / 2,
                (min.z + max.z) / 2
        );
    }


    @Override
    public AABB getBoundingBox() {
        return this;
    }
}
