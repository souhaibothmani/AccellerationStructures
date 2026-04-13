package com.example.accellerationstructures;

/**
 * Sphere - Represents a sphere in 3D space and tests ray intersection.
 *
 * A sphere is defined by:
 *   - Center (C): a point in 3D space
 *   - Radius (R): how big the sphere is
 *
 * The sphere equation from slide 2:
 *   |P - C|² = R²
 *   "Any point P on the sphere surface is exactly R distance from center C"
 */
public class Sphere implements Hittable{

    // The center point of the sphere
    public Vec3 center;

    // The radius (distance from center to surface)
    public float radius;

    public Sphere(Vec3 center, float radius) {
        this.center = center;
        this.radius = radius;
    }

    /**
     * INTERSECT - Tests if a ray hits this sphere and returns the t values.
     *
     * This implements the quadratic formula from slide 2:
     *
     * Step 1: Substitute ray equation P(t) = O + tD into sphere equation |P - C|² = R²
     *         → |(O + tD) - C|² = R²
     *
     * Step 2: Rearrange into quadratic form: at² + bt + c = 0
     *         where:
     *           L = O - C          (vector from sphere center to ray origin)
     *           a = D · D          (always positive, = 1 if direction is normalized)
     *           b = 2 * (L · D)    (how aligned the ray is with the center-to-origin vector)
     *           c = L · L - R²     (squared distance from origin to center, minus radius²)
     *
     * Step 3: Solve with quadratic formula: t = (-b ± √(b² - 4ac)) / 2a
     *
     * The DISCRIMINANT (Δ = b² - 4ac) tells us how many intersections:
     *   - Δ < 0 → NO intersection (ray misses the sphere entirely)
     *   - Δ = 0 → ONE intersection (ray grazes the sphere tangentially)
     *   - Δ > 0 → TWO intersections (ray enters and exits the sphere)
     *                t1 = entry point, t2 = exit point
     *
     * @param ray the ray to test against this sphere
     * @return float array with t values [t1, t2] if hit, or null if no intersection.
     *         t1 is always the closer (entry) point, t2 the farther (exit) point.
     *         If t1 < 0, the intersection is behind the ray origin.
     */
    public float[] intersect(Ray ray) {

        // L = O - C
        // Vector from the sphere center to the ray origin.
        // This tells us where the ray starts relative to the sphere.
        Vec3 L = ray.origin.subtract(center);

        // a = D · D
        // Dot product of direction with itself = squared length of direction.
        // If direction is normalized (length 1), this will be 1.
        float a = ray.direction.dot(ray.direction);

        // b = 2 * (L · D)
        // This measures how much the ray is pointing towards/away from the sphere.
        // If b is negative, the ray points towards the sphere center.
        float b = 2.0f * L.dot(ray.direction);

        // c = L · L - R²
        // L·L = squared distance from ray origin to sphere center.
        // Subtract R² to check if origin is inside (c < 0) or outside (c > 0) the sphere.
        float c = L.dot(L) - radius * radius;

        // Δ = b² - 4ac (the discriminant)
        // This is the value under the square root in the quadratic formula.
        // It determines if solutions exist.
        float discriminant = b * b - 4 * a * c;

        // Δ < 0: No real solutions → ray misses the sphere completely
        if (discriminant < 0) {
            return null;
        }

        // √Δ - needed for both solutions
        float sqrtDiscriminant = (float) Math.sqrt(discriminant);

        // t1 = (-b - √Δ) / 2a  → the CLOSER intersection (entry point)
        float t1 = (-b - sqrtDiscriminant) / (2 * a);

        // t2 = (-b + √Δ) / 2a  → the FARTHER intersection (exit point)
        float t2 = (-b + sqrtDiscriminant) / (2 * a);

        // Return both t values.
        // To get the actual 3D point: ray.pointAt(t1) or ray.pointAt(t2)
        return new float[]{t1, t2};
    }

    @Override
    public String toString() {
        return "Sphere(center=" + center + ", radius=" + radius + ")";
    }

    @Override
    public Intersection intersect(Ray ray, int objectIndex) {
        //step1 - reuse existing intersect methods
        float[] ts = intersect(ray);

        //step 2 - missed entirely
        if (ts == null) return null;

        //step 3 - pick closest positive t
        //t1 is entry , t2 is exit
        //we want the smallest t that is still in front of the ray( t >=0 )
        float t;
        if (ts[0] >=0 ){
            t = ts[0];  //entry point is in front -> use this
        } else if (ts[1] >= 1){
            t = ts[1]; //entry point is behind -> use exit point
        } else {
            return null; //both behind ray origin -> miss
        }

        //step 4 - compute hitPoint from t
        Vec3 hitPoint = ray.origin.add(ray.direction.scale(t));

        //step 5 - normal points outward from center to hitPoint
        Vec3 normal = hitPoint.subtract(center).normalize();

        //step 6 - return complete Intersection
        //objectIndex comes form parameter of the method
        return new Intersection(t, hitPoint, objectIndex , normal);
    }

    private AABB cachedBox;

    @Override
    public AABB getBoundingBox() {
        if (cachedBox != null) return cachedBox;
        //min corner: go radius distance LEFT , DOWN , BACK from center
        Vec3 min = new Vec3(
                center.x - radius, //leftmost point
                center.y - radius,  //bottommost point
                center.z - radius   //backmost point
        );

        //max cornerL go radius distance RIGHT, UP , FORWARD from center
        Vec3 max= new Vec3(
                center.x + radius, //rightmost point
                center.y + radius, //upmost point
                center.z + radius //frontmost point
        );

        cachedBox = new AABB(min,max);
        return cachedBox;
    }
}
