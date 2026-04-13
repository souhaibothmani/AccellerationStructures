/**
 * Vec3 - A 3D vector/point utility class.
 *
 * In 3D graphics, almost everything is described using 3-component vectors:
 * - Positions (points in space): e.g., ray origin, sphere center, triangle vertices
 * - Directions: e.g., ray direction, surface normals
 *
 * This class provides all the vector math operations needed for
 * ray intersection calculations (dot product, cross product, etc.).
 */
public class Vec3 {

    // The three components of the vector
    // In 3D space: x = left/right, y = up/down, z = forward/backward
    public float x, y, z;

    /**
     * Constructor - creates a new 3D vector.
     *
     * Can represent either a POINT (a position in space)
     * or a DIRECTION (like where a ray is going).
     *
     * Example: new Vec3(1, 2, 3) = point at x=1, y=2, z=3
     *          or a direction pointing towards (1, 2, 3)
     */
    public Vec3(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * ADD - Adds two vectors component by component.
     *
     * Geometrically: if you walk along vector A then walk along vector B,
     * the result is where you end up.
     *
     * Example: (1,2,3) + (4,5,6) = (5,7,9)
     *
     * Used in: Ray equation P(t) = O + t*D (origin + scaled direction)
     */
    public Vec3 add(Vec3 other) {
        return new Vec3(x + other.x, y + other.y, z + other.z);
    }

    /**
     * SUBTRACT - Subtracts another vector from this one.
     *
     * Geometrically: gives you the direction/distance FROM other TO this.
     *
     * Example: (5,7,9) - (4,5,6) = (1,2,3)
     *
     * Used in:
     * - Ray-Sphere: L = O - C (vector from sphere center to ray origin)
     * - Möller-Trumbore: E1 = B - A, E2 = C - A (triangle edge vectors)
     */
    public Vec3 subtract(Vec3 other) {
        return new Vec3(x - other.x, y - other.y, z - other.z);
    }

    /**
     * SCALE - Multiplies the vector by a scalar (a single number).
     *
     * Geometrically: stretches or shrinks the vector.
     * - scale(2) = twice as long, same direction
     * - scale(-1) = same length, opposite direction
     * - scale(0.5) = half as long
     *
     * Example: (1,2,3) * 2 = (2,4,6)
     *
     * Used in: Ray equation t*D (scaling the direction by parameter t)
     *          Normal interpolation: w*N0 + u*N1 + v*N2
     */
    public Vec3 scale(float scalar) {
        return new Vec3(x * scalar, y * scalar, z * scalar);
    }

    /**
     * DOT PRODUCT - The most important operation in ray intersection math.
     *
     * Formula: a·b = ax*bx + ay*by + az*bz (multiply matching components, sum them)
     *
     * What it tells you:
     * - Positive: vectors point in roughly the SAME direction
     * - Zero: vectors are PERPENDICULAR (90°)
     * - Negative: vectors point in roughly OPPOSITE directions
     *
     * Also: a·a = length² (dot product of a vector with itself = squared length)
     *
     * Example: (1,0,0)·(0,1,0) = 0 (perpendicular)
     *          (1,0,0)·(1,0,0) = 1 (same direction)
     *
     * Used in:
     * - Ray-Sphere: a = D·D, b = 2*(L·D), c = L·L - R²
     * - Möller-Trumbore: solving for t, u, v
     */
    public float dot(Vec3 other) {
        return x * other.x + y * other.y + z * other.z;
    }

    /**
     * CROSS PRODUCT - Gives a vector PERPENDICULAR to both input vectors.
     *
     * Formula: a × b = (ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx)
     *
     * The result vector is perpendicular to BOTH a and b.
     * Its length equals the area of the parallelogram formed by a and b.
     *
     * Example: (1,0,0) × (0,1,0) = (0,0,1)
     *          X-axis crossed with Y-axis gives Z-axis
     *
     * Used in:
     * - Möller-Trumbore: P = D × E2, Q = T × E1 (intermediate vectors for solving t, u, v)
     * - Computing triangle normals from edges
     */
    public Vec3 cross(Vec3 other) {
        return new Vec3(
                y * other.z - z * other.y,  // new x
                z * other.x - x * other.z,  // new y
                x * other.y - y * other.x   // new z
        );
    }

    /**
     * LENGTH (magnitude) - How long the vector is.
     *
     * Formula: |v| = sqrt(x² + y² + z²)
     *
     * This is just the 3D version of the Pythagorean theorem.
     *
     * Example: |(3,4,0)| = sqrt(9+16+0) = sqrt(25) = 5
     *
     * Used in: checking distances, normalizing vectors
     */
    public float length() {
        return (float) Math.sqrt(x * x + y * y + z * z);
    }

    /**
     * NORMALIZE - Makes the vector length = 1, keeping its direction.
     *
     * Formula: v_normalized = v / |v|
     *
     * A "unit vector" (length 1) is useful because it represents
     * pure direction without any magnitude. Many formulas assume
     * normalized vectors (especially ray directions and normals).
     *
     * Example: (3,0,0) normalized = (1,0,0)
     *          (1,1,0) normalized = (0.707, 0.707, 0)
     *
     * Used in:
     * - Normalizing ray direction before intersection tests
     * - Interpolated surface normal: N_final = normalize(wN0 + uN1 + vN2)
     */
    public Vec3 normalize() {
        float len = length();
        // Avoid dividing by zero if the vector has no length
        if (len == 0) return new Vec3(0, 0, 0);
        return new Vec3(x / len, y / len, z / len);
    }

    /**
     * Returns a readable string representation for debugging.
     * Example output: "Vec3(1.0, 2.0, 3.0)"
     */
    @Override
    public String toString() {
        return "Vec3(" + x + ", " + y + ", " + z + ")";
    }
}
