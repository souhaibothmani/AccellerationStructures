/**
 * Ray - Represents a ray in 3D space.
 *
 * A ray is like a laser beam: it starts at a point and goes in a direction forever.
 *
 * Defined by the equation from the slides:
 *     P(t) = O + t * D
 *
 * Where:
 *   - O = origin (where the ray starts)
 *   - D = direction (which way it's pointing)
 *   - t = parameter (how far along the ray). t=0 is the origin, t=1 is one "D-length" away
 *   - P(t) = the resulting point on the ray at distance t
 *
 * Example:
 *   Origin = (0, 0, 0), Direction = (1, 0, 0)
 *   t=0  → P = (0,0,0)  (at the origin)
 *   t=1  → P = (1,0,0)  (1 unit to the right)
 *   t=5  → P = (5,0,0)  (5 units to the right)
 *   t=-1 → P = (-1,0,0) (behind the origin — usually we ignore t < 0)
 */
public class Ray {

    // Where the ray starts
    public Vec3 origin;

    // Which direction the ray travels (should be normalized for correct distance calculations)
    public Vec3 direction;

    /**
     * Constructor - creates a ray from an origin point and direction.
     *
     * The direction is normalized (made length = 1) so that the t parameter
     * directly corresponds to actual distance along the ray.
     *
     * @param origin    the starting point of the ray
     * @param direction which way the ray points (will be normalized)
     */
    public Ray(Vec3 origin, Vec3 direction) {
        this.origin = origin;
        this.direction = direction.normalize();
    }

    /**
     * POINT AT - Computes the point on the ray at parameter t.
     *
     * This is the ray equation from the slides: P(t) = O + t * D
     *
     * Steps:
     *   1. Scale the direction by t  →  t * D
     *   2. Add it to the origin      →  O + (t * D)
     *
     * When we find an intersection (e.g., ray hits a sphere at t=3.5),
     * we call pointAt(3.5) to get the actual 3D coordinates of the hit.
     *
     * @param t how far along the ray (t >= 0 means in front of origin)
     * @return the 3D point at that position on the ray
     */
    public Vec3 pointAt(float t) {
        // O + t * D
        return origin.add(direction.scale(t));
    }

    @Override
    public String toString() {
        return "Ray(origin=" + origin + ", direction=" + direction + ")";
    }
}
