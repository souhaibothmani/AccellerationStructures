/**
 * Intersection - Represents the result of a ray-object intersection test.
 *
 * When a ray hits an object, we need to know:
 *   - t: how far along the ray the hit occurred
 *   - hitPoint: the 3D coordinate where the hit happened
 *   - objectIndex: which object in the scene was hit (for identifying it)
 *   - normal: the surface normal at the hit point (optional, for lighting)
 *
 * This class is returned by acceleration structures to give full information
 * about which object was hit and where.
 */
public class Intersection {

    // How far along the ray the intersection occurred (t in the ray equation P(t) = O + t*D)
    public float t;

    // The 3D point where the ray hit the object
    public Vec3 hitPoint;

    // Index of the object that was hit (identifies which sphere/AABB/triangle in the scene)
    public int objectIndex;

    // The surface normal at the hit point (optional, useful for shading)
    public Vec3 normal;

    /**
     * Constructor for intersection with all information.
     *
     * @param t            distance along ray
     * @param hitPoint     3D coordinates of the hit
     * @param objectIndex  which object was hit
     * @param normal       surface normal at the hit (can be null)
     */
    public Intersection(float t, Vec3 hitPoint, int objectIndex, Vec3 normal) {
        this.t = t;
        this.hitPoint = hitPoint;
        this.objectIndex = objectIndex;
        this.normal = normal;
    }

    /**
     * Constructor without normal (for objects where normal doesn't matter).
     */
    public Intersection(float t, Vec3 hitPoint, int objectIndex) {
        this(t, hitPoint, objectIndex, null);
    }

    /**
     * Returns true if this intersection is closer (smaller t) than another.
     * Useful for finding the closest hit when multiple intersections occur.
     */
    public boolean isCloserThan(Intersection other) {
        if (other == null) return true;
        return this.t < other.t;
    }

    @Override
    public String toString() {
        return "Intersection(t=" + t + ", hitPoint=" + hitPoint + ", objectIndex=" + objectIndex + ")";
    }
}
