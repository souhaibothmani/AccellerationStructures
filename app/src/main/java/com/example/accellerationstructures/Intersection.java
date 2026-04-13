package com.example.accellerationstructures;

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
/*
                FINAL NOTES
so an intersection when created is composed of a t (which is how far along the ray the intersection happens ) ,
 a hitpoint(so 3d point (x y z ) where the ray hits the object (so in a graph kind of with coordinates)) ,
 a objectIndex ( index (so number of identification) of the hitted object ) and then finally the normal
 (so the surface normal of the hitpoint ) . . Intersection construct can be also without a normal
 (for object where the normal doesnt matter .
  then the method is closer than compares the t (so distance along the ray where the intersection happens)
   of when it will be hit and it compares
  which intersection hits first . the tostring methods just print it

 */