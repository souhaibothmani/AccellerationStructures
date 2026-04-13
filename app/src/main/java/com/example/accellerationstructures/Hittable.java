package com.example.accellerationstructures;

/**
 * Hittable - Interface that every scene object must implement.
 *
 * Any object that can be tested against a ray (Sphere, Triangle, AABB)
 * must be able to:
 *   1. Test if a ray intersects it → intersect()
 *      Returns a complete Intersection object (t, hitPoint, normal, objectIndex)
 *      or null if the ray misses.
 *
 *   2. Provide its bounding box → getBoundingBox()
 *      Returns an AABB that fully wraps the object.
 *      This is used by all acceleration structures (Grid, Octree, kD-Tree, BVH)
 *      to spatially organize objects without knowing their exact shape.
 */
public interface Hittable {

    Intersection intersect(Ray ray , int objectIndex);  //returns the intersection object
                                                        // (so t , hitPoint , normal and object index)

    AABB getBoundingBox();  //returns a box that wraps the object

}
