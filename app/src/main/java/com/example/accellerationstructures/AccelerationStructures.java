package com.example.accellerationstructures;

import java.util.List;

/**
 * AccelerationStructure - Interface that every acceleration structure must implement.
 *
 * Any structure that organizes scene objects for fast ray intersection
 * (BruteForce, Grid, Octree, kD-Tree, BVH) must be able to:
 *
 *   1. build()                → build the structure from a list of objects
 *   2. insert()               → add a single object into the structure
 *   3. delete()               → remove a single object from the structure
 *   4. findFirstIntersection() → given a ray, return the closest hit
 */
public interface AccelerationStructures {

    /**
     * Builds the structure from scratch using a list of objects.
     * This is called once at the start before shooting any rays.
     *
     * @param objects the full list of scene objects (Spheres, Triangles, AABBs)
     */
    void build (List<Hittable> objects);


    /**
     * Inserts a single object into the already-built structure.
     * Used when a new object is added to the scene dynamically.
     *
     * @param object the object to insert
     */
    void insert(Hittable object);

    /**
     * Deletes a single object from the structure.
     * Used when an object is removed from the scene dynamically.
     *
     * @param object the object to remove
     */
    void delete(Hittable object);

    /**
     * Finds the FIRST (closest) intersection of a ray with any object in the scene.
     * This is the core operation — called once per ray per frame.
     *
     * @param ray the ray to test
     * @return the closest Intersection if anything was hit, null if ray hits nothing
     */
    Intersection findFirstIntersection(Ray ray);

    /**
     * BROAD-PHASE PAIR QUERY.
     * Returns every pair of object indices (i, j) whose AABBs overlap
     * according to this structure's spatial logic.
     *
     * Each pair is returned as int[]{i, j} with i < j (canonical form).
     * No duplicates — a given pair appears at most once in the list.
     *
     * The whole point of this method is to show how each structure AVOIDS
     * the O(N^2) pair enumeration that BruteForce has to do:
     *   - UniformGrid: pairs objects within the same cell only
     *   - Octree / KDTree / BVH: prunes whole subtrees when AABBs don't overlap
     *
     * Results MUST be identical across all structures (same pair set),
     * otherwise the implementation has a bug.
     *
     * @return list of overlapping pairs as int[]{i, j} with i < j
     */
    java.util.List<int[]> queryPairs();
}
