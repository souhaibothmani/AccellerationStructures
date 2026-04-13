package com.example.accellerationstructures;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * BVH - Bounding Volume Hierarchy.
 *
 * SAME AS KDTREE:
 *   - binary tree (2 children per node)
 *   - recursive subdivision
 *   - leaf nodes store actual objects
 *   - prunes branches when ray misses bounds
 *
 * DIFFERENT FROM KDTREE:
 *   - divides OBJECTS not SPACE
 *   - bounding boxes adapt to object positions (shrink-wrap)
 *   - boxes CAN overlap between siblings
 *   - cannot guarantee early exit like KDTree
 *   - but faster to build and update (no SAH/median needed)
 */

public class BVH implements AccelerationStructures {

    // ─────────────────────────────────────────
    // BVH NODE (inner class)
    // ─────────────────────────────────────────

    private static class BVHNode {

        // [EQUAL TO KDTREE] bounds wraps everything in this node
        // DIFFERENCE: in KDTree bounds = spatial region
        //             in BVH bounds = shrink-wrap around objects
        AABB bounds;

        // [EQUAL TO KDTREE] exactly 2 children
        BVHNode left;
        BVHNode right;

        // [EQUAL TO KDTREE] objects only in leaf nodes
        List<Hittable> objects;

        // [EQUAL TO KDTREE] true if no children
        boolean isLeaf;

        BVHNode(AABB bounds) {
            this.bounds = bounds;
            this.objects = new ArrayList<>();
            this.isLeaf = true;
        }
    }

    // ─────────────────────────────────────────
    // BVH FIELDS
    // ─────────────────────────────────────────

    // [EQUAL TO KDTREE] root covers entire scene
    private BVHNode root;

    // [EQUAL TO KDTREE] flat list for objectIndex lookup
    private List<Hittable> objects;
    private Map<Hittable, Integer> indexMap;

    // [EQUAL TO KDTREE] max objects per leaf
    private static final int MAX_OBJECTS = 4;

    public BVH() {
        this.objects = new ArrayList<>();
    }

    // ─────────────────────────────────────────
    // BUILD
    // ─────────────────────────────────────────

    /**
     * [DIFFERENT FROM KDTREE]
     * KDTree: inserts objects one by one into spatial regions
     * BVH:    builds top-down by repeatedly splitting object list in half
     *         each node's bounds shrink-wraps its objects
     */
    @Override
    public void build(List<Hittable> objects) {
        this.objects = new ArrayList<>(objects);
        this.indexMap = new IdentityHashMap<>(objects.size());
        for (int i = 0; i < objects.size(); i++) {
            indexMap.put(objects.get(i), i);
        }

        // build tree top-down from full object list
        root = buildNode(this.objects, 0);
    }

    /**
     * Recursively builds a BVH node from a list of objects.
     *
     * Step 1: compute bounds that wrap ALL objects in list
     * Step 2: if few enough objects → make leaf
     * Step 3: otherwise → split list in half, recurse
     */
    private BVHNode buildNode(List<Hittable> nodeObjects, int depth) {

        // Step 1 - compute bounds wrapping all objects in this node
        // [DIFFERENT FROM KDTREE] bounds computed FROM objects, not from space
        AABB bounds = computeBounds(nodeObjects);
        BVHNode node = new BVHNode(bounds);

        // Step 2 - few enough objects → make leaf
        if (nodeObjects.size() <= MAX_OBJECTS) {
            node.objects = new ArrayList<>(nodeObjects);
            node.isLeaf = true;
            return node;
        }
        // Step 3 - split object list in half
        // [DIFFERENT FROM KDTREE] we split OBJECTS not SPACE
        // sort by center on longest axis → split at median
        int axis = getLongestAxis(bounds);
        List<Hittable> sorted = sortByAxis(nodeObjects, axis);

        int mid = sorted.size() / 2;
        List<Hittable> leftObjects  = sorted.subList(0, mid);
        List<Hittable> rightObjects = sorted.subList(mid, sorted.size());

        // Step 4 - recurse into children
        node.isLeaf = false;
        node.left  = buildNode(new ArrayList<>(leftObjects),  depth + 1);
        node.right = buildNode(new ArrayList<>(rightObjects), depth + 1);

        return node;
    }

    // ─────────────────────────────────────────
    // INSERT
    // ─────────────────────────────────────────

    /**
     * [DIFFERENT FROM KDTREE]
     * KDTree: inserts into correct spatial region
     * BVH:    rebuilds entire tree (simplest correct approach)
     *         because bounds need to be recomputed bottom-up
     *
     * In production engines this is optimized with "node refitting"
     * but rebuild is correct and simpler for our purposes
     */
    @Override
    public void insert(Hittable object) {
        objects.add(object);
        // rebuild tree with new object included
        root = buildNode(objects, 0);
    }

    // ─────────────────────────────────────────
    // DELETE
    // ─────────────────────────────────────────

    /**
     * [DIFFERENT FROM KDTREE]
     * Same reason as insert → rebuild after removing object
     */
    @Override
    public void delete(Hittable object) {
        objects.remove(object);
        if (!objects.isEmpty()) {
            root = buildNode(objects, 0);
        } else {
            root = null;
        }
    }

    // ─────────────────────────────────────────
    // TRAVERSE
    // ─────────────────────────────────────────

    @Override
    public Intersection findFirstIntersection(Ray ray) {
        if (root == null) return null;
        return traverseNode(root, ray);
    }

    /**
     * [DIFFERENT FROM KDTREE]
     * KDTree: visits Near first → if hit found → skips Far entirely
     *         (safe because partitions don't overlap)
     *
     * BVH:    must check BOTH children if both boxes are hit
     *         (because boxes CAN overlap → Far might have closer hit)
     *         BUT still skips children whose AABB is missed entirely
     */
    private Intersection traverseNode(BVHNode node, Ray ray) {
        if (node == null) return null;

        // [EQUAL TO KDTREE] prune if ray misses this node's bounds
        float[] ts = node.bounds.intersect(ray);
        if (ts == null) return null;

        if (node.isLeaf) {
            // [EQUAL TO KDTREE] test all objects in leaf
            Intersection closest = null;
            for (Hittable obj : node.objects) {
                int index = indexMap.get(obj);
                Intersection hit = obj.intersect(ray, index);
                if (hit != null && hit.isCloserThan(closest)) {
                    closest = hit;
                }
            }
            return closest;
        }

        // [DIFFERENT FROM KDTREE]
        // must check BOTH children and keep closest
        // cannot skip Far even if Near hits
        // because boxes overlap → Far might have closer object
        Intersection leftHit  = traverseNode(node.left,  ray);
        Intersection rightHit = traverseNode(node.right, ray);

        // return whichever hit is closer
        if (leftHit == null)  return rightHit;
        if (rightHit == null) return leftHit;
        return leftHit.isCloserThan(rightHit) ? leftHit : rightHit;
    }

    // ─────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────

    /**
     * [DIFFERENT FROM KDTREE]
     * Computes AABB that shrink-wraps around a list of objects.
     * In KDTree bounds come from spatial split.
     * In BVH bounds come from the objects themselves.
     */
    private AABB computeBounds(List<Hittable> objects) {
        Vec3 min = new Vec3(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
        Vec3 max = new Vec3(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);

        for (Hittable obj : objects) {
            AABB box = obj.getBoundingBox();
            min = min.min(box.min);
            max = max.max(box.max);
        }
        return new AABB(min, max);
    }

    /**
     * [DIFFERENT FROM KDTREE]
     * Finds the longest axis of the bounds.
     * We split on the longest axis to create the most balanced tree.
     *
     * Example:
     *   bounds size = (10, 2, 5)
     *   longest axis = X (size 10)
     *   → split on X
     */
    private int getLongestAxis(AABB bounds) {
        float dx = bounds.max.x - bounds.min.x;
        float dy = bounds.max.y - bounds.min.y;
        float dz = bounds.max.z - bounds.min.z;

        if (dx >= dy && dx >= dz) return 0; // X is longest
        if (dy >= dz)             return 1; // Y is longest
        return 2;                           // Z is longest
    }

    /**
     * [DIFFERENT FROM KDTREE]
     * Sorts objects by their center position on a given axis.
     * Used to split the object list into two balanced halves.
     */
    private List<Hittable> sortByAxis(List<Hittable> objects, int axis) {
        List<Hittable> sorted = new ArrayList<>(objects);
        sorted.sort((a, b) -> {
            float centerA = getAxisValue(a.getBoundingBox().getCenter(), axis);
            float centerB = getAxisValue(b.getBoundingBox().getCenter(), axis);
            return Float.compare(centerA, centerB);
        });
        return sorted;
    }

    /**
     * [EQUAL TO KDTREE] gets x, y or z component based on axis index
     */
    private float getAxisValue(Vec3 v, int axis) {
        if (axis == 0) return v.x;
        if (axis == 1) return v.y;
        return v.z;
    }
}