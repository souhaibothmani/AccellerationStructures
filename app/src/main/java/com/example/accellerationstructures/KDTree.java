package com.example.accellerationstructures;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * KDTree - Efficient Spatial Partitioning.
 *
 * SAME AS OCTREE:
 *   - recursive tree structure
 *   - prunes branches when ray misses bounds
 *   - leaf nodes store actual objects
 *   - internal nodes just organize space
 *
 * DIFFERENT FROM OCTREE:
 *   - splits into 2 children only (not 8)
 *   - splits one axis at a time (cycles X → Y → Z)
 *   - split position = median of object centers (adaptive, not always center)
 *   - partitions are STRICTLY DISJOINT → guaranteed early exit in traversal
 */
public class KDTree implements AccelerationStructures {

    // ─────────────────────────────────────────
    // KDTREE NODE (inner class)
    // ─────────────────────────────────────────

    private static class KDTreeNode {

        // [EQUAL TO OCTREE] region of space this node covers
        AABB bounds;

        // [DIFFERENT FROM OCTREE] only 2 children instead of 8
        KDTreeNode left;   // objects on the LEFT/BELOW/BEHIND the split plane
        KDTreeNode right;  // objects on the RIGHT/ABOVE/FRONT of the split plane

        // [EQUAL TO OCTREE] objects stored here (only in leaf nodes)
        List<Hittable> objects;

        // [EQUAL TO OCTREE] true if no children yet
        boolean isLeaf;

        // [DIFFERENT FROM OCTREE] which axis was used to split this node
        // 0 = X axis, 1 = Y axis, 2 = Z axis
        int splitAxis;

        // [DIFFERENT FROM OCTREE] where on that axis the split plane sits
        // everything < splitPos goes LEFT, everything >= splitPos goes RIGHT
        float splitPos;

        KDTreeNode(AABB bounds) {
            this.bounds = bounds;
            this.objects = new ArrayList<>();
            this.isLeaf = true;
            this.splitAxis = 0;
            this.splitPos = 0;
        }
    }

    // ─────────────────────────────────────────
    // KDTREE FIELDS
    // ─────────────────────────────────────────

    // [EQUAL TO OCTREE] root covers entire scene
    private KDTreeNode root;

    // [EQUAL TO OCTREE] flat list for objectIndex lookup
    private List<Hittable> objects;
    private Map<Hittable, Integer> indexMap;

    // [EQUAL TO OCTREE] max objects per leaf before splitting
    private static final int MAX_OBJECTS = 4;

    // [EQUAL TO OCTREE] max depth to prevent infinite subdivision
    private static final int MAX_DEPTH = 20;

    public KDTree() {
        this.objects = new ArrayList<>();
    }

    // ─────────────────────────────────────────
    // BUILD
    // ─────────────────────────────────────────

    // [EQUAL TO OCTREE] same build logic
    @Override
    public void build(List<Hittable> objects) {
        this.objects = new ArrayList<>(objects);
        this.indexMap = new IdentityHashMap<>(objects.size());
        for (int i = 0; i < objects.size(); i++) {
            indexMap.put(objects.get(i), i);
        }

        // [EQUAL TO OCTREE] compute scene bounds
        AABB sceneBounds = computeSceneBounds(objects);

        // [EQUAL TO OCTREE] create root node
        root = new KDTreeNode(sceneBounds);

        // [EQUAL TO OCTREE] insert each object
        for (Hittable obj : objects) {
            insertIntoNode(root, obj, 0);
        }
    }

    // ─────────────────────────────────────────
    // INSERT
    // ─────────────────────────────────────────

    // [EQUAL TO OCTREE] same insert logic
    @Override
    public void insert(Hittable object) {
        objects.add(object);

        AABB box = object.getBoundingBox();
        if (!root.bounds.overlaps(box)) {
            AABB newBounds = new AABB(
                    root.bounds.min.min(box.min),
                    root.bounds.max.max(box.max)
            );
            root = new KDTreeNode(newBounds);
            for (Hittable obj : objects) {
                insertIntoNode(root, obj, 0);
            }
        } else {
            insertIntoNode(root, object, 0);
        }
    }

    /**
     * [DIFFERENT FROM OCTREE]
     * Octree: inserts into ALL overlapping children (up to 8)
     * KDTree: inserts into LEFT, RIGHT, or BOTH children (only 2)
     *         based on which side of the split plane the object is on
     */
    private void insertIntoNode(KDTreeNode node, Hittable object, int depth) {

        // [EQUAL TO OCTREE] skip if object doesn't overlap this node
        if (!node.bounds.overlaps(object.getBoundingBox())) return;

        if (node.isLeaf) {
            // [EQUAL TO OCTREE] not full or max depth → store here
            if (node.objects.size() < MAX_OBJECTS || depth >= MAX_DEPTH) {
                node.objects.add(object);
            } else {
                // [EQUAL TO OCTREE] too full → subdivide then retry
                subdivide(node, depth);
                insertIntoNode(node, object, depth);
            }
        } else {
            // [DIFFERENT FROM OCTREE]
            // only 2 children to check instead of 8
            // check which side of split plane the object is on
            AABB box = object.getBoundingBox();

            // get the object's extent on the split axis
            float objMin = getAxisValue(box.min, node.splitAxis);
            float objMax = getAxisValue(box.max, node.splitAxis);

            // if object overlaps left side → insert left
            if (objMin <= node.splitPos) {
                insertIntoNode(node.left, object, depth + 1);
            }
            // if object overlaps right side → insert right
            // (object can be in BOTH if it straddles the split plane)
            if (objMax >= node.splitPos) {
                insertIntoNode(node.right, object, depth + 1);
            }
        }
    }

    /**
     * [DIFFERENT FROM OCTREE]
     * Octree:  splits into 8 children at the CENTER point
     * KDTree:  splits into 2 children at the MEDIAN of object centers
     *          cycling through X → Y → Z axes
     */
    private void subdivide(KDTreeNode node, int depth) {

        // [DIFFERENT FROM OCTREE] cycle through axes: depth 0=X, 1=Y, 2=Z, 3=X...
        node.splitAxis = depth % 3;

        // [DIFFERENT FROM OCTREE] split position = median of object centers on this axis
        // median ensures roughly equal objects on each side
        node.splitPos = computeMedian(node.objects, node.splitAxis);

        // [DIFFERENT FROM OCTREE] create only 2 children instead of 8
        // left child: region from node min to split plane
        // right child: region from split plane to node max
        Vec3 leftMax = new Vec3(node.bounds.max.x, node.bounds.max.y, node.bounds.max.z);
        Vec3 rightMin = new Vec3(node.bounds.min.x, node.bounds.min.y, node.bounds.min.z);

        // set the split axis boundary
        setAxisValue(leftMax, node.splitAxis, node.splitPos);
        setAxisValue(rightMin, node.splitAxis, node.splitPos);

        node.left  = new KDTreeNode(new AABB(node.bounds.min, leftMax));
        node.right = new KDTreeNode(new AABB(rightMin, node.bounds.max));

        // [EQUAL TO OCTREE] mark as internal node
        node.isLeaf = false;

        // [EQUAL TO OCTREE] redistribute existing objects into children
        List<Hittable> oldObjects = node.objects;
        node.objects = new ArrayList<>();

        for (Hittable obj : oldObjects) {
            AABB box = obj.getBoundingBox();
            float objMin = getAxisValue(box.min, node.splitAxis);
            float objMax = getAxisValue(box.max, node.splitAxis);

            if (objMin <= node.splitPos) insertIntoNode(node.left,  obj, depth + 1);
            if (objMax >= node.splitPos) insertIntoNode(node.right, obj, depth + 1);
        }
    }

    // ─────────────────────────────────────────
    // DELETE
    // ─────────────────────────────────────────

    // [EQUAL TO OCTREE] same delete logic
    @Override
    public void delete(Hittable object) {
        objects.remove(object);
        deleteFromNode(root, object);
    }

    // [EQUAL TO OCTREE] same recursive delete
    private void deleteFromNode(KDTreeNode node, Hittable object) {
        if (node == null) return;
        if (!node.bounds.overlaps(object.getBoundingBox())) return;

        if (node.isLeaf) {
            // [EQUAL TO OCTREE] remove from leaf
            node.objects.remove(object);
        } else {
            // [DIFFERENT FROM OCTREE] only 2 children to recurse into
            deleteFromNode(node.left,  object);
            deleteFromNode(node.right, object);
        }
    }

    // ─────────────────────────────────────────
    // TRAVERSE
    // ─────────────────────────────────────────

    // [EQUAL TO OCTREE] entry point
    @Override
    public Intersection findFirstIntersection(Ray ray) {
        return traverseNode(root, ray);
    }

    /**
     * [DIFFERENT FROM OCTREE]
     * Octree:  recurses into ALL children, keeps closest
     * KDTree:  visits NEAR child first, if hit found → skip FAR child entirely
     *          this works because partitions are STRICTLY DISJOINT
     *          → guaranteed early exit, no redundant checks
     */
    private Intersection traverseNode(KDTreeNode node, Ray ray) {
        if (node == null) return null;

        // [EQUAL TO OCTREE] prune if ray misses this node's bounds
        float[] ts = node.bounds.intersect(ray);
        if (ts == null) return null;

        if (node.isLeaf) {
            // [EQUAL TO OCTREE] test all objects in leaf
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

        // [DIFFERENT FROM OCTREE] determine Near and Far child
        // based on ray direction on split axis
        KDTreeNode near;
        KDTreeNode far;

        // if ray goes positive on split axis → left is Near, right is Far
        // if ray goes negative on split axis → right is Near, left is Far
        float rayDir = getAxisValue(ray.direction, node.splitAxis);
        if (rayDir >= 0) {
            near = node.left;
            far  = node.right;
        } else {
            near = node.right;
            far  = node.left;
        }

        // [DIFFERENT FROM OCTREE] visit Near first
        Intersection nearHit = traverseNode(near, ray);

        // [DIFFERENT FROM OCTREE] EARLY EXIT
        // if Near found a hit → skip Far entirely
        // safe because partitions don't overlap!
        if (nearHit != null) return nearHit;

        // Near missed → check Far
        return traverseNode(far, ray);
    }

    // ─────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────

    // [EQUAL TO OCTREE] same scene bounds computation
    private AABB computeSceneBounds(List<Hittable> objects) {
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
     * [DIFFERENT FROM OCTREE]
     * Computes the median center position of all objects on a given axis.
     * This gives a balanced split — roughly equal objects on each side.
     */
    private float computeMedian(List<Hittable> objects, int axis) {
        float sum = 0;
        for (Hittable obj : objects) {
            sum += getAxisValue(obj.getBoundingBox().getCenter(), axis);
        }
        return sum / objects.size();
    }

    /**
     * [DIFFERENT FROM OCTREE]
     * Gets the x, y or z component of a Vec3 based on axis index.
     * 0 = x, 1 = y, 2 = z
     */
    private float getAxisValue(Vec3 v, int axis) {
        if (axis == 0) return v.x;
        if (axis == 1) return v.y;
        return v.z;
    }

    /**
     * [DIFFERENT FROM OCTREE]
     * Sets the x, y or z component of a Vec3 based on axis index.
     * Used to construct left/right child bounds.
     */
    private void setAxisValue(Vec3 v, int axis, float value) {
        if (axis == 0) v.x = value;
        else if (axis == 1) v.y = value;
        else v.z = value;
    }
}