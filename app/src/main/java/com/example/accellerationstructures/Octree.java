package com.example.accellerationstructures;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * Octree - Hierarchical spatial subdivision.
 *
 * Recursively splits space into 8 octants based on local complexity.
 * Dense regions get subdivided more, sparse regions stay as large nodes.
 *
 * Time complexity:
 *   Build:    O(N log N)
 *   Traverse: O(log N) average
 */
public class Octree implements AccelerationStructures {

    // ─────────────────────────────────────────
    // OCTREE NODE
    // ─────────────────────────────────────────
    private static class OctreeNode{

        //bounds that gives the size of the octree
        AABB bounds;

        //sub-nodes inside the octree noded (could be null if it's a leaf)
        OctreeNode[] children;

        //objects stored in this node
        List<Hittable> objects;

        //true if OctreeNode = null
        boolean isLeaf;

        //when initializing we have isLead set as true and children composed of 8 octrees
        OctreeNode(AABB bounds){
            this.bounds = bounds;
            this.objects = new ArrayList<>();
            this.isLeaf = true;
            this.children = new OctreeNode[8];
        }
    }

    // ─────────────────────────────────────────
    // OCTREE FIELDS
    // ─────────────────────────────────────────

    //root node covers the entire scene(first ever box)
    private OctreeNode root;

    //flat list of all objects (for objectIndex lookup)
    private List<Hittable> objects;

    //O(1) index lookup instead of O(N) indexOf
    private Map<Hittable, Integer> indexMap;

    //max objects per leaf before we split
    private static final int MAX_OBJECT = 4;

    //max depth to prevent infinite subdivision
    private static final int MAX_DEPTH = 6; //so octree doesn't have infinite children

    // ─────────────────────────────────────────
    // BUILD
    // ─────────────────────────────────────────
    @Override
    public void build(List<Hittable> objects) {
        this.objects = new ArrayList<>(objects);
        this.indexMap = new IdentityHashMap<>(objects.size());
        for (int i = 0; i < objects.size(); i++) {
            indexMap.put(objects.get(i), i);
        }

        //Step 1 - compute the whole scene bounds(wrap all objects
        AABB sceneBounds = computeSceneBounds(objects);

        //Step 2 - create root node covering entire scene
        root = new OctreeNode((sceneBounds));

        //Step 3 - insert each object into the tree
        for (Hittable obj : objects){
            insertIntoNode(root , obj, 0);
        }

    }

    // ─────────────────────────────────────────
    // INSERT
    // ─────────────────────────────────────────
    @Override
    public void insert(Hittable object) {
        objects.add(object);

        //expand root bounds if object is outside
        AABB box = object.getBoundingBox();
        if(!root.bounds.overlaps(box)){
            //rebuild with expanded bounds
            AABB newBounds = new AABB(
                    root.bounds.min.min(box.min),
                    root.bounds.max.max(box.max)
            );
            root =  new OctreeNode(newBounds);
            for(Hittable obj : objects){
                insertIntoNode(root, obj, 0);
            }
        }else {
            insertIntoNode(root, object, 0);
        }
    }
    /**
     * Recursively inserts an object into the correct node.
     *
     * If leaf and not full → store here
     * If leaf and full → subdivide into 8 children, redistribute
     * If internal → insert into overlapping children
     */
    private void insertIntoNode(OctreeNode node, Hittable object, int depth){

        //if object doesnt overlap this node then skip
        if(!node.bounds.overlaps(object.getBoundingBox())) return;

        //if leaf node
        if (node.isLeaf) {
            //if not full or max depth reached -> store here
            if (node.objects.size() < MAX_OBJECT || depth >= MAX_DEPTH){
                node.objects.add(object);
            } else {
                //subdivide and redistribute
                subdivide(node , depth);
                insertIntoNode(node,object , depth + 1);
            }
        } else {
            //internal node -> insert into all overlapping children
            for (OctreeNode child : node.children) {
                if(child != null){
                    insertIntoNode(child, object , depth + 1);
                }
            }
        }

    }

    /**
     * Splits a leaf node into 8 children octants.
     * Redistributes existing objects into the children.
     */
    private void subdivide(OctreeNode node, int depth){
        Vec3 center = node.bounds.getCenter();
        Vec3 min = node.bounds.min;
        Vec3 max = node.bounds.max;

        //Crete 8 children by splitting at the center
        //each child gets one octant per parent space
        node.children[0] = new OctreeNode(new AABB(min, center));
        node.children[1] = new OctreeNode(new AABB(
                new Vec3(center.x, min.y, min.z),
                new Vec3(max.x, center.y, center.z)));
        node.children[2] = new OctreeNode(new AABB(
                new Vec3(min.x, center.y, min.z),
                new Vec3(center.x, max.y, center.z)));
        node.children[3] = new OctreeNode(new AABB(
                new Vec3(center.x, center.y, min.z),
                new Vec3(max.x, max.y, center.z)));
        node.children[4] = new OctreeNode(new AABB(
                new Vec3(min.x, min.y, center.z),
                new Vec3(center.x, center.y, max.z)));
        node.children[5] = new OctreeNode(new AABB(
                new Vec3(center.x, min.y, center.z),
                new Vec3(max.x, center.y, max.z)));
        node.children[6] = new OctreeNode(new AABB(
                new Vec3(min.x, center.y, center.z),
                new Vec3(center.x, max.y, max.z)));
        node.children[7] = new OctreeNode(new AABB(center, max));

        //mark as internal node
        node.isLeaf= false;

        //redistribute existing objects into children
        List<Hittable> oldObjects = node.objects;
        node.objects = new ArrayList<>();
        for (Hittable obj : oldObjects){
            AABB objBox = obj.getBoundingBox();
            for (OctreeNode child : node.children) {
                if (child.bounds.overlaps(objBox)) {
                    child.objects.add(obj);
                }
            }
        }
    }
    // ─────────────────────────────────────────
    // DELETE
    // ─────────────────────────────────────────
    @Override
    public void delete(Hittable object) {
        objects.remove(object);
        deleteFromNote(root, object);
    }

    /**
     * Recursively removes object from all nodes it lives in.
     */
    private void deleteFromNote(OctreeNode node, Hittable object){
        if (node == null) return;

        //if object doesn't overlap into this node -> skip
        if(!node.bounds.overlaps(object.getBoundingBox())) return;

        if(node.isLeaf){
            node.objects.remove(object);
        }else {
            for(OctreeNode child : node.children){
                deleteFromNote(child,object);
            }
        }
    }

    // ─────────────────────────────────────────
    // TRAVERSE
    // ─────────────────────────────────────────
    @Override
    public Intersection findFirstIntersection(Ray ray) {
        return traverseNode(root, ray);
    }

    /**
     * Recursively traverses the octree.
     *
     * At each node:
     * 1. Test ray against node bounds (AABB slab method)
     * 2. If miss → skip entire subtree (prune)
     * 3. If leaf → test ray against all objects in node
     * 4. If internal → recurse into all children, keep closest hit
     */
    private Intersection traverseNode(OctreeNode node, Ray ray){
        if (node == null) return null;

        //Step 1 - does ray hit this node's bound at all?
        //if not -> prune entire subtree
        float[] ts = node.bounds.intersect(ray);
        if( ts == null) return null;

        Intersection closest = null;

        if (node.isLeaf){
            //Step 2 - leaf node -> test all objects here
            for(Hittable obj : node.objects){
                int index = indexMap.get(obj);
                Intersection hit = obj.intersect(ray, index);
                    if (hit != null && hit.isCloserThan(closest)) {
                        closest = hit;
                    }
                }
            } else {
            // Step 3 - internal node -> recurse into all children
            for(OctreeNode child : node.children){
                Intersection hit = traverseNode(child,ray);
                if (hit != null && hit.isCloserThan(closest)) {
                    closest = hit;
                }
            }
        }
        return closest;

    }

    // ─────────────────────────────────────────
    // HELPER METHODS
    // ─────────────────────────────────────────

    private AABB computeSceneBounds(List<Hittable> objects){
        Vec3 min = new Vec3(Float.MAX_VALUE , Float.MAX_VALUE, Float.MAX_VALUE);
        Vec3 max = new Vec3(Float.MIN_VALUE, Float.MIN_VALUE, Float.MIN_VALUE);

        for (Hittable obj : objects) {
            AABB box = obj.getBoundingBox();
            min = min.min(box.min);
            max = max.max(box.max);
        }

        return new AABB(min, max);

    }
}