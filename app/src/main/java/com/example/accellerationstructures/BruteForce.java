package com.example.accellerationstructures;

import java.util.ArrayList;
import java.util.List;

/**
 * BruteForce - The simplest possible acceleration structure.
 *
 * No spatial organization whatsoever.
 * Every ray is tested against EVERY object in the scene.
 *
 * Time complexity: O(M x N)
 *   M = number of rays
 *   N = number of objects
 *
 * Purpose:
 *   1. Baseline for benchmarking (everything else should beat this)
 *   2. Correctness reference (always finds the true closest hit)
 */
public class BruteForce implements AccelerationStructures{

    private List<Hittable> objects;

    public BruteForce(){
        this.objects = new ArrayList<>();
    }

    /**
     * Build - just stores the list, nothing else to do
     */
    @Override
    public void build(List<Hittable> objects) {
        this.objects = new ArrayList<>(objects);
    }

    /**
     * Insert - just add to the list
     */
    @Override
    public void insert(Hittable object) {
        objects.add(object);
    }

    @Override
    public void delete(Hittable object) {
        objects.remove(object);
    }


    /**
     * findFirstIntersection - tests ray against EVERY object, returns closest hit.
     *
     * This is the core O(N) loop per ray:
     *   - no skipping
     *   - no pruning
     *   - no spatial tricks
     *   - just test everything and keep the closest
     */
    @Override
    public Intersection findFirstIntersection(Ray ray) {

        Intersection closest = null;

        for (int i =0;i < objects.size(); i++ ){

            //test the ray against this object
            Intersection hit = objects.get(i).intersect(ray,i);

            //compare with already closest if not null
            if(hit != null && hit.isCloserThan(closest)){
                //if closer -> save it as closest
                closest = hit;
            }
        }

        //return null if not intersection
        return closest;
    }

    // ─────────────────────────────────────────
    // PAIR QUERY (broad-phase)
    // ─────────────────────────────────────────
    /**
     * Brute-force pair query: tests every possible pair of objects.
     *
     * This is the O(N^2) BASELINE and the CORRECTNESS GROUND TRUTH.
     * Every acceleration structure's queryPairs() must return the same set.
     *
     * Total tests performed: N*(N-1)/2.
     * No pruning, no spatial info used — just nested loops.
     */
    @Override
    public List<int[]> queryPairs() {
        List<int[]> result = new ArrayList<>();
        int n = objects.size();

        // nested loop, j starts at i+1 so:
        //   - we never test (a, a)
        //   - we never test both (a, b) and (b, a)
        //   - pairs are automatically canonical (i < j)
        for (int i = 0; i < n; i++) {
            AABB boxI = objects.get(i).getBoundingBox();
            for (int j = i + 1; j < n; j++) {
                AABB boxJ = objects.get(j).getBoundingBox();
                if (boxI.overlaps(boxJ)) {
                    result.add(new int[]{i, j});
                }
            }
        }
        return result;
    }
}
