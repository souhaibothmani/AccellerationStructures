package com.example.accellerationstructures;

import org.junit.Test;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

/**
 * PairBenchmarkTest — BROAD-PHASE pair-intersection benchmark.
 *
 * Goal: compare how fast each acceleration structure can find all pairs of
 * objects whose AABBs overlap. Brute force has to test every pair — O(N^2).
 * The acceleration structures only test pairs that share a spatial region
 * (same cell / same leaf / subtree whose bounds overlap), so they skip huge
 * numbers of pair tests.
 *
 * Metrics reported per structure:
 *   - Build(ms)     build cost once
 *   - Query(ms)     queryPairs() wall-clock time
 *   - #AABBTests    exact count of AABB.overlaps calls (deterministic)
 *   - #Pairs        number of overlapping pairs found
 *   - TimeSpeedup   BruteForce query time / this structure's query time
 *   - TestSpeedup   BruteForce #tests / this structure's #tests
 *   - Correct       does the pair set match BruteForce exactly?
 */
public class PairBenchmarkTest {

    @Test
    public void pairBenchmark() {
        // ─────────────────────────────────────────
        // SCENE SETUP
        // ─────────────────────────────────────────
        // Keep N modest by default — brute force is O(N^2).
        // N=5000 gives ~12.5M pair tests for BruteForce, still manageable.
        int N = 5000;
        List<Hittable> objects = generateRandomObjects(N);

        System.out.println("=== PAIR-INTERSECTION BENCHMARK ===");
        System.out.println("N = " + N + " objects (Sphere / AABB / Triangle mix)");
        System.out.println();

        // ─────────────────────────────────────────
        // RUN EVERY STRUCTURE ON THE SAME SCENE
        // ─────────────────────────────────────────
        Result bf     = runStructure("BruteForce",  new BruteForce(),  objects);
        Result grid   = runStructure("UniformGrid", new UniformGrid(), objects);
        Result octree = runStructure("Octree",      new Octree(),      objects);
        Result kd     = runStructure("KDTree",      new KDTree(),      objects);
        Result bvh    = runStructure("BVH",         new BVH(),         objects);

        // ─────────────────────────────────────────
        // CORRECTNESS CHECK — every structure must match BruteForce exactly
        // ─────────────────────────────────────────
        Set<Long> reference = toSet(bf.pairs);
        grid.correct   = reference.equals(toSet(grid.pairs));
        octree.correct = reference.equals(toSet(octree.pairs));
        kd.correct     = reference.equals(toSet(kd.pairs));
        bvh.correct    = reference.equals(toSet(bvh.pairs));
        bf.correct     = true; // ground truth by definition

        // ─────────────────────────────────────────
        // PERFORMANCE TABLE
        // ─────────────────────────────────────────
        System.out.println("=== RESULTS ===");
        System.out.println("─────────────────────────────────────────────────────────────────────────────────────");
        System.out.printf("%-12s %8s %8s %14s %8s %12s %12s %8s%n",
                "Structure", "Build", "Query", "#AABBTests", "#Pairs",
                "TimeSpeedup", "TestSpeedup", "Correct");
        System.out.println("─────────────────────────────────────────────────────────────────────────────────────");
        printRow(bf,     bf);
        printRow(grid,   bf);
        printRow(octree, bf);
        printRow(kd,     bf);
        printRow(bvh,    bf);
        System.out.println("─────────────────────────────────────────────────────────────────────────────────────");
    }

    // ─────────────────────────────────────────
    // BENCHMARK CORE
    // ─────────────────────────────────────────

    /** Holds the measured metrics for one structure's run. */
    private static class Result {
        String name;
        long buildNs;
        long queryNs;
        long aabbTests;      // AABB.overlapTestCount during queryPairs
        List<int[]> pairs;
        boolean correct;
    }

    /**
     * Build the structure, reset the AABB counter, time queryPairs().
     * Counter is reset AFTER build so we only count test work done by
     * queryPairs itself, not the build (which also calls overlaps).
     */
    private Result runStructure(String name, AccelerationStructures s, List<Hittable> objs) {
        Result r = new Result();
        r.name = name;

        // --- BUILD (not counted in tests) ---
        long t0 = System.nanoTime();
        s.build(objs);
        r.buildNs = System.nanoTime() - t0;

        // reset counter so we measure ONLY the query phase
        AABB.resetOverlapCounter();

        // --- QUERY ---
        long t1 = System.nanoTime();
        r.pairs = s.queryPairs();
        r.queryNs = System.nanoTime() - t1;
        r.aabbTests = AABB.overlapTestCount;

        return r;
    }

    /**
     * Converts a pair list to a Set<Long> for fast equality comparison.
     * Each pair {i, j} becomes the long ((i<<32) | j) with i<j already.
     */
    private Set<Long> toSet(List<int[]> pairs) {
        Set<Long> set = new HashSet<>(pairs.size() * 2);
        for (int[] p : pairs) {
            int lo = Math.min(p[0], p[1]);
            int hi = Math.max(p[0], p[1]);
            set.add(((long) lo << 32) | (hi & 0xffffffffL));
        }
        return set;
    }

    /** One formatted row. BruteForce is the reference for both speedups. */
    private void printRow(Result r, Result bf) {
        double timeSpeedup = (double) bf.queryNs / Math.max(1, r.queryNs);
        double testSpeedup = (double) bf.aabbTests / Math.max(1, r.aabbTests);
        System.out.printf("%-12s %7dms %7dms %14d %8d %11.1fx %11.1fx %8s%n",
                r.name,
                r.buildNs / 1_000_000,
                r.queryNs / 1_000_000,
                r.aabbTests,
                r.pairs.size(),
                timeSpeedup,
                testSpeedup,
                r.correct ? "OK" : "FAIL"
        );
    }

    // ─────────────────────────────────────────
    // SCENE GENERATOR (same seed as FinalBenchmarkTest for consistency)
    // ─────────────────────────────────────────
    /**
     * Random mix of Sphere / AABB / Triangle in a 20x20x20 volume.
     * Small objects so real overlaps exist but are sparse — a realistic
     * broad-phase workload.
     */
    private List<Hittable> generateRandomObjects(int N) {
        List<Hittable> objects = new ArrayList<>();
        Random rand = new Random(42);

        for (int i = 0; i < N; i++) {
            float x = rand.nextFloat() * 20 - 10;
            float y = rand.nextFloat() * 20 - 10;
            float z = rand.nextFloat() * 20 - 10;
            int type = i % 3;

            if (type == 0) {
                float r = rand.nextFloat() * 0.5f + 0.1f;
                objects.add(new Sphere(new Vec3(x, y, z), r));
            } else if (type == 1) {
                float size = rand.nextFloat() * 0.5f + 0.1f;
                objects.add(new AABB(
                        new Vec3(x, y, z),
                        new Vec3(x + size, y + size, z + size)));
            } else {
                Vec3 normal = new Vec3(0, 0, -1);
                objects.add(new Triangle(
                        new Vec3(x,        y,        z),
                        new Vec3(x + 0.5f, y,        z),
                        new Vec3(x,        y + 0.5f, z),
                        normal, normal, normal));
            }
        }
        return objects;
    }
}
