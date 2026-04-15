package com.example.accellerationstructures;

import org.junit.Test;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class FinalBenchmarkTest {

    @Test
    public void finalBenchmark() {

        // ─────────────────────────────────────────
        // SCENE SETUP
        // ─────────────────────────────────────────
        int N = 100000;  // number of objects
        int M = 10000;   // number of rays
        List<Hittable> objects = generateRandomObjects(N);
        List<Ray> rays = generateRandomRays(M);

        System.out.println("=== FINAL BENCHMARK ===");
        System.out.println("N = " + N + " objects (mix of Sphere, AABB, Triangle)");
        System.out.println("M = " + M + " rays");
        System.out.println();

        // ─────────────────────────────────────────
        // RUN ALL STRUCTURES
        // ─────────────────────────────────────────

        // --- BRUTEFORCE ---
        BruteForce bruteForce = new BruteForce();
        bruteForce.build(objects);
        long bfTime = benchmark(bruteForce, rays);
        Intersection bfRef = bruteForce.findFirstIntersection(rays.get(0));

        // --- UNIFORM GRID --
        UniformGrid grid = new UniformGrid();
        grid.build(objects);
        long gridTime = benchmark(grid, rays);
        Intersection gridRef = grid.findFirstIntersection(rays.get(0));

        // --- OCTREE ---
        Octree octree = new Octree();
        octree.build(objects);
        long octreeTime = benchmark(octree, rays);
        Intersection octreeRef = octree.findFirstIntersection(rays.get(0));

        // --- KDTREE ---
        KDTree kdTree = new KDTree();
        kdTree.build(objects);
        long kdTime = benchmark(kdTree, rays);
        Intersection kdRef = kdTree.findFirstIntersection(rays.get(0));

        // --- BVH ---
        BVH bvh = new BVH();
        bvh.build(objects);
        long bvhTime = benchmark(bvh, rays);
        Intersection bvhRef = bvh.findFirstIntersection(rays.get(0));

        // ─────────────────────────────────────────
        // CORRECTNESS CHECK
        // ─────────────────────────────────────────
        System.out.println("=== CORRECTNESS (first ray) ===");
        System.out.println("BruteForce:  " + bfRef);
        System.out.println("UniformGrid: " + gridRef  + " " + checkCorrect(bfRef, gridRef));
        System.out.println("Octree:      " + octreeRef + " " + checkCorrect(bfRef, octreeRef));
        System.out.println("KDTree:      " + kdRef     + " " + checkCorrect(bfRef, kdRef));
        System.out.println("BVH:         " + bvhRef    + " " + checkCorrect(bfRef, bvhRef));
        System.out.println();

        // ─────────────────────────────────────────
        // PERFORMANCE TABLE
        // ─────────────────────────────────────────
        System.out.println("=== PERFORMANCE ===");
        System.out.println("──────────────────────────────────────────");
        System.out.printf("%-15s %10s %10s%n", "Structure", "Time(ms)", "Speedup");
        System.out.println("──────────────────────────────────────────");
        printResult("BruteForce",  bfTime,     bfTime);
        printResult("UniformGrid", gridTime,   bfTime);
        printResult("Octree",      octreeTime, bfTime);
        printResult("KDTree",      kdTime,     bfTime);
        printResult("BVH",         bvhTime,    bfTime);
        System.out.println("──────────────────────────────────────────");
        System.out.println();

        // ─────────────────────────────────────────
        // WINNER
        // ─────────────────────────────────────────
        long fastest = Math.min(bfTime,
                Math.min(gridTime,
                        Math.min(octreeTime,
                                Math.min(kdTime, bvhTime))));

        String winner =
                fastest == bfTime     ? "BruteForce"  :
                        fastest == gridTime   ? "UniformGrid" :
                                fastest == octreeTime ? "Octree"      :
                                        fastest == kdTime     ? "KDTree"      : "BVH";

        System.out.println("🏆 WINNER: " + winner +
                " (" + fastest/1_000_000 + "ms, " +
                String.format("%.1fx", (float)bfTime/fastest) + " faster than BruteForce)");
    }

    // ─────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────

    /**
     * Runs M rays through the structure and returns total time in nanoseconds.
     */
    private long benchmark(AccelerationStructures structure, List<Ray> rays) {
        long start = System.nanoTime();
        for (Ray ray : rays) {
            structure.findFirstIntersection(ray);
        }
        return System.nanoTime() - start;
    }

    /**
     * Checks if two intersections are the same (same t and same objectIndex).
     */
    private String checkCorrect(Intersection reference, Intersection result) {
        if (reference == null && result == null) return "✅";
        if (reference == null || result == null) return "❌ (one is null)";
        boolean sameT      = Math.abs(reference.t - result.t) < 0.001f;
        boolean sameObject = reference.objectIndex == result.objectIndex;
        return (sameT && sameObject) ? "✅" : "❌ (different hit)";
    }

    /**
     * Prints one row of the performance table.
     */
    private void printResult(String name, long time, long bfTime) {
        System.out.printf("%-15s %10s %10s%n",
                name,
                time / 1_000_000 + "ms",
                String.format("%.1fx", (float) bfTime / time)
        );
    }

    /**
     * Generates N random objects (mix of Sphere, AABB, Triangle).
     */
    private List<Hittable> generateRandomObjects(int N) {
        List<Hittable> objects = new ArrayList<>();
        Random rand = new Random(42);

        for (int i = 0; i < N; i++) {
            float x = rand.nextFloat() * 20 - 10;
            float y = rand.nextFloat() * 20 - 10;
            float z = rand.nextFloat() * 20;
            int type = i % 3;

            if (type == 0) {
                float r = rand.nextFloat() * 0.5f + 0.1f;
                objects.add(new Sphere(
                        new Vec3(x, y, z), r
                ));
            } else if (type == 1) {
                float size = rand.nextFloat() * 0.5f + 0.1f;
                objects.add(new AABB(
                        new Vec3(x, y, z),
                        new Vec3(x + size, y + size, z + size)
                ));
            } else {
                Vec3 normal = new Vec3(0, 0, -1);
                objects.add(new Triangle(
                        new Vec3(x,        y,        z),
                        new Vec3(x + 0.5f, y,        z),
                        new Vec3(x,        y + 0.5f, z),
                        normal, normal, normal
                ));
            }
        }
        return objects;
    }

    /**
     * Generates M random rays shooting forward along Z axis.
     */
    private List<Ray> generateRandomRays(int M) {
        List<Ray> rays = new ArrayList<>();
        Random rand = new Random(123);

        for (int i = 0; i < M; i++) {
            float x = rand.nextFloat() * 4 - 2;
            float y = rand.nextFloat() * 4 - 2;
            rays.add(new Ray(
                    new Vec3(x, y, 0),
                    new Vec3(0, 0, 1).normalize()
            ));
        }
        return rays;
    }
}