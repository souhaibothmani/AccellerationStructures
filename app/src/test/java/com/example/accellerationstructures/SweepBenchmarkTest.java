package com.example.accellerationstructures;

import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * SweepBenchmarkTest — runs every acceleration structure across a grid of
 * scene sizes (N) and ray counts (M), and writes the results to a CSV file
 * that can be plotted (see scripts/plot_benchmarks.py).
 *
 * Output: build/benchmarks/sweep.csv
 *   columns: N,M,structure,build_ms,query_ms,total_ms
 *
 * Run:
 *   ./gradlew :app:testDebugUnitTest --tests \
 *     "com.example.accellerationstructures.SweepBenchmarkTest"
 */
public class SweepBenchmarkTest {

    // Sweep grid — tweak these to widen / narrow the experiment.
    // Keep totals reasonable: every (N, M) pair builds 5 structures.
    private static final int[] N_VALUES = { 1_000, 5_000, 10_000, 50_000, 100_000 };
    private static final int[] M_VALUES = { 100, 1_000, 5_000, 10_000, 50_000 };

    @Test
    public void sweep() throws IOException {
        File outDir = new File("build/benchmarks");
        outDir.mkdirs();
        File outFile = new File(outDir, "sweep.csv");

        try (PrintWriter csv = new PrintWriter(new FileWriter(outFile))) {
            csv.println("N,M,structure,build_ms,query_ms,total_ms");

            for (int N : N_VALUES) {
                List<Hittable> objects = generateRandomObjects(N);

                for (int M : M_VALUES) {
                    List<Ray> rays = generateRandomRays(M);
                    System.out.printf("=== N=%d  M=%d ===%n", N, M);

                    // Each entry: name -> factory. Built fresh per (N, M).
                    Map<String, AccelerationStructures> structures = new LinkedHashMap<>();
                    structures.put("BruteForce",  new BruteForce());
                    structures.put("UniformGrid", new UniformGrid());
                    structures.put("Octree",      new Octree());
                    structures.put("KDTree",      new KDTree());
                    structures.put("BVH",         new BVH());

                    for (Map.Entry<String, AccelerationStructures> e : structures.entrySet()) {
                        String name = e.getKey();
                        AccelerationStructures s = e.getValue();

                        // Build time — measured separately so the chart can show
                        // build vs. query trade-offs (e.g. BVH builds slower but
                        // queries faster).
                        long t0 = System.nanoTime();
                        s.build(objects);
                        long buildNs = System.nanoTime() - t0;

                        // Warmup so the JIT compiles the hot path before timing.
                        // Without this the first structure in the loop looks slow.
                        for (int i = 0; i < Math.min(rays.size(), 200); i++) {
                            s.findFirstIntersection(rays.get(i));
                        }

                        long t1 = System.nanoTime();
                        for (Ray r : rays) s.findFirstIntersection(r);
                        long queryNs = System.nanoTime() - t1;

                        double buildMs = buildNs / 1_000_000.0;
                        double queryMs = queryNs / 1_000_000.0;
                        csv.printf("%d,%d,%s,%.3f,%.3f,%.3f%n",
                                N, M, name, buildMs, queryMs, buildMs + queryMs);

                        System.out.printf("  %-12s build=%8.1fms  query=%8.1fms%n",
                                name, buildMs, queryMs);
                    }
                }
            }
        }

        System.out.println();
        System.out.println("CSV written to: " + outFile.getAbsolutePath());
        System.out.println("Plot with:    python3 scripts/plot_benchmarks.py");
    }

    // ───────── scene generators (same RNG seeds as FinalBenchmarkTest so
    // results are comparable across the two test files) ─────────

    private List<Hittable> generateRandomObjects(int N) {
        List<Hittable> objects = new ArrayList<>(N);
        Random rand = new Random(42);
        for (int i = 0; i < N; i++) {
            float x = rand.nextFloat() * 20 - 10;
            float y = rand.nextFloat() * 20 - 10;
            float z = rand.nextFloat() * 20;
            int type = i % 3;
            if (type == 0) {
                objects.add(new Sphere(new Vec3(x, y, z), rand.nextFloat() * 0.5f + 0.1f));
            } else if (type == 1) {
                float sz = rand.nextFloat() * 0.5f + 0.1f;
                objects.add(new AABB(new Vec3(x, y, z), new Vec3(x + sz, y + sz, z + sz)));
            } else {
                Vec3 n = new Vec3(0, 0, -1);
                objects.add(new Triangle(
                        new Vec3(x, y, z),
                        new Vec3(x + 0.5f, y, z),
                        new Vec3(x, y + 0.5f, z),
                        n, n, n));
            }
        }
        return objects;
    }

    private List<Ray> generateRandomRays(int M) {
        List<Ray> rays = new ArrayList<>(M);
        Random rand = new Random(123);
        for (int i = 0; i < M; i++) {
            float x = rand.nextFloat() * 4 - 2;
            float y = rand.nextFloat() * 4 - 2;
            rays.add(new Ray(new Vec3(x, y, 0), new Vec3(0, 0, 1)));
        }
        return rays;
    }
}
