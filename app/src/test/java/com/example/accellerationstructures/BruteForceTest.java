package com.example.accellerationstructures;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * Unit tests for BruteForce.
 *
 * BruteForce has no spatial tricks: it tests every ray against every object and
 * keeps the closest hit. Because of that simplicity, it is the OACLE we trust to
 * be correct — every other structure (Grid, Octree, KDTree, BVH) is validated
 * against it. So these tests exist to prove the oracle itself is trustworthy.
 *
 * What we check here:
 *   1. Correctness of findFirstIntersection (hit, miss, closest-of-many, order
 *      independence, mixed primitive types).
 *   2. API behavior (build / insert / delete / re-build).
 *   3. Performance shape — the O(N) per ray claim from the README.
 */
public class BruteForceTest {

    // Floating point tolerance for comparing t values.
    // Sphere/Triangle intersections use floats so we can't expect bit-exact equality.
    private static final float EPS = 1e-4f;

    // -------- small helpers to keep each test readable --------

    /** Sphere centered at (x,y,z) with radius r. */
    private static Sphere sphereAt(float x, float y, float z, float r) {
        return new Sphere(new Vec3(x, y, z), r);
    }

    /**
     * A ray that starts on the z=0 plane at (x,y,0) and shoots straight forward
     * along +Z. This matches the benchmark's ray setup, and makes expected t
     * values trivial to reason about (t == distance traveled in Z).
     */
    private static Ray rayForwardZ(float x, float y) {
        return new Ray(new Vec3(x, y, 0), new Vec3(0, 0, 1));
    }

    // ─────────────────────────────────────────────────────────────────────
    // CORRECTNESS TESTS
    // ─────────────────────────────────────────────────────────────────────

    @Test
    public void emptySceneReturnsNull() {
        // No objects at all → nothing to hit → must return null (not crash).
        BruteForce bf = new BruteForce();
        bf.build(new ArrayList<>());
        assertNull(bf.findFirstIntersection(rayForwardZ(0, 0)));
    }

    @Test
    public void singleHitReturnsThatObject() {
        // Sphere centered at z=5 with radius 1 → front surface is at z=4.
        // Ray starts at origin shooting +Z → expected t = 4.
        BruteForce bf = new BruteForce();
        bf.build(Collections.singletonList(sphereAt(0, 0, 5, 1)));

        Intersection hit = bf.findFirstIntersection(rayForwardZ(0, 0));
        assertNotNull(hit);                       // we must get a hit back
        assertEquals(0, hit.objectIndex);         // index 0 = the only sphere
        assertEquals(4.0f, hit.t, EPS);           // entry distance = 4
    }

    @Test
    public void missReturnsNull() {
        // Sphere is far off-axis, ray goes straight forward → cannot hit.
        BruteForce bf = new BruteForce();
        bf.build(Collections.singletonList(sphereAt(100, 100, 5, 1)));
        assertNull(bf.findFirstIntersection(rayForwardZ(0, 0)));
    }

    @Test
    public void returnsClosestHitNotFirstInList() {
        // Two spheres on the same ray path. The FAR one is listed first.
        // BruteForce must return the NEAR one regardless of list order —
        // this is the whole point of "first intersection" (smallest t).
        Sphere far  = sphereAt(0, 0, 20, 1);
        Sphere near = sphereAt(0, 0, 5,  1);

        BruteForce bf = new BruteForce();
        bf.build(Arrays.asList(far, near));        // near is at index 1

        Intersection hit = bf.findFirstIntersection(rayForwardZ(0, 0));
        assertNotNull(hit);
        assertEquals("closest must be the near sphere", 1, hit.objectIndex);
        assertEquals(4.0f, hit.t, EPS);
    }

    @Test
    public void resultIndependentOfListOrder() {
        // Same two spheres, two different orderings → t must match.
        // If this fails, BruteForce is leaking list-order into its result.
        Sphere a = sphereAt(0, 0, 5,  1);
        Sphere b = sphereAt(0, 0, 20, 1);

        BruteForce bf1 = new BruteForce();
        bf1.build(Arrays.asList(a, b));
        BruteForce bf2 = new BruteForce();
        bf2.build(Arrays.asList(b, a));

        Intersection h1 = bf1.findFirstIntersection(rayForwardZ(0, 0));
        Intersection h2 = bf2.findFirstIntersection(rayForwardZ(0, 0));

        assertEquals(h1.t, h2.t, EPS);
    }

    @Test
    public void mixedPrimitivesAllReachable() {
        // Three different Hittable types stacked along +Z so we know the
        // expected closest is the triangle (z=3), then box (z=8), then sphere (z=15).
        // This proves the loop in BruteForce treats every Hittable uniformly.
        Vec3 n = new Vec3(0, 0, -1); // triangle vertex normal (faces the ray)
        Triangle tri = new Triangle(
                new Vec3(-1, -1, 3), new Vec3(1, -1, 3), new Vec3(0, 1, 3),
                n, n, n);
        AABB box = new AABB(new Vec3(-1, -1, 8), new Vec3(1, 1, 9));
        Sphere sph = sphereAt(0, 0, 15, 1);

        BruteForce bf = new BruteForce();
        bf.build(Arrays.asList(tri, box, sph));

        Intersection hit = bf.findFirstIntersection(rayForwardZ(0, 0));
        assertNotNull(hit);
        assertEquals("triangle is closest", 0, hit.objectIndex);
        assertTrue(hit.t < 4.0f);                  // triangle sits at z≈3
    }

    // ─────────────────────────────────────────────────────────────────────
    // API TESTS — build / insert / delete contract
    // ─────────────────────────────────────────────────────────────────────

    @Test
    public void insertMakesObjectHittable() {
        // Start empty, insert a sphere on the ray path → must now be hittable.
        BruteForce bf = new BruteForce();
        bf.build(new ArrayList<>());
        assertNull(bf.findFirstIntersection(rayForwardZ(0, 0)));

        bf.insert(sphereAt(0, 0, 5, 1));
        assertNotNull(bf.findFirstIntersection(rayForwardZ(0, 0)));
    }

    @Test
    public void deleteRemovesObject() {
        // After delete, the sphere must no longer be considered.
        Sphere s = sphereAt(0, 0, 5, 1);
        BruteForce bf = new BruteForce();
        bf.build(Collections.singletonList(s));
        assertNotNull(bf.findFirstIntersection(rayForwardZ(0, 0)));

        bf.delete(s);
        assertNull(bf.findFirstIntersection(rayForwardZ(0, 0)));
    }

    @Test
    public void buildReplacesPreviousScene() {
        // Important contract: calling build() twice must REPLACE the scene,
        // not append. If it appended, the first sphere would still be hit.
        BruteForce bf = new BruteForce();
        bf.build(Collections.singletonList(sphereAt(0, 0, 5, 1)));   // hittable
        bf.build(Collections.singletonList(sphereAt(100, 100, 5, 1))); // off-axis

        assertNull("second build must replace, not append",
                bf.findFirstIntersection(rayForwardZ(0, 0)));
    }

    // ─────────────────────────────────────────────────────────────────────
    // PERFORMANCE SHAPE — confirms the O(N) per-ray claim from the README
    // ─────────────────────────────────────────────────────────────────────

    @Test
    public void scalesLinearlyWithN() {
        // We don't measure exact ns/op (JVM jitter makes that flaky) — we just
        // confirm the SHAPE: 10× more objects should take meaningfully longer.
        // Threshold of 3× is loose on purpose so this isn't a flaky test.
        List<Ray> rays = new ArrayList<>();
        for (int i = 0; i < 200; i++) rays.add(rayForwardZ(0, 0));

        long small = timeTraversal(buildWith(1_000),  rays);
        long large = timeTraversal(buildWith(10_000), rays);

        assertTrue("10x objects should take clearly longer (O(N) per ray): "
                + small + "ns vs " + large + "ns", large > small * 3);
    }

    /**
     * Build a BruteForce scene of N tiny spheres clustered far off-axis so the
     * test rays MISS every one. We want the worst case: BruteForce can't early-
     * out on a hit, so it really does test all N objects per ray.
     */
    private static BruteForce buildWith(int n) {
        List<Hittable> objs = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            objs.add(sphereAt(50 + i * 0.001f, 50, 5, 0.1f));
        }
        BruteForce bf = new BruteForce();
        bf.build(objs);
        return bf;
    }

    /**
     * Time one full pass of all rays. The 3 warmup passes let the JIT compile
     * the hot path before we start the clock — without warmup the first run is
     * dominated by interpreter/compile overhead, not the actual algorithm.
     */
    private static long timeTraversal(BruteForce bf, List<Ray> rays) {
        for (int i = 0; i < 3; i++) for (Ray r : rays) bf.findFirstIntersection(r);
        long start = System.nanoTime();
        for (Ray r : rays) bf.findFirstIntersection(r);
        return System.nanoTime() - start;
    }
}
