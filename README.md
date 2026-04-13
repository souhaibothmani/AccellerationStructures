# Acceleration Structures Benchmark

A study and comparison of ray-intersection acceleration structures implemented from scratch in Java, benchmarked against a brute-force baseline on a mixed scene of spheres, axis-aligned boxes, and triangles.

The goal is not to ship a renderer but to **understand** how each structure behaves: how it is built, how it is traversed, and where it wins or loses depending on the scene.

---

## 1. Repository overview

The project is an Android Studio module, but all the relevant logic is plain Java located under:

```
app/src/main/java/com/example/accellerationstructures/
```

### Core primitives

| File | Role |
|------|------|
| `Vec3.java`        | 3D vector with the usual algebra (add, sub, dot, cross, normalize, min/max). |
| `Ray.java`         | Ray = origin + direction. |
| `AABB.java`        | Axis-aligned bounding box. Implements both `Hittable` (slab ray test) and acts as the bounding primitive for every structure. |
| `Intersection.java`| Result of a ray test: `t`, `hitPoint`, `normal`, `objectIndex`. |
| `Hittable.java`    | Interface every scene primitive implements (`intersect`, `getBoundingBox`). |
| `Sphere.java`      | Analytic sphere intersection. Caches its bounding box. |
| `Triangle.java`    | Möller–Trumbore triangle intersection. Caches its bounding box. |

### Acceleration structures

All five structures implement the same interface so the benchmark can treat them uniformly:

```java
public interface AccelerationStructures {
    void build(List<Hittable> objects);
    void insert(Hittable object);
    void delete(Hittable object);
    Intersection findFirstIntersection(Ray ray);
}
```

| File | Strategy |
|------|----------|
| `BruteForce.java`  | Loop over every object, keep the closest hit. The reference baseline. |
| `UniformGrid.java` | Subdivide the scene bounds into a regular `R³` grid. Insert each object into every cell its AABB overlaps. Traverse using 3D-DDA cell marching. |
| `Octree.java`      | Hierarchical 8-way subdivision. Splits a node when it exceeds `MAX_OBJECT`, capped at `MAX_DEPTH = 6`. |
| `KDTree.java`      | Hierarchical 2-way subdivision along the longest axis using the spatial median. |
| `BVH.java`         | Top-down build that recursively splits the object list at the median along the longest axis; each node's AABB shrink-wraps its contents. |

### Benchmark

`app/src/test/java/com/example/accellerationstructures/FinalBenchmarkTest.java` builds a random scene of `N` objects, fires `M` rays, and measures total traversal time per structure. It also verifies correctness by comparing every structure's first hit to the brute-force result.

---

## 2. Running

```bash
./gradlew :app:testDebugUnitTest --tests "com.example.accellerationstructures.FinalBenchmarkTest"
```

The test JVM is configured with a 4 GB heap and G1GC in `app/build.gradle.kts` — the dataset (100k objects × 5 structures held in memory, plus tree allocations) does not fit in the default Gradle test heap.

---

## 3. Scene setup

| Parameter | Value |
|-----------|-------|
| Objects (`N`) | 100 000 |
| Rays (`M`)    | 10 000 |
| Object mix    | 1/3 spheres, 1/3 AABBs, 1/3 triangles, cycled by index |
| Object distribution | uniform random in `[-10, 10] × [-10, 10] × [0, 20]` |
| Ray origins   | uniform random in `[-2, 2] × [-2, 2] × {0}` |
| Ray direction | `(0, 0, 1)` (forward along Z) |
| RNG seeds     | objects = `42`, rays = `123` (deterministic) |

This is a deliberately **uniform** distribution — important context for the results below.

---

## 4. Correctness

All structures return the same first-ray hit as brute force:

```
Intersection(t=0.3113679, hitPoint=Vec3(0.8926966, -1.0510244, 0.3113679), objectIndex=93471)
```

| Structure   | Match |
|-------------|:-----:|
| UniformGrid |  yes  |
| Octree      |  yes  |
| KDTree      |  yes  |
| BVH         |  yes  |

---

## 5. Performance

Latest run (N = 100 000, M = 10 000):

| Structure    | Time (ms) | Speedup vs. BruteForce |
|--------------|----------:|-----------------------:|
| BruteForce   |     8 571 |                   1.0× |
| Octree       |       209 |                  40.9× |
| BVH          |       130 |                  65.6× |
| KDTree       |         8 |                 987.9× |
| **UniformGrid** | **5** |             **1 557.8×** |

**Winner:** `UniformGrid` — 5 ms, ~1 558× faster than brute force.

---

## 6. Why UniformGrid wins (and BVH does not)

In production renderers (Embree, OptiX, PBRT) the BVH is the de-facto standard. So why does it lose here?

### Why the grid dominates this scene
The scene is uniform random. Every grid cell contains roughly the same number of objects, and ray traversal becomes near-`O(1)` per cell with a tight 3D-DDA march. There is no wasted work on empty regions and no deep tree to walk.

### Why BVH underperforms here
This implementation is a *teaching* BVH, missing the optimizations that make production BVHs fast:

1. **No SAH (Surface Area Heuristic).** Splits use the spatial median on the longest axis, not the cost-optimal split. Bounding boxes overlap more, so more rays touch both children.
2. **No nearest-child-first traversal.** Both children are visited every time; a real BVH descends into the closer child first and prunes the farther one once a closer hit is found.
3. **Pointer-chased nodes.** Production BVHs use flat arrays of small structs to keep traversal cache-coherent. Object/node allocations here scatter across the heap.
4. **Uniform scene = no clustering to exploit.** BVHs shine when geometry is *non-uniform* (a detailed mesh next to empty space). Uniform random data is the worst case for hierarchical adaptation.

### Why KDTree is close to the grid
KDTree splits at the spatial median per axis. On uniform data those splits balance cleanly, depth stays small, and traversal is cheap. It does not match the grid only because it still pays log-depth pointer walks.

### Why Octree is the slowest of the accelerated structures
Octree subdivides on object count, not on cost. Many objects lie near cell boundaries and are duplicated into multiple children, inflating leaf occupancy and visiting cost. `MAX_DEPTH` is also clamped (6) to keep memory bounded, which leaves leaves heavier than ideal.

---

## 7. Performance pitfalls fixed during development

Two non-obvious issues were the difference between "all structures look broken" and the table above:

### `objects.indexOf(obj)` in every leaf hit test
Each structure used `List.indexOf` to recover an object's index — an `O(N)` linear scan on a 100 000-element list, called for every primitive intersection in every ray. That single line made every accelerated structure **slower than brute force**. Replaced with an `IdentityHashMap<Hittable, Integer>` populated once at build time.

Before:

```
Octree            585497ms       0.0x
BVH               350847ms       0.0x
UniformGrid        21428ms       0.7x
KDTree             16026ms       0.9x
BruteForce         14439ms       1.0x  ← winner
```

After:

```
UniformGrid            5ms    1557.8x  ← winner
KDTree                 8ms     987.9x
BVH                  130ms      65.6x
Octree               209ms      40.9x
BruteForce          8571ms       1.0x
```

### `Triangle.getBoundingBox()` allocating per call
Building the Octree triggered `OutOfMemoryError` because each overlap test allocated a fresh `AABB` + two `Vec3`s. With hundreds of millions of build-time bbox queries, the allocator collapsed. Fixed by caching the box on `Triangle` (and `Sphere`) on first computation.

### Octree subdivision cascade
The original `subdivide` redistributed objects by recursively re-inserting each one into all 8 children, which immediately re-triggered subdivision when objects straddled split planes. Replaced with direct overlap-tested writes into child leaves.

---

## 8. Takeaways

- An acceleration structure is only as fast as its weakest hot-path operation. A single `O(N)` lookup hidden inside an `O(log N)` traversal silently ruins everything.
- Benchmarks need to match the workload they are trying to characterize. A uniform random scene flatters grids and punishes hierarchical structures — the opposite of what most real 3D content looks like.
- The "best" structure depends entirely on the scene. Production renderers favor BVH not because it always wins benchmarks, but because it degrades gracefully on the kinds of geometry artists actually produce.
