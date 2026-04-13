package com.example.accellerationstructures;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * UniformGrid - Divides the scene into equal-sized 3D cells (voxels).
 *
 * Build:    wrap entire scene in one big AABB, divide into equal cells
 * Insert:   find which cells the object overlaps, add it there
 * Delete:   remove object from all cells it was in
 * Traverse: step cell by cell along ray (3D-DDA), test objects in each cell
 *
 * Time complexity:
 *   Build:    O(N)
 *   Traverse: O(√N) average
 */
public class UniformGrid implements AccelerationStructures{


    // The grid is a 3D array of cells
    // Each cell contains a list of objects that overlap it
    private List<Hittable>[][][] cells;

    //Bounding box of entire scene
    private AABB sceneBounds;

    //How many cells per axis (same for X , Y , Z)
    private int resolution;

    //Facade of the boxes
    private Vec3 cellSize;

    //List of objects in the scene
    private List<Hittable> objects;
    private Map<Hittable, Integer> indexMap;

    public UniformGrid(){
        this.objects = new ArrayList<>();
    }

    // ─────────────────────────────────────────
    // BUILD
    // ─────────────────────────────────────────

    /**
     * Builds the grid from scratch.
     *
     * Step 1: find scene bounding box (wrap ALL objects)
     * Step 2: decide resolution (cube root of N)
     * Step 3: compute cell size
     * Step 4: insert each object into the cells it overlaps
     */
    @Override
    public void build(List<Hittable> objects) {
            this.objects = new ArrayList<>(objects);
            this.indexMap = new IdentityHashMap<>(objects.size());
            for (int i = 0; i < objects.size(); i++) {
                indexMap.put(objects.get(i), i);
            }

            //Step 1 - find scene bounding box
            sceneBounds = computeSceneBounds(objects);

            //Step 2 - resolution = cube root of N
            // e.g. 1000 objects -> 10x10x10 grid
            resolution = (int) Math.max(1, Math.cbrt(objects.size()));

            //Step 3 - compute cell size
            //how big is each cell in world units?
            Vec3 sceneSize = sceneBounds.max.subtract(sceneBounds.min);
            cellSize = new Vec3(
                    sceneSize.x / resolution,
                    sceneSize.y / resolution,
                    sceneSize.z / resolution
            );

            //Step 4 - initialize empty cells
            cells = new ArrayList[resolution][resolution][resolution];
            for (int x = 0; x < resolution; x++)
                for(int y = 0; y < resolution ; y++)
                    for(int z = 0; z < resolution ; z++)
                        cells[x][y][z] = new ArrayList<>();

            //Step 5 - insert each object into overlapping cells
            for(Hittable obj : objects){
                insertIntoCells(obj);
            }
    }

    // ─────────────────────────────────────────
    // INSERT
    // ─────────────────────────────────────────

    @Override
    public void insert(Hittable object) {
        objects.add(object);
        insertIntoCells(object);
    }

    /**
     * Finds which cells this object overlaps and adds it there.
     *
     * Steps:
     * 1. Get object bounding box
     * 2. Convert min/max corners to cell indices
     * 3. Loop over all cells in that range
     * 4. Check if cell actually overlaps object box
     * 5. Add object to that cell
     */
    private void insertIntoCells(Hittable object) {
        AABB box = object.getBoundingBox();

        // convert world coords to cell indices
        int xMin = getCellIndex(box.min.x, sceneBounds.min.x, cellSize.x);
        int yMin = getCellIndex(box.min.y, sceneBounds.min.y, cellSize.y);
        int zMin = getCellIndex(box.min.z, sceneBounds.min.z, cellSize.z);

        int xMax = getCellIndex(box.max.x, sceneBounds.min.x, cellSize.x);
        int yMax = getCellIndex(box.max.y, sceneBounds.min.y, cellSize.y);
        int zMax = getCellIndex(box.max.z, sceneBounds.min.z, cellSize.z);

        // clamp to grid boundaries
        xMin = Math.max(0, xMin); yMin = Math.max(0, yMin); zMin = Math.max(0, zMin);
        xMax = Math.min(resolution-1, xMax);
        yMax = Math.min(resolution-1, yMax);
        zMax = Math.min(resolution-1, zMax);

        // add to all overlapping cells
        for (int x = xMin; x <= xMax; x++)
            for (int y = yMin; y <= yMax; y++)
                for (int z = zMin; z <= zMax; z++)
                    cells[x][y][z].add(object);
    }

    // ─────────────────────────────────────────
    // DELETE
    // ─────────────────────────────────────────
    @Override
    public void delete(Hittable object) {
        objects.remove(object);
        // remove from all cells
        for (int x = 0; x < resolution; x++)
            for (int y = 0; y < resolution; y++)
                for (int z = 0; z < resolution; z++)
                    cells[x][y][z].remove(object);
    }


    // ─────────────────────────────────────────
    // TRAVERSE (3D-DDA)
    // ─────────────────────────────────────────
    /**
     * Steps the ray cell by cell using 3D-DDA algorithm.
     * Stops immediately at first hit (early exit).
     */
    @Override
    public Intersection findFirstIntersection(Ray ray) {
        //Step 1 - does the ray hit the scene at all?
        float[] ts = sceneBounds.intersect(ray);
        if(ts == null) return null;

        //Step 2 - find entry point into grid
        float tEntry = Math.max(ts[0], 0);
        Vec3 entryPoint = ray.origin.add(ray.direction.scale(tEntry));

        //Step 3 - find starting cell
        int[] cell = getCell(entryPoint);
        if (cell == null) return null;
        int cx = cell[0];
        int cy = cell[1];
        int cz = cell[2];

        // Step 4 - DDA setup
        // step direction: which way does ray move per axis?
        int stepX = ray.direction.x >= 0 ? 1 : -1;
        int stepY = ray.direction.y >= 0 ? 1 : -1;
        int stepZ = ray.direction.z >= 0 ? 1 : -1;

        // tDelta: how much t increases when crossing one full cell on each axis
        float tDeltaX = Math.abs(cellSize.x / ray.direction.x);
        float tDeltaY = Math.abs(cellSize.y / ray.direction.y);
        float tDeltaZ = Math.abs(cellSize.z / ray.direction.z);

        // tMax: at what t value does the ray cross the NEXT boundary on each axis?
        float tMaxX = computeTMax(entryPoint.x, cx, stepX,
                sceneBounds.min.x, cellSize.x, ray.direction.x);
        float tMaxY = computeTMax(entryPoint.y, cy, stepY,
                sceneBounds.min.y, cellSize.y, ray.direction.y);
        float tMaxZ = computeTMax(entryPoint.z, cz, stepZ,
                sceneBounds.min.z, cellSize.z, ray.direction.z);

        Intersection closest = null;

        // Step 5 - march through cells
        while (cx >= 0 && cx < resolution &&
                cy >= 0 && cy < resolution &&
                cz >= 0 && cz < resolution) {

            // test all objects in this cell
            for (Hittable obj : cells[cx][cy][cz]) {
                int index = indexMap.get(obj);
                Intersection hit = obj.intersect(ray, index);
                if (hit != null && hit.isCloserThan(closest)) {
                    closest = hit;
                }
            }

            // early exit - found a hit in this cell
            if (closest != null) return closest;

            // advance to next cell (move along axis with smallest tMax)
            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                cx += stepX;
                tMaxX += tDeltaX;
            } else if (tMaxY < tMaxZ) {
                cy += stepY;
                tMaxY += tDeltaY;
            } else {
                cz += stepZ;
                tMaxZ += tDeltaZ;
            }
        }

        return closest;

    }

    // ─────────────────────────────────────────
    // HELPER METHODS
    // ─────────────────────────────────────────
    /**
     * Finds the bounding box that wraps ALL objects in the scene.
     */
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
     * Converts a world coordinate to a cell index on one axis.
     * e.g. x=5.0, sceneMin=0.0, cellSize=1.0 → cell index 5
     */
    private int getCellIndex(float pos, float sceneMin, float size) {
        return (int) ((pos - sceneMin) / size);
    }

    /**
     * Returns the [x, y, z] cell indices for a world position.
     * Returns null if position is outside the grid.
    */
    private int[] getCell(Vec3 point) {
        int x = getCellIndex(point.x, sceneBounds.min.x, cellSize.x);
        int y = getCellIndex(point.y, sceneBounds.min.y, cellSize.y);
        int z = getCellIndex(point.z, sceneBounds.min.z, cellSize.z);

        if (x < 0 || x >= resolution ||
                y < 0 || y >= resolution ||
                z < 0 || z >= resolution) return null;

        return new int[]{x, y, z};
    }


    /**
     * Computes the t value at which the ray first crosses a cell boundary on one axis.
     */
    private float computeTMax(float entryPos, int cellIdx, int step,
                              float sceneMin, float cellSize, float dir) {
        float cellBoundary;
        if (step > 0) {
            cellBoundary = sceneMin + (cellIdx + 1) * cellSize;
        } else {
            cellBoundary = sceneMin + cellIdx * cellSize;
        }
        return (cellBoundary - entryPos) / dir;
    }
}
