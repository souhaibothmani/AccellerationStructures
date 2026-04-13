/**
 * Triangle - Represents a triangle in 3D space with per-vertex normals.
 *
 * A triangle is defined by 3 vertices (A, B, C) — the corners.
 * Each vertex also has a NORMAL vector (nA, nB, nC) used for smooth shading.
 *
 * Uses the Möller-Trumbore algorithm (slide 4) for ray intersection,
 * barycentric coordinates (slide 5) to know WHERE on the triangle the hit is,
 * and interpolated normals (slide 6) for smooth shading at the hit point.
 */
public class Triangle {

    // The three corners of the triangle
    public Vec3 A, B, C;

    // Normal vectors at each vertex (for smooth shading / interpolation)
    // These can be different from the flat triangle normal,
    // allowing curved surfaces to look smooth.
    public Vec3 nA, nB, nC;

    public Triangle(Vec3 A, Vec3 B, Vec3 C, Vec3 nA, Vec3 nB, Vec3 nC) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.nA = nA;
        this.nB = nB;
        this.nC = nC;
    }

    /**
     * INTERSECT - Tests if a ray hits this triangle using Möller-Trumbore (slide 4).
     *
     * The idea:
     * Any point on the triangle can be written as:
     *     P = (1 - u - v) * A + u * B + v * C
     *
     * Where u, v are barycentric coordinates (slide 5):
     *   - u = weight for vertex B
     *   - v = weight for vertex C
     *   - w = (1 - u - v) = weight for vertex A
     *   - u >= 0, v >= 0, u + v <= 1 means the point is INSIDE the triangle
     *
     * We set the ray equation equal to the triangle equation:
     *     O + tD = (1 - u - v)A + uB + vC
     *
     * And solve for t, u, v using cross products and dot products.
     * This avoids computing the triangle's plane equation explicitly.
     *
     * @param ray the ray to test
     * @return float array [t, u, v] if hit, null if miss.
     *         t = distance along ray, u and v = barycentric coordinates.
     *         Use ray.pointAt(t) to get the 3D hit point.
     *         Use getInterpolatedNormal(u, v) to get the smooth normal at that point.
     */
    public float[] intersect(Ray ray) {

        // A small value to avoid floating point precision issues
        // (prevents false misses/hits on triangle edges)
        float EPSILON = 0.000001f;

        // E1 = B - A (edge 1: from vertex A to vertex B)
        // E2 = C - A (edge 2: from vertex A to vertex C)
        // These two edges define the triangle's plane
        Vec3 E1 = B.subtract(A);
        Vec3 E2 = C.subtract(A);

        // P = D × E2 (cross product of ray direction and edge 2)
        // This vector is perpendicular to both D and E2.
        // Used to compute the determinant and the u coordinate.
        Vec3 P = ray.direction.cross(E2);

        // det = E1 · P (determinant of the matrix [-D, E1, E2])
        // If det is ~0, the ray is PARALLEL to the triangle → no hit.
        // The determinant also tells us if we're hitting the front or back face.
        float det = E1.dot(P);

        // If determinant is near zero, the ray lies in the triangle's plane
        // or is parallel to it → no intersection possible
        if (det > -EPSILON && det < EPSILON) {
            return null;
        }

        // Precompute 1/det to avoid repeated division (optimization)
        float invDet = 1.0f / det;

        // T = O - A (vector from vertex A to ray origin)
        // This positions the ray origin relative to the triangle
        Vec3 T = ray.origin.subtract(A);

        // u = (T · P) / det
        // First barycentric coordinate — weight for vertex B
        // Geometrically: ratio of area(CAP) / area(ABC) from slide 5
        float u = T.dot(P) * invDet;

        // If u < 0 or u > 1, the intersection point is outside the triangle
        // (past edge CA or edge AB)
        if (u < 0 || u > 1) {
            return null;
        }

        // Q = T × E1 (cross product of T and edge 1)
        // Used to compute v and t
        Vec3 Q = T.cross(E1);

        // v = (D · Q) / det
        // Second barycentric coordinate — weight for vertex C
        // Geometrically: ratio of area(ABP) / area(ABC) from slide 5
        float v = ray.direction.dot(Q) * invDet;

        // If v < 0 or u + v > 1, intersection is outside the triangle
        // (u + v > 1 means we've gone past edge BC)
        if (v < 0 || u + v > 1) {
            return null;
        }

        // t = (E2 · Q) / det
        // This is HOW FAR along the ray the intersection is
        float t = E2.dot(Q) * invDet;

        // If t < 0, the triangle is BEHIND the ray origin → doesn't count
        if (t < EPSILON) {
            return null;
        }

        // Hit! Return t (distance) and u, v (barycentric coordinates)
        // w (weight for vertex A) = 1 - u - v (not returned but can be computed)
        return new float[]{t, u, v};
    }

    /**
     * GET INTERPOLATED NORMAL - Computes the smooth normal at a hit point (slide 6).
     *
     * Instead of using the flat triangle normal (which makes surfaces look faceted),
     * we BLEND the three vertex normals using the barycentric coordinates.
     *
     * Formula from slide 6:
     *     N_final = normalize(w * nA + u * nB + v * nC)
     *
     * Where:
     *   w = 1 - u - v  (weight for vertex A)
     *   u              (weight for vertex B)
     *   v              (weight for vertex C)
     *
     * The closer the hit point is to a vertex, the more that vertex's normal dominates.
     * This is what makes curved surfaces look smooth (Phong shading from session 2).
     *
     * @param u barycentric coordinate for vertex B (from intersect())
     * @param v barycentric coordinate for vertex C (from intersect())
     * @return the interpolated and normalized surface normal at the hit point
     */
    public Vec3 getInterpolatedNormal(float u, float v) {
        // w = weight for vertex A
        float w = 1.0f - u - v;

        // Blend: w * nA + u * nB + v * nC
        // Each vertex normal contributes proportionally to how close the hit is to that vertex
        Vec3 normal = nA.scale(w).add(nB.scale(u)).add(nC.scale(v));

        // Normalize to make it a unit vector (length = 1)
        return normal.normalize();
    }

    @Override
    public String toString() {
        return "Triangle(A=" + A + ", B=" + B + ", C=" + C + ")";
    }
}
