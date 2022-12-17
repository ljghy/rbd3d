#include <rbd3d/collision/CuboidContact.h>

#include <array>

namespace rbd3d
{

struct QueryResult
{
    int i, j;
    float depth;
    glm::vec3 sgn1;
    glm::vec3 sgn2;
};

QueryResult testFace(const Cuboid &c1, const Cuboid &c2,
                     std::array<glm::vec3, 3> &axes)
{
    float proj1, mid, proj2;
    QueryResult ret{-1, 0, std::numeric_limits<float>::max()};
    glm::vec3 halfSize1 = 0.5f * c1.size(), halfSize2 = 0.5f * c2.size();
    for (int i = 0; i < 3; ++i)
    {
        axes[i] = c1.rotation() * axes[i];
        glm::vec3 sgn;
        glm::vec3 sup = c2.support(axes[i], sgn);
        proj1 = glm::dot(sup - c1.translation(), axes[i]);
        mid = glm::dot(c2.translation() - c1.translation(), axes[i]);
        proj2 = 2.f * mid - proj1;

        float depth1 = halfSize1[i] - glm::min(proj1, proj2), depth2 = halfSize1[i] + glm::max(proj1, proj2);
        if (depth1 <= 0 || depth2 <= 0)
        {
            ret.i = -1;
            return ret;
        }
        if (depth1 < ret.depth && depth1 < depth2)
        {
            ret.i = i;
            ret.j = 1;
            ret.depth = depth1;
            ret.sgn1 = sgn;
        }
        else if (depth2 < ret.depth)
        {
            ret.i = i;
            ret.j = -1;
            ret.depth = depth2;
            ret.sgn1 = sgn;
        }
    }
    return ret;
}

QueryResult testEdge(const Cuboid &c1, const Cuboid &c2,
                     const std::array<glm::vec3, 3> &axes1,
                     const std::array<glm::vec3, 3> &axes2)
{
    QueryResult ret{-1, 0, std::numeric_limits<float>::max()};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            glm::vec3 axis = glm::cross(axes1[i], axes2[j]);
            if (axis.x == 0.f && axis.y == 0.f && axis.z == 0.f)
                continue;
            axis = glm::normalize(axis);

            glm::vec3 sgn1, sgn1n, sgn2, sgn2n;
            glm::vec3 sup1 = c1.support(axis, sgn1),
                      sup1n = c1.support(-axis, sgn1n),
                      sup2 = c2.support(axis, sgn2),
                      sup2n = c2.support(-axis, sgn2n);

            float p1 = glm::dot(sup1 - c1.translation(), axis),
                  q1 = glm::dot(sup1n - c1.translation(), axis),
                  p2 = glm::dot(sup2 - c1.translation(), axis),
                  q2 = glm::dot(sup2n - c1.translation(), axis);

            float d1 = glm::max(p2, q2) - glm::min(p1, q1),
                  d2 = glm::max(p1, q1) - glm::min(p2, q2);

            if (d1 <= 0 || d2 <= 0)
            {
                ret.i = -1;
                return ret;
            }

            if (d1 < d2 && d1 < ret.depth) // -axis
            {
                ret.i = i;
                ret.j = j;
                ret.depth = d1;
                ret.sgn1 = sgn1n;
                ret.sgn2 = sgn2;
            }
            else if (d2 < ret.depth) // +axis
            {
                ret.i = i;
                ret.j = j;
                ret.depth = d2;
                ret.sgn1 = sgn1;
                ret.sgn2 = sgn2n;
            }
        }
    return ret;
}

struct Polygon
{
    int count = 0;
    std::array<glm::vec2, 8> vertices;

    void push(const glm::vec2 &vert) { vertices[count++] = vert; }
};

glm::vec2 intersection(const glm::vec2 &v1, const glm::vec2 &v2, int i, float b)
{
    int j = (i + 1) % 2;
    float t = (b - v1[i]) / (v2[i] - v1[i]);
    glm::vec2 ret;
    ret[i] = b;
    ret[j] = (1.f - t) * v1[j] + t * v2[j];
    return ret;
}

Polygon rectClipping(Polygon p, const glm::vec2 &bound)
{
    Polygon ret(p);
    int sgn[]{1, 1, -1, -1};
    int index[]{0, 1, 0, 1};

    for (int j = 0; j < 4; ++j)
    {
        p = ret;
        ret.count = 0;
        int idx = index[j], s = sgn[j];

        for (int i = 0; i < p.count; ++i)
        {
            const auto &curr = p.vertices[i];
            const auto &prev = p.vertices[(i + p.count - 1) % p.count];

            if (s * curr[idx] <= bound[idx])
            {
                if (s * prev[idx] > bound[idx])
                    ret.push(intersection(prev, curr, idx, s * bound[idx]));
                ret.push(curr);
            }
            else if (s * prev[idx] <= bound[idx])
            {
                ret.push(intersection(prev, curr, idx, s * bound[idx]));
            }
        }
    }
    return ret;
}

void reduceContactPoint(int pointCount, const std::array<ContactPoint, 8> &contactPoints, ContactManifold &ret)
{
    if (pointCount < 5)
    {
        ret.pointCount = pointCount;
        for (int i = 0; i < pointCount; ++i)
            ret.contactPoints[i] = contactPoints[i];
        return;
    }
    ret.pointCount = 4;
    ret.contactPoints[0] = contactPoints[0];

    float maxSqrDis = -1.f;
    int i1 = 0;
    for (int i = 1; i < pointCount; ++i)
    {
        glm::vec3 d = contactPoints[i].position - contactPoints[0].position;
        float sqrDis = d.x * d.x + d.y * d.y + d.z * d.z;
        if (sqrDis > maxSqrDis)
        {
            maxSqrDis = sqrDis;
            i1 = i;
        }
    }
    ret.contactPoints[1] = contactPoints[i1];

    float maxArea = -1.f;
    int i2 = 0;
    for (int i = 1; i < pointCount; ++i)
    {
        if (i != i1)
        {
            glm::vec3 e1 = ret.contactPoints[0].position - contactPoints[i].position,
                      e2 = ret.contactPoints[1].position - contactPoints[i].position,
                      c = glm::cross(e1, e2);
            float area = c.x * c.x + c.y * c.y + c.z * c.z;
            if (area > maxArea)
            {
                maxArea = area;
                i2 = i;
            }
        }
    }
    ret.contactPoints[2] = contactPoints[i2];

    maxArea = -1.f;
    int i3 = 0;
    for (int j = 0; j < 3; ++j)
    {
        int j1 = (j + 1) % 3, j2 = (j + 2) % 3;
        glm::vec3 e1 = ret.contactPoints[j1].position - ret.contactPoints[j].position,
                  e2 = ret.contactPoints[j2].position - ret.contactPoints[j].position,
                  c1 = glm::cross(e1, e2);
        for (int i = 1; i < pointCount; ++i)
        {
            if (i != i1 && i != i2)
            {
                glm::vec3 e3 = ret.contactPoints[j1].position - contactPoints[i].position,
                          e4 = ret.contactPoints[j2].position - contactPoints[i].position,
                          c2 = glm::cross(e3, e4);
                if (glm::dot(c2, c1) > 0)
                    continue;
                float area = c2.x * c2.x + c2.y * c2.y + c2.z * c2.z;
                if (area > maxArea)
                {
                    maxArea = area;
                    i3 = i;
                }
            }
        }
    }
    ret.contactPoints[3] = contactPoints[i3];
}

ContactManifold createFaceContact(const Cuboid &c1, const Cuboid &c2,
                                  const QueryResult &faceQuery,
                                  const std::array<glm::vec3, 3> &axes1,
                                  const std::array<glm::vec3, 3> &axes2)
{
    ContactManifold ret;

    int i = faceQuery.i; // reference face index
    int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
    ret.normal = axes1[i] * float(faceQuery.j);

    glm::vec3 ndot = glm::abs(glm::vec3(glm::dot(axes2[0], ret.normal),
                                        glm::dot(axes2[1], ret.normal),
                                        glm::dot(axes2[2], ret.normal)));

    int j = ndot.x > ndot.y && ndot.x > ndot.z ? 0 : (ndot.y > ndot.z ? 1 : 2); // incident face index
    int j1 = (j + 1) % 3, j2 = (j + 2) % 3;

    float k = -glm::sign(glm::dot(axes2[j], ret.normal));
    glm::quat invRot = glm::conjugate(c1.rotation());
    glm::vec3 faceNormal = axes2[j] * k;
    faceNormal = invRot * faceNormal;
    glm::vec3 faceCenter(0.f);
    faceCenter[j] = k * 0.5f * c2.size()[j];
    faceCenter = invRot * (c2.rotation() * faceCenter + c2.translation() - c1.translation());

    Polygon polygon{4};
    float sx[]{1.f, -1.f, -1.f, 1.f};
    float sy[]{1.f, 1.f, -1.f, -1.f};
    for (int s = 0; s < 4; ++s)
    {
        glm::vec3 diag(0.5f * c2.size());
        diag[j] = 0;
        diag[j1] *= sx[s];
        diag[j2] *= sy[s];
        diag = invRot * (c2.rotation() * diag);
        glm::vec3 vert = faceCenter + diag;
        polygon.vertices[s] = glm::vec2(vert[i1], vert[i2]);
    }

    polygon = rectClipping(polygon, 0.5f * glm::vec2(c1.size()[i1], c1.size()[i2]));

    int pointCount = 0;
    std::array<ContactPoint, 8> contactPoints;

    for (int s = 0; s < polygon.count; ++s)
    {
        glm::vec3 vert(0.f);
        vert[i1] = polygon.vertices[s].x;
        vert[i2] = polygon.vertices[s].y;
        vert[i] = (glm::dot(faceCenter, faceNormal) - faceNormal[i1] * vert[i1] - faceNormal[i2] * vert[i2]) / faceNormal[i];

        float depth = 0.5f * c1.size()[i] - faceQuery.j * vert[i];
        if (depth > 0)
        {
            vert[i] = faceQuery.j * 0.5f * c1.size()[i];
            vert = c1.rotation() * vert + c1.translation();
            contactPoints[pointCount++] = {vert, depth};
        }
    }

    reduceContactPoint(pointCount, contactPoints, ret);

    return ret;
}

ContactManifold createEdgeContact(const Cuboid &c1, const Cuboid &c2,
                                  const QueryResult &edgeQuery,
                                  const std::array<glm::vec3, 3> &axes1,
                                  const std::array<glm::vec3, 3> &axes2)
{
    glm::vec3 origin1 = c1.rotation() * (edgeQuery.sgn1 * 0.5f * c1.size()) + c1.translation();
    glm::vec3 origin2 = c2.rotation() * (edgeQuery.sgn2 * 0.5f * c2.size()) + c2.translation();
    const glm::vec3 &d1 = axes1[edgeQuery.i], d2 = axes2[edgeQuery.j];

    glm::vec3 delta = origin1 - origin2;
    float ddot = glm::dot(d1, d2);
    float k = 1.f - ddot * ddot,
          t1 = glm::dot(delta, ddot * d2 - d1) / k,
          t2 = glm::dot(delta, d2 - ddot * d1) / k;

    ContactManifold ret;
    ret.pointCount = 1;
    ret.contactPoints[0] = {0.5f * (origin1 + origin2 + t1 * d1 + t2 * d2), edgeQuery.depth};

    ret.normal = glm::cross(d1, d2) / sqrt(k);
    if (glm::dot(ret.normal, origin1 - c1.translation()) < 0.f)
        ret.normal = -ret.normal;
    return ret;
}

ContactManifold cuboidContact(const Cuboid &c1, const Cuboid &c2)
{
    ContactManifold ret;
    std::array<glm::vec3, 3> axes1{glm::vec3(1.f, 0.f, 0.f),
                                   glm::vec3(0.f, 1.f, 0.f),
                                   glm::vec3(0.f, 0.f, 1.f)},
        axes2(axes1);

    QueryResult faceQuery1 = testFace(c1, c2, axes1);
    if (faceQuery1.i < 0)
        return ret;

    QueryResult faceQuery2 = testFace(c2, c1, axes2);
    if (faceQuery2.i < 0)
        return ret;

    QueryResult edgeQuery = testEdge(c1, c2, axes1, axes2);
    if (edgeQuery.i < 0)
        return ret;

    constexpr float tol = 1e-5f;

    if (faceQuery1.depth <= faceQuery2.depth && faceQuery1.depth <= edgeQuery.depth + tol)
    {
        ret = createFaceContact(c1, c2, faceQuery1, axes1, axes2);
    }
    else if (faceQuery2.depth <= edgeQuery.depth + tol)
    {
        ret = createFaceContact(c2, c1, faceQuery2, axes2, axes1);
        ret.normal = -ret.normal;
    }
    else
    {
        ret = createEdgeContact(c1, c2, edgeQuery, axes1, axes2);
    }
    return ret;
}

} // namespace rbd3d