#include <rbd3d/rigidbody/GJK.h>

namespace rbd3d
{

SimplexVert support(const RigidbodyBase &a, const RigidbodyBase &b, const glm::vec3 &dir)
{
    glm::vec3 supA = a.support(dir);
    return {supA - b.support(-dir), supA}; // vb = va - vert
}

static bool updateLine(Simplex &simplex, size_t &simplexDim, glm::vec3 &dir)
{
    glm::vec3 e = simplex[1].vert - simplex[0].vert;
    if (glm::dot(simplex[1].vert, e) > 0)
    {
        dir = glm::cross(e, glm::cross(e, simplex[0].vert));
    }
    else
    {
        simplex[0] = simplex[1];
        --simplexDim;
        dir = -simplex[1].vert;
    }
    return false;
}

static bool updateTriangle(Simplex &simplex, size_t &simplexDim, glm::vec3 &dir)
{
    glm::vec3 e0 = simplex[0].vert - simplex[2].vert;
    glm::vec3 e1 = simplex[1].vert - simplex[2].vert;
    glm::vec3 n = glm::cross(e1, e0);

    if (glm::dot(glm::cross(e0, n), simplex[2].vert) > 0)
    {
        if (glm::dot(simplex[2].vert, e0) > 0)
        {
            simplex[0] = simplex[1];
            simplex[1] = simplex[2];
            --simplexDim;
            return updateLine(simplex, simplexDim, dir);
        }
        else
        {
            simplex[1] = simplex[2];
            --simplexDim;
            dir = glm::cross(glm::cross(simplex[1].vert, e0), e0);
        }
    }
    else
    {
        if (glm::dot(glm::cross(n, e1), simplex[2].vert) > 0)
        {
            simplex[0] = simplex[1];
            simplex[1] = simplex[2];
            --simplexDim;
            return updateLine(simplex, simplexDim, dir);
        }
        else
        {
            if (glm::dot(n, simplex[2].vert) > 0)
            {
                dir = -n;
            }
            else
            {
                std::swap(simplex[0], simplex[1]);
                dir = n;
            }
        }
    }
    return false;
}

static bool updateTetrahedron(Simplex &simplex, size_t &simplexDim, glm::vec3 &dir)
{
    glm::vec3 e0 = simplex[0].vert - simplex[3].vert,
              e1 = simplex[1].vert - simplex[3].vert,
              e2 = simplex[2].vert - simplex[3].vert,
              n0 = glm::cross(e1, e2),
              n1 = glm::cross(e2, e0),
              n2 = glm::cross(e0, e1);

    if (glm::dot(n0, simplex[3].vert) < 0)
    {
        simplex[0] = simplex[1];
        simplex[1] = simplex[2];
        simplex[2] = simplex[3];
        --simplexDim;
        return updateTriangle(simplex, simplexDim, dir);
    }

    if (glm::dot(n1, simplex[3].vert) < 0)
    {
        simplex[1] = simplex[2];
        simplex[2] = simplex[3];
        --simplexDim;
        return updateTriangle(simplex, simplexDim, dir);
    }

    if (glm::dot(n2, simplex[3].vert) < 0)
    {
        simplex[2] = simplex[3];
        --simplexDim;
        return updateTriangle(simplex, simplexDim, dir);
    }

    return true;
}

static bool updateSimplex(Simplex &simplex, size_t &simplexDim, glm::vec3 &dir)
{

    switch (simplexDim)
    {
    case 1:
        return updateLine(simplex, simplexDim, dir);
    case 2:
        return updateTriangle(simplex, simplexDim, dir);
    case 3:
        return updateTetrahedron(simplex, simplexDim, dir);
    }
    return false;
}

constexpr int GJK_MAX_ITER = 64;

bool collision(const RigidbodyBase &a, const RigidbodyBase &b, SimplexVert *pSimplex)
{
    auto sup = support(a, b, glm::vec3(1.f, 0.f, 0.f));

    Simplex simplex;
    simplex[0] = sup;
    size_t simplexDim = 0;

    glm::vec3 dir = -sup.vert;

    int iter = 0;
    while (true)
    {
        ++iter;
        if (iter > GJK_MAX_ITER)
            return false;

        sup = support(a, b, dir);
        if (glm::dot(sup.vert, dir) < 0)
            return false;

        ++simplexDim;
        simplex[simplexDim] = sup;

        if (updateSimplex(simplex, simplexDim, dir)) // contains origin
        {
            if (pSimplex != nullptr)
                std::copy(simplex.begin(), simplex.end(), pSimplex);
            return true;
        }
    }

    return false;
}

} // namespace rbd3d