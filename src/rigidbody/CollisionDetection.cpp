#include <rbd3d/rigidbody/CollisionDetection.h>

#include <set>

namespace rbd3d
{

glm::vec3 normal(const Polytope &p, size_t idx)
{
    const auto &tri = p.faces[idx];
    return glm::cross(p.vertices[tri.y].vert - p.vertices[tri.x].vert, p.vertices[tri.z].vert - p.vertices[tri.x].vert);
}

struct ivec2lt
{
    bool operator()(const glm::ivec2 &v1, const glm::ivec2 &v2) const
    {
        if (v1.x != v2.x)
            return v1.x < v2.x;
        return v1.y < v2.y;
    }
};

void extendPolytope(Polytope &p, const SimplexVert &vert)
{
    std::set<glm::ivec2, ivec2lt> edges;
    std::vector<glm::ivec3> faces;
    faces.reserve(p.faces.size());

    for (size_t i = 0; i < p.faces.size(); ++i)
    {
        glm::vec3 n = normal(p, i);
        if (glm::dot(n, vert.vert - p.vertices[p.faces[i].x].vert) >= 0)
        {
            for (uint8_t j = 0; j < 3; ++j)
            {
                glm::ivec2 edge(p.faces[i][j], p.faces[i][(j + 1) % 3]);
                auto ite = edges.find({edge.y, edge.x});
                if (ite != edges.end())
                    edges.erase(ite);
                else
                    edges.insert(edge);
            }
        }
        else
            faces.push_back(p.faces[i]);
    }

    for (auto &edge : edges)
        faces.push_back({edge.x, edge.y, p.vertices.size()});
    p.vertices.push_back(vert);
    p.faces = faces;
}

} // namespace rbd3d