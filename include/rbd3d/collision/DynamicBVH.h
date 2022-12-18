#ifndef RBD3D_COLLISION_DYNAMIC_BVH_H_
#define RBD3D_COLLISION_DYNAMIC_BVH_H_

#include <rbd3d/collision/AABB.h>

#include <vector>

namespace rbd3d
{

constexpr int nullIndex = -1;

struct BVHNode
{
    AABB aabb;
    int index;
    int parent;
    int child1;
    int child2;

    BVHNode(const AABB &_aabb,
            int _index = nullIndex,
            int _parent = nullIndex,
            int _child1 = nullIndex,
            int _child2 = nullIndex)
        : aabb(_aabb), index(_index), parent(_parent), child1(_child1), child2(_child2)
    {
    }
};

class DynamicBVH
{
public:
    void insertLeaf(const AABB &aabb, int index);

private:
    float inheritedAreaDiff(AABB aabb, int index);

private:
    std::vector<BVHNode> m_nodes;
    int m_rootIndex;
};

} // namespace rbd3d

#endif