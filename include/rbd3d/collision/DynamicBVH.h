#ifndef RBD3D_COLLISION_DYNAMIC_BVH_H_
#define RBD3D_COLLISION_DYNAMIC_BVH_H_

#include <rbd3d/collision/AABB.h>
#include <rbd3d/rigidbody/RigidbodyBase.h>
#include <vector>

namespace rbd3d
{

extern const int nullIndex;

struct BVHNode
{

    AABB aabb;
    RigidbodyBase *rigidbody;
    int parent;
    int child1;
    int child2;

    BVHNode(const AABB &_aabb, RigidbodyBase *_rigidbody = nullptr,
            int _parent = nullIndex, int _child1 = nullIndex, int _child2 = nullIndex)
        : aabb(_aabb), rigidbody(_rigidbody), parent(_parent), child1(_child1), child2(_child2) {}
};

class DynamicBVH
{
public:
    DynamicBVH();
    void insertLeaf(const AABB &aabb, RigidbodyBase *rigidbody);
    void update();
    void detectCollision(RigidbodyBase *rigidbody, std::vector<RigidbodyBase *> &collisions);

private:
    float inheritedAreaDiff(AABB aabb, int index);

    void removeAndInsert(int index, const AABB &newAABB);
    void insertLeafAt(int leafIndex, int newParent, const AABB &aabb, RigidbodyBase *rigidbody);
    void refit(int index);
    void rotate(int index);

    // TODO: Tree rotation

private:
    std::vector<BVHNode> m_nodes;
    int m_rootIndex;
};

} // namespace rbd3d

#endif