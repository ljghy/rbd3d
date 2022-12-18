#include <rbd3d/collision/DynamicBVH.h>

#include <queue>

#include <cassert>

namespace rbd3d
{

const int nullIndex = -1;

struct AreaCost
{
    int index;
    float cost;

    bool operator<(const AreaCost &other) const { return cost < other.cost; }
};

DynamicBVH::DynamicBVH() : m_rootIndex(nullIndex) {}

float DynamicBVH::inheritedAreaDiff(AABB aabb, int index)
{
    aabb = AABB::merge(aabb, m_nodes[index].aabb);
    float area = 0.f;
    index = m_nodes[index].parent;
    while (index != nullIndex)
    {
        aabb = AABB::merge(aabb, m_nodes[index].aabb);
        area += aabb.area() - m_nodes[index].aabb.area();
        index = m_nodes[index].parent;
    }
    return area;
}

void DynamicBVH::refit(int index)
{
    while (index != nullIndex)
    {
        int child1 = m_nodes[index].child1;
        int child2 = m_nodes[index].child2;
        m_nodes[index].aabb = AABB::merge(m_nodes[child1].aabb, m_nodes[child2].aabb);
        index = m_nodes[index].parent;
    }
}

void DynamicBVH::removeAndInsert(int index, const AABB &newAABB)
{
    assert(m_nodes[index].child1 == nullIndex &&
           m_nodes[index].child2 == nullIndex &&
           index < m_nodes.size());
    if (m_nodes.size() == 1)
    {
        m_nodes[0].aabb = newAABB;
        return;
    }
    int parent = m_nodes[index].parent;
    int theOtherChild = (index == m_nodes[parent].child1 ? m_nodes[parent].child2 : m_nodes[parent].child1);
    if (parent == m_rootIndex)
        m_rootIndex = theOtherChild;
    int grandparent = m_nodes[parent].parent;
    m_nodes[theOtherChild].parent = grandparent;
    if (grandparent != nullIndex)
        (parent == m_nodes[grandparent].child1 ? m_nodes[grandparent].child1 : m_nodes[grandparent].child2) = theOtherChild;
    refit(grandparent);

    insertLeafAt(index, parent, newAABB, m_nodes[index].rigidbody);
}

void DynamicBVH::insertLeafAt(int leafIndex, int newParent, const AABB &aabb, RigidbodyBase *rigidbody)
{
    m_nodes[leafIndex].aabb = aabb;
    m_nodes[leafIndex].rigidbody = rigidbody;

    int sibling = m_rootIndex;
    float minCost = AABB::merge(aabb, m_nodes[m_rootIndex].aabb).area();
    std::priority_queue<AreaCost> q;
    q.push({m_rootIndex, minCost});

    while (!q.empty())
    {
        AreaCost curr = q.top();
        q.pop();
        if (curr.index != m_rootIndex)
        {
            curr.cost = AABB::merge(aabb, m_nodes[curr.index].aabb).area() +
                        inheritedAreaDiff(aabb, curr.index);
            if (curr.cost < minCost)
            {
                minCost = curr.cost;
                sibling = curr.index;
            }
        }
        if (m_nodes[curr.index].child1 == nullIndex)
        {
            continue;
        }
        float lowerBound = aabb.area() + curr.cost - m_nodes[curr.index].aabb.area();
        if (lowerBound < minCost)
        {
            q.push({m_nodes[curr.index].child1, lowerBound});
            q.push({m_nodes[curr.index].child2, lowerBound});
        }
    }

    int oldParent = m_nodes[sibling].parent;
    m_nodes[newParent] = {AABB::merge(aabb, m_nodes[sibling].aabb), nullptr, oldParent};

    if (oldParent != nullIndex)
    {
        (m_nodes[oldParent].child1 == sibling ? m_nodes[oldParent].child1 : m_nodes[oldParent].child2) = newParent;
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leafIndex;
        m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
    }
    else
    {
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leafIndex;
        m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
        m_rootIndex = newParent;
    }

    refit(m_nodes[leafIndex].parent);
}

void DynamicBVH::insertLeaf(const AABB &aabb, RigidbodyBase *rigidbody)
{
    int leafIndex = m_nodes.size();
    m_nodes.emplace_back(aabb, rigidbody);
    if (m_nodes.size() == 1)
    {
        m_rootIndex = leafIndex;
        return;
    }

    int sibling = m_rootIndex;
    float minCost = AABB::merge(aabb, m_nodes[m_rootIndex].aabb).area();
    std::priority_queue<AreaCost> q;
    q.push({m_rootIndex, minCost});

    while (!q.empty())
    {
        AreaCost curr = q.top();
        q.pop();
        if (curr.index != m_rootIndex)
        {
            curr.cost = AABB::merge(aabb, m_nodes[curr.index].aabb).area() +
                        inheritedAreaDiff(aabb, curr.index);
            if (curr.cost < minCost)
            {
                minCost = curr.cost;
                sibling = curr.index;
            }
        }
        if (m_nodes[curr.index].child1 == nullIndex)
        {
            continue;
        }
        float lowerBound = aabb.area() + curr.cost - m_nodes[curr.index].aabb.area();
        if (lowerBound < minCost)
        {
            q.push({m_nodes[curr.index].child1, lowerBound});
            q.push({m_nodes[curr.index].child2, lowerBound});
        }
    }

    int oldParent = m_nodes[sibling].parent;
    int newParent = m_nodes.size();
    m_nodes.emplace_back(AABB::merge(aabb, m_nodes[sibling].aabb), nullptr, oldParent);

    if (oldParent != nullIndex)
    {
        (m_nodes[oldParent].child1 == sibling ? m_nodes[oldParent].child1 : m_nodes[oldParent].child2) = newParent;
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leafIndex;
        m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
    }
    else
    {
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leafIndex;
        m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
        m_rootIndex = newParent;
    }

    refit(m_nodes[leafIndex].parent);
}

void DynamicBVH::update()
{

    for (int i = 0; i < m_nodes.size(); ++i)
        if (m_nodes[i].rigidbody != nullptr && !(m_nodes[i].rigidbody->tightAABB() <= m_nodes[i].aabb))
        {
            removeAndInsert(i, m_nodes[i].rigidbody->enlargedAABB(1.2f));
        }
}

void DynamicBVH::detectCollision(RigidbodyBase *rigidbody, std::vector<RigidbodyBase *> &collisions)
{
    detectCollision(rigidbody, rigidbody->tightAABB(), collisions, m_rootIndex);
}

void DynamicBVH::detectCollision(RigidbodyBase *rigidbody, const AABB &aabb, std::vector<RigidbodyBase *> &collisions, int index)
{
    if (index == nullIndex || (m_nodes[index].child1 == nullIndex && rigidbody <= m_nodes[index].rigidbody))
        return;
    if (AABB::intersect(aabb, m_nodes[index].aabb))
    {
        if (m_nodes[index].child1 == nullIndex)
            collisions.push_back(m_nodes[index].rigidbody);
        else
        {
            detectCollision(rigidbody, aabb, collisions, m_nodes[index].child1);
            detectCollision(rigidbody, aabb, collisions, m_nodes[index].child2);
        }
    }
}

} // namespace rbd3d