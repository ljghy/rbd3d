#include <rbd3d/collision/DynamicBVH.h>

#include <queue>

namespace rbd3d
{

struct AreaCost
{
    int index;
    float cost;

    bool operator<(const AreaCost &other) const { return cost < other.cost; }
};

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

void DynamicBVH::insertLeaf(const AABB &aabb, int index)
{
    int leafIndex = m_nodes.size();
    m_nodes.emplace_back(aabb, index);
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
    m_nodes.emplace_back(AABB::merge(aabb, m_nodes[sibling].aabb), nullIndex, oldParent);

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

    int i = m_nodes[leafIndex].parent;
    while (i != nullIndex)
    {
        int child1 = m_nodes[i].child1;
        int child2 = m_nodes[i].child2;
        m_nodes[i].aabb = AABB::merge(m_nodes[child1].aabb, m_nodes[child2].aabb);
        i = m_nodes[i].parent;
    }
}

} // namespace rbd3d