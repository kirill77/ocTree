#pragma once

#include <array>
#include "basics/bboxes.h"

template <class T>
struct OcBoxStack
{
    OcBoxStack() { }
    OcBoxStack(NvU32 rootIndex, BBox3<T> rootBox)
    {
        m_nodesStack[0] = rootIndex;
        m_boxStack[0] = rootBox;
    }
    OcBoxStack(const OcBoxStack&other) : m_curDepth(other.m_curDepth), m_childBits(other.m_childBits)
    { 
        for (NvU32 u = 0; u <= m_curDepth; ++u)
        {
            m_nodesStack[u] = other.m_nodesStack[u];
            m_boxStack[u] = other.m_boxStack[u];
        }
    }
    const BBox3<T>& getBox(NvU32 uDepth) const { nvAssert(uDepth <= m_curDepth); return m_boxStack[uDepth]; }
    const BBox3<T>& getCurBox() const { return m_boxStack[getCurDepth()]; }
    NvU32 getCurDepth() const { return m_curDepth; }
    NvU32 getNodeIndex(NvU32 uDepth) const { nvAssert(uDepth <= m_curDepth); return m_nodesStack[uDepth]; }
    NvU32 getCurNodeIndex() const { return m_nodesStack[m_curDepth]; }
    bool isDescendent(const OcBoxStack& other) const { return getCurDepth() >= other.getCurDepth() && getNodeIndex(other.getCurDepth()) == other.getCurNodeIndex(); }

    void push(NvU32 uChild, NvU32 uChildNodeIndex)
    {
        nvAssert(getCurDepth() + 1 < ARRAY_ELEMENT_COUNT(m_nodesStack));
        m_nodesStack[getCurDepth() + 1] = uChildNodeIndex;
        auto& srcBox = m_boxStack[m_curDepth];
        auto& dstBox = m_boxStack[++m_curDepth];
        dstBox = srcBox;
        for (NvU32 uDim = 0; uDim < 3; ++uDim)
        {
            NvU32 dimBit = ((uChild >> uDim) & 1);
            m_childBits[uDim] = (m_childBits[uDim] << 1) | dimBit;
            dstBox[dimBit ^ 1][uDim] = (dstBox[0][uDim] + dstBox[1][uDim]) / 2;
        }
    }
    NvU32 pop()
    {
        nvAssert(m_curDepth > 0);
        --m_curDepth;
        NvU32 uChild = 0;
        for (NvU32 uDim = 2; uDim < 3; --uDim)
        {
            NvU32 dimBit = m_childBits[uDim] & 1;
            m_childBits[uDim] >>= 1;
            uChild = (uChild << 1) | dimBit;
        }
        return uChild;
    }

private:
    NvU32 m_curDepth = 0;
    uint3 m_childBits; // remembers which child we went into when doing push()
    std::array<BBox3<T>, 32> m_boxStack;
    std::array<NvU32, 32> m_nodesStack;
};

template <class T>
struct MyRay
{
    bool intersects(const BBox3<T>& bbox) const
    {
        for (NvU32 uDim = 0; uDim < 3; ++uDim)
        {
            if (m_vDir[uDim] <= 0)
            {
                if (m_vOrg[uDim] < bbox.m_vMin[uDim] || bbox.m_vMax[uDim] - m_vOrg[uDim] < m_fMaxDist * m_vDir[uDim])
                    return false;
            }
            else
            {
                if (m_vOrg[uDim] > bbox.m_vMax[uDim] || bbox.m_vMin[uDim] - m_vOrg[uDim] > m_fMaxDist * m_vDir[uDim])
                    return false;
            }
        }
        return true;
    }
    rtvector<T, 3> m_vOrg, m_vDir;
    T m_fMaxDist;
};

template <class Access> // implements all access to outside world required by this class
struct OcTreeNode
{
    typedef OcTreeNode<Access> NodeType;
    typedef typename Access::T T;
    typedef typename Access::NODE_DATA NODE_DATA;

    NODE_DATA m_nodeData;

    void initAsParent(NvU32 uFirstChild)
    {
        m_uFirstChild = uFirstChild;
    }
    bool isLeaf() const { return m_uFirstChild == ~0U; }
    void initLeaf(NvU32 firstPoint, NvU32 endPoint)
    {
        m_uFirstPoint = firstPoint;
        m_uEndPoint = endPoint;
        m_uFirstChild = ~0U;
        nvAssert(isLeaf());
    }
    NvU32 getFirstPoint() const { return m_uFirstPoint; }
    NvU32 getEndPoint() const { return m_uEndPoint; }
    NvU32 getNPoints() const { return m_uEndPoint - m_uFirstPoint; }
    NvU32 getFirstChild() const { nvAssert(!isLeaf()); return m_uFirstChild; }

    struct BoxIterator : public OcBoxStack<T>
    {
        typedef OcBoxStack<T> BASE;

        BoxIterator(NvU32 rootIndex, BBox3<T> rootBox, Access& access) : BASE(rootIndex, rootBox), m_access(access) { }
        BoxIterator(const BoxIterator& other) : BASE(other), m_access(other.m_access) { }
        bool next()
        {
            if (!accessCurNode().isLeaf())
            {
                BASE::push(0, accessCurNode().getFirstChild());
                return true;
            }
            return nextNoDescendent();
        }
        bool nextNoDescendent()
        {
            for ( ; ; )
            {
                if (BASE::getCurDepth() == 0)
                    return false;
                NvU32 uChild = BASE::pop();
                if (uChild == 7)
                {
                    if (BASE::getCurDepth() == 0)
                        return false;
                    continue;
                }
                BASE::push(uChild + 1, accessCurNode().getFirstChild() + uChild + 1);
                return true;
            }
            return false;
        }
        OcTreeNode& accessCurNode() { return m_access.accessNode(BASE::getCurNodeIndex()); }

    private:
        Access& m_access;
    };

    // hierarchical ray tracing
    static void trace(NvU32 rootIndex, const BBox3<T>& rootBox, Access& access, MyRay<T>& ray)
    {
        BoxIterator boxIt(rootIndex, rootBox, access);
        for ( ; ; )
        {
            if (!ray.intersects(boxIt.getCurBox()))
            {
                if (!boxIt.nextNoDescendent())
                    return;
                continue;
            }
            if (!boxIt.next())
                return;
        }
    }

    // add node2node interactions.
    // we try to avoid n^2 complexity by breaking out of the loop early in case all interactions for particular nodes have been fully accounted.
    static void addNode2NodeInteractions(NvU32 rootIndex, const BBox3<T> &rootBox, Access& access)
    {
        BoxIterator it1(rootIndex, rootBox, access);
        do
        {
            const OcTreeNode& node1 = it1.accessCurNode();
            // in the general case it would be too hard to remember which nodes already interacted with which other nodes
            // so we use more restrictive case of node1.isLeaf() && node2 >= node1 (comparison goes over traversal order)
            if (node1.isLeaf())
            {
                if (!access.canHaveInteraction(node1))
                {
                    continue;
                }
                for (BoxIterator it2(it1); ; )
                {
                    const OcTreeNode& node2 = it2.accessCurNode();
                    if (!access.canHaveInteraction(node2))
                    {
                        if (!it2.next()) break;
                        continue;
                    }
                    if (access.addLeafAndNodeInteraction(it1.getCurNodeIndex(), it1, it2.getCurNodeIndex(), it2))
                    {
                        if (!it2.nextNoDescendent()) break;
                        continue;
                    }
                    if (!it2.next()) break;
                }
            }
            nvAssert(access.isOkToBeNotLeaf(node1));
        } while (it1.next());
    }

private:
    NvU32 m_uFirstChild = ~0U; // = ~0U if it's a leaf
    NvU32 m_uFirstPoint;
    NvU32 m_uEndPoint;
};

template <class Access>
struct OcTree
{
    typedef typename Access::T T;

    OcTree(Access& access) : m_access(access)
    {
    }
    bool split(const OcBoxStack<T>& stack)
    {
        auto& node = m_nodes[stack.getCurNodeIndex()];
        nvAssert(node.isLeaf() && node.getNPoints());

        const auto& bbox = stack.getCurBox();
        auto vCenter = bbox.computeCenter();
        NvU32 uFirstPoint = node.getFirstPoint();
        NvU32 uEndPoint = node.getEndPoint();

        NvU32 splitZ = m_access.loosePointsSort(uFirstPoint, uEndPoint, vCenter[2], 2);
        NvU32 splitY0 = m_access.loosePointsSort(uFirstPoint, splitZ, vCenter[1], 1);
        NvU32 splitY1 = m_access.loosePointsSort(splitZ, uEndPoint, vCenter[1], 1);
        NvU32 splitX0 = m_access.loosePointsSort(uFirstPoint, splitY0, vCenter[0], 0);
        NvU32 splitX1 = m_access.loosePointsSort(splitY0, splitZ, vCenter[0], 0);
        NvU32 splitX2 = m_access.loosePointsSort(splitZ, splitY1, vCenter[0], 0);
        NvU32 splitX3 = m_access.loosePointsSort(splitY1, uEndPoint, vCenter[0], 0);

        NvU32 uFirstChild = (NvU32)m_nodes.size();
        node.initAsParent(uFirstChild);
        m_nodes.resize(uFirstChild + 8);

        m_nodes[uFirstChild + 0].initLeaf(uFirstPoint, splitX0);
        m_nodes[uFirstChild + 1].initLeaf(splitX0, splitY0);
        m_nodes[uFirstChild + 2].initLeaf(splitY0, splitX1);
        m_nodes[uFirstChild + 3].initLeaf(splitX1, splitZ);
        m_nodes[uFirstChild + 4].initLeaf(splitZ, splitX2);
        m_nodes[uFirstChild + 5].initLeaf(splitX2, splitY1);
        m_nodes[uFirstChild + 6].initLeaf(splitY1, splitX3);
        m_nodes[uFirstChild + 7].initLeaf(splitX3, uEndPoint);

        return true;
    }

    Access& m_access;
    std::vector<OcTreeNode<Access>> m_nodes;
};