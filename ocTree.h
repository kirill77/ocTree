#pragma once

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
    BBox3<T> m_boxStack[32];
    NvU32 m_nodesStack[32];
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

    bool isLeaf() const { return m_uFirstChild == ~0U; }
    void initLeaf(NvU32 firstPoint, NvU32 endPoint)
    {
        m_uFirstPoint = firstPoint;
        m_uEndPoint = endPoint;
        nvAssert(isLeaf());
    }
    NvU32 getFirstPoint() const { return m_uFirstPoint; }
    NvU32 getEndPoint() const { return m_uEndPoint; }
    NvU32 getNPoints() const { return m_uEndPoint - m_uFirstPoint; }
    NvU32 getFirstChild() const { nvAssert(!isLeaf()); return m_uFirstChild; }

    bool split(const OcBoxStack<T>& stack, Access &access)
    {
        nvAssert(isLeaf() && getNPoints());

        NvU32 firstPoints[8] = { 0 };
        NvU32 endPoints[8] = { 0 };
        access.createChildrenPointIndices(stack, firstPoints, endPoints);
        NvU32 uFirstChild = m_uFirstChild = access.getNNodes();
        access.resizeNodes(m_uFirstChild + 8);

        for (NvU32 u = 0; u < 8; ++u)
        {
            access.accessNode(uFirstChild + u).initLeaf(firstPoints[u], endPoints[u]);
        }

        return true;
    }

    struct BoxIterator : public OcBoxStack<T>
    {
        typedef OcBoxStack<T> BASE;

        BoxIterator(NvU32 rootIndex, BBox3<T> rootBox, Access& access) : BASE(rootIndex, rootBox), m_access(access) { }
        BoxIterator(const BoxIterator& other) : BASE(other), m_access(other.m_access) { }
        bool next()
        {
            if (!getNode().isLeaf())
            {
                BASE::push(0, getNode().getFirstChild());
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
                BASE::push(uChild + 1, getNode().getFirstChild() + uChild + 1);
                return true;
            }
            return false;
        }
        OcTreeNode& getNode() { return m_access.accessNode(BASE::getCurNodeIndex()); }

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
    // hierarhical computation of force - tries to avoid N^2 complexity by lumping large groups of far-away particles together
    static void computeForces(NvU32 rootIndex, const BBox3<T> &rootBox, Access& access)
    {
        BoxIterator dstIt(rootIndex, rootBox, access);
        do
        {
            const OcTreeNode& dstNode = dstIt.getNode();
            if (dstNode.isLeaf())
            {
                if (!dstNode.getNPoints())
                {
                    continue;
                }
                for (BoxIterator srcIt(rootIndex, rootBox, access); ; )
                {
                    const OcTreeNode& srcNode = srcIt.getNode();
                    if (!srcNode.getNPoints())
                    {
                        nvAssert(srcNode.isLeaf());
                        if (!srcIt.next()) break;
                        continue;
                    }
                    if (access.addNode2LeafContribution(dstIt.getCurNodeIndex(), dstIt, srcIt.getCurNodeIndex(), srcIt))
                    {
                        if (!srcIt.nextNoDescendent()) break;
                        continue;
                    }
                    if (!srcIt.next()) break;
                }
            }
            nvAssert(dstNode.getNPoints()); // internal node must have points in it - otherwise why did we split it
        } while (dstIt.next());
    }

private:
    NvU32 m_uFirstChild = ~0U; // = ~0U if it's a leaf
    NvU32 m_uFirstPoint;
    NvU32 m_uEndPoint;
};