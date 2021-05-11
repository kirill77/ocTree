#pragma once

#include "basics/bboxes.h"

template <class T>
struct OcBoxStack
{
    OcBoxStack() { }
    OcBoxStack(const OcBoxStack&other) : m_curDepth(other.m_curDepth), m_childBits(other.m_childBits)
    { 
        for (NvU32 u = 0; u <= m_curDepth; ++u) m_boxStack[u] = other.m_boxStack[u];
    }

    void init(BBox3<T> rootBox)
    {
        m_curDepth = 0;
        m_childBits = uint3({ 0, 0, 0 });
        m_boxStack[0] = rootBox;
    }

    const BBox3<T>& getBox(NvU32 uDepth) const { nvAssert(uDepth <= m_curDepth); return m_boxStack[uDepth]; }
    NvU32 getCurDepth() const { return m_curDepth; }

    void push(NvU32 uChild) // returns previous values of changed box boundaries
    {
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
};

template <class Access> // implements all access to outside world required by this class
struct OcTreeNode
{
    typedef OcTreeNode<Access> NodeType;
    typedef typename Access::FLOAT_TYPE FLOAT_TYPE;
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

    bool split(const rtvector<FLOAT_TYPE, 3>& vCenter, Access &access)
    {
        nvAssert(isLeaf() && getNPoints());

        NvU32 splitZ  = loosePointsSort(m_uFirstPoint, m_uEndPoint, vCenter[2], 2, access);
        NvU32 splitY0 = loosePointsSort(m_uFirstPoint, splitZ, vCenter[1], 1, access);
        NvU32 splitY1 = loosePointsSort(splitZ, m_uEndPoint, vCenter[1], 1, access);
        NvU32 splitX0 = loosePointsSort(m_uFirstPoint, splitY0, vCenter[0], 0, access);
        NvU32 splitX1 = loosePointsSort(splitY0, splitZ, vCenter[0], 0, access);
        NvU32 splitX2 = loosePointsSort(splitZ, splitY1, vCenter[0], 0, access);
        NvU32 splitX3 = loosePointsSort(splitY1, m_uEndPoint, vCenter[0], 0, access);

        NvU32 uFirstPoint = m_uFirstPoint;
        NvU32 uEndPoint = m_uEndPoint;
        NvU32 uFirstChild = m_uFirstChild = access.getNNodes();
        access.resizeNodes(m_uFirstChild + 8);

        access.accessNode(uFirstChild + 0).initLeaf(uFirstPoint, splitX0);
        access.accessNode(uFirstChild + 1).initLeaf(splitX0, splitY0);
        access.accessNode(uFirstChild + 2).initLeaf(splitY0, splitX1);
        access.accessNode(uFirstChild + 3).initLeaf(splitX1, splitZ);
        access.accessNode(uFirstChild + 4).initLeaf(splitZ, splitX2);
        access.accessNode(uFirstChild + 5).initLeaf(splitX2, splitY1);
        access.accessNode(uFirstChild + 6).initLeaf(splitY1, splitX3);
        access.accessNode(uFirstChild + 7).initLeaf(splitX3, uEndPoint);

        return true;
    }

    struct BoxIterator : public OcBoxStack<FLOAT_TYPE>
    {
        typedef OcBoxStack<FLOAT_TYPE> BASE;

        BoxIterator(NvU32 rootIndex, BBox3<FLOAT_TYPE> rootBox, Access& access) : m_access(access)
        {
            BASE::init(rootBox);
            m_nodesStack[0] = rootIndex;
        }
        BoxIterator(const BoxIterator& other) : BASE(other), m_access(other.m_access)
        {
            for (NvU32 u = 0; u <= BASE::getCurDepth(); ++u)
                m_nodesStack[u] = other.m_nodesStack[u];
        }
        void push(NvU32 uChild)
        {
            nvAssert(BASE::getCurDepth() + 1 < ARRAY_ELEMENT_COUNT(m_nodesStack));
            m_nodesStack[BASE::getCurDepth() + 1] = getNode().getFirstChild() + uChild;
            BASE::push(uChild);
        }
        bool next()
        {
            if (!getNode().isLeaf())
            {
                push(0);
                return true;
            }
            return nextNoDescendent();
        }
        bool nextNoDescendent()
        {
            NvU32 uChild;
            for (; ; )
            {
                if (BASE::getCurDepth() == 0)
                    return false;
                uChild = BASE::pop();
                if (uChild == 7)
                {
                    if (BASE::getCurDepth() == 0)
                        return false;
                    continue;
                }
                push(uChild + 1);
                return true;
            }
            return false;
        }
        OcTreeNode& getNode()
        {
            return m_access.accessNode(getNodeIndex());
        }
        NvU32 getNodeIndex() const { return m_nodesStack[BASE::getCurDepth()]; }

    private:
        Access& m_access;
        NvU32 m_nodesStack[32];
    };

    // hierarhical computation of force - tries to avoid N^2 complexity by lumping large groups of far-away particles together
    static void computeForces(NvU32 rootIndex, const BBox3<FLOAT_TYPE> &rootBox, Access& access)
    {
        bool bIt1NextSucceeded = true;
        for (BoxIterator it1(rootIndex, rootBox, access); bIt1NextSucceeded; bIt1NextSucceeded = it1.next())
        {
            if (!it1.getNode().getNPoints())
                continue;
            const OcTreeNode& node1 = it1.getNode();
            if (node1.isLeaf())
            {
                for (NvU32 uPoint1 = node1.m_uFirstPoint; uPoint1 < node1.m_uEndPoint; ++uPoint1)
                {
                    for (NvU32 uPoint2 = uPoint1 + 1; uPoint2 < node1.m_uEndPoint; ++uPoint2)
                    {
                        access.addPoint2PointContributions(uPoint1, uPoint2);
                    }
                }
            }
            BoxIterator it2(it1);
            if (!it2.nextNoDescendent())
                continue;
            bool bIt2NextSuceeded = true;
            for (; bIt2NextSuceeded; )
            {
                if (!it2.getNode().getNPoints())
                {
                    bIt2NextSuceeded = it2.next();
                    continue;
                }
                if (access.addNode2NodeContributions(it1.getNodeIndex(), it1, it2.getNodeIndex(), it2))
                {
                    bIt2NextSuceeded = it2.nextNoDescendent();
                    continue;
                }
                if (it1.getNode().isLeaf() && it2.getNode().isLeaf())
                {
                    const NodeType& node2 = it2.getNode();
                    for (NvU32 uPoint2 = node2.m_uFirstPoint; uPoint2 < node2.m_uEndPoint; ++uPoint2)
                    {
                        if (!access.addNode2PointContributions(it1.getNodeIndex(), it1, uPoint2))
                        {
                            for (NvU32 uPoint1 = node1.m_uFirstPoint; uPoint1 < node1.m_uEndPoint; ++uPoint1)
                            {
                                access.addPoint2PointContributions(uPoint1, uPoint2);
                            }
                        }
                    }
                    bIt2NextSuceeded = it2.next();
                    continue;
                }
                bIt2NextSuceeded = it2.next();
            }
        }
    }

private:
    // returns index of first point for which points[u][uDim] >= fSplit
    static NvU32 loosePointsSort(NvU32 uBegin, NvU32 uEnd, FLOAT_TYPE fSplit, NvU32 uDim, Access &access)
    {
        for ( ; ; ++uBegin)
        {
            nvAssert(uBegin <= uEnd);
            if (uBegin == uEnd)
                return uEnd;
            FLOAT_TYPE f1 = access.getPoint(uBegin)[uDim];
            if (f1 < fSplit)
                continue;
            // search for element with which we can swap
            for (--uEnd; ; --uEnd)
            {
                nvAssert(uBegin <= uEnd);
                if (uBegin == uEnd)
                    return uEnd;
                FLOAT_TYPE f2 = access.getPoint(uEnd)[uDim];
                if (f2 < fSplit)
                    break;
            }
            access.swapPoints(uBegin, uEnd);
        }
    }
    NvU32 m_uFirstChild = ~0U; // = ~0U if it's a leaf
    NvU32 m_uFirstPoint;
    NvU32 m_uEndPoint;
};