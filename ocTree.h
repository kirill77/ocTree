#pragma once

#include "basics/bboxes.h"

template <class T, NvU32 MAX_DEPTH>
struct OcTreeBoxStack
{
    void init(BBox3<T> rootBox)
    {
        m_curDepth = 0;
        m_curBox = rootBox;
        m_uBox[0] = m_uBox[1] = m_uBox[2] = 0;
    }
    void push(NvU32 uChild)
    {
        for (NvU32 uDim = 0; uDim < 3; ++uDim)
        {
            NvU32 dimBit = ((uChild >> uDim) & 1);
            m_uBox[uDim] = (m_uBox[uDim] << 1) | dimBit;
            m_boundsStack[m_curDepth][uDim] = m_curBox[dimBit ^ 1][uDim];
            m_curBox[dimBit ^ 1][uDim] = (m_curBox[0][uDim] + m_curBox[1][uDim]) / 2;
        }
        ++m_curDepth;
    }
    NvU32 pop() // returns child index (0 to 7)
    {
        --m_curDepth;
        NvU32 uChild = 0;
        for (NvU32 uDim = 2; uDim < 3; --uDim)
        {
            NvU32 dimBit = m_uBox[uDim] & 1;
            m_uBox[uDim] >>= 1;
            m_curBox[dimBit ^ 1][uDim] = m_boundsStack[m_curDepth][uDim];
            uChild = (uChild << 1) | dimBit;
        }
        return uChild;
    }
    const BBox3<T>& getCurBox() const { return m_curBox; }
    NvU32 getCurDepth() const { return m_curDepth; }

protected:
    NvU32 m_curDepth = 0;
private:
    rtvector<T, 3> m_boundsStack[MAX_DEPTH];
    BBox3<T> m_curBox;
    NvU32 m_uBox[3]; // starts with 0,0,0, *= 2 when we descend, += 1 when we shift
};

template <class Access> // implements all access to outside world required by this class
struct OcTreeNode
{
    typedef OcTreeNode<Access> NodeType;
    typedef typename Access::FLOAT_TYPE FLOAT_TYPE;

    OcTreeNode() : m_uFirstChild(~0U) { }
    ~OcTreeNode() { }

    bool isLeaf() const { return m_uEndPoint != ~0U; }

    void initLeaf(NvU32 firstPoint, NvU32 endPoint)
    {
        m_uFirstPoint = firstPoint;
        m_uEndPoint = endPoint;
        nvAssert(isLeaf());
    }
    NvU32 getFirstPoint() const { nvAssert(isLeaf()); return m_uFirstPoint; }
    NvU32 getEndPoint() const { nvAssert(isLeaf()); return m_uEndPoint; }
    NvU32 getNPoints() const { nvAssert(isLeaf()); return m_uEndPoint - m_uFirstPoint; }
    NvU32 getFirstChild() const { nvAssert(!isLeaf()); return m_uFirstChild; }

    bool split(const rtvector<FLOAT_TYPE, 3>& vCenter, Access &access)
    {
        if (!isLeaf() || m_uFirstPoint == m_uEndPoint) return false;

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
        m_uEndPoint = ~0U;
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

    struct BoxIterator : OcTreeBoxStack<float, 32>
    {
        BoxIterator(NvU32 uRootNode, const BBox3f& rootBox, Access& access);
        BoxIterator(const BoxIterator& other);
        void descend(NvU32 childIndex);
        NvU32 ascend(); // returns index of the child where we've been
        NvU32 getCurNodeIndex() const { return m_nodesStack[m_curDepth]; }
        bool isAccuracyMatch(const BoxIterator& other);

    private:
        Access& m_access;
        NvU32 m_nodesStack[32];
    };

    // assuming each point has scalar charge and there is a force acting between each pair of charges, compute cumulative force acting on each point.
    // accuracy is understood like this: if two octree boxes are of the same size and distance between them is >= nAccuracyDist, we compute box<->box
    // force instead of point<->point force
    static void computePointForces(NvU32 rootIndex, Access& access)
    {
        bool bIt1NextSucceeded = true;
        for (BoxIterator it1(rootIndex, access); bIt1NextSucceeded; bIt1NextSucceeded = it1.next())
        {
            if (it1.isEmptyNode())
                continue;
            NodeType& curNode = it1.getCurNode();
            if (curNode.isLeaf())
            {
                access.addPoint2PointContributions(curNode.m_uFirstPoint, curNode.m_uEndPoint);
            }
            BoxIterator it2(it1);
            if (!it2.nextNoDescendent())
                continue;
            bool bIt2NextSuceeded = true;
            for (; bIt2NextSuceeded; )
            {
                if (it2.isEmptyNode())
                {
                    bIt2NextSuceeded = it2.next();
                    continue;
                }
                if (it2.isBox2BoxAccuracyMatch(it1))
                {
                    access.addNode2NodeContributions(it1.getNodeIndex(), it1.getCurBox(), it2.getNodeIndex(), it2.getCurBox());
                    bIt2NextSuceeded = it2.nextNoDescendent();
                    continue;
                }
                if (it2.isLeaf())
                {
                    const NodeType& node1 = it1.getCurNode();
                    const NodeType& node2 = it2.getCurNode();
                    for (NvU32 uPoint2 = node2.m_firstPoint; uPoint2 < node2.m_endPoint; ++uPoint2)
                    {
                        if (it1.isBox2PointAccuracyMatch(uPoint2))
                        {
                            access.addNode2PointContribution(it1.getNodeIndex(), it1.getCurBox(), uPoint2);
                        }
                        else
                        {
                            for (NvU32 uPoint1 = node1.m_firstPoint; uPoint1 < node1.m_uEndPoint; ++uPoint1)
                            {
                                access.addPoint2PointContribution(uPoint1, uPoint2);
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
    union
    {
        NvU32 m_uFirstChild; // index into NodeAllocator
        NvU32 m_uFirstPoint; // index into PointAllocator
    };
    NvU32 m_uEndPoint = ~0U; // inner node would have this set to ~0U
#if 0
    // implementation of BoxIterator methods
    BoxIterator::BoxIterator(NvU32 uRootNode, const BBox3f& rootBox, Access& access) : m_access(access)
    {
        uBox[0] = uBox[1] = uBox[2] = uDepth = 0;
        nodesStack[0] = uRootNode;
        curBox = rootBox;
    }
    BoxIterator::BoxIterator(const BoxIterator& other)
    {
        uBox[0] = other.uBox[0];
        uBox[1] = other.uBox[1];
        uBox[2] = other.uBox[2];
        for (NvU32 u = 0; u < other.uDepth; ++u)
        {
            boundsStack[u] = other.boundsStack[u];
            nodesStack[u] = other.nodesStack[u];
        }
        curBox = other.getCurBox;
        uDepth = other.uDepth;
        nodesStack[uDepth] = other.nodesStack[uDepth];
    }
    void BoxIterator::descend(NodeArray& nodeArray, NvU32 childIndex)
    {
        nvAssert(childIndex < 8 && uDepth < 31);
        for (NvU32 uDim = 0; uDim < 3; ++uDim)
        {
            NvU32 dimBit = ((childIndex >> uDim) & 1);
            uBox[uDim] = (uBox[uDim] << 1) | dimBit;
            boundsStack[uDepth] = curBox[dimBit ^ 1][uDim];
            curBox[dimBit ^ 1][uDim] = (curBox[0][uDim] + curBox[1][uDim]) / 2;
        }
        nodesStack[uDepth + 1] = nodeArray[nodesStack[uDepth]].getChildren();
        ++uDepth;
    }
    NvU32 BoxIterator::ascend()
    {
        --uDepth;
        NvU32 dimBit = uBox[0] & 1;
        curBox[dimBit ^ 1][0] = boundsStack[uDepth][0];
        NvU32 uChild = dimBit;
        for (NvU32 uDim = 1; uDim < 3; ++uDim)
        {
            NvU32 dimBit = uBox[uDim] & 1;
            curBox[dimBit ^ 1][uDim] = boundsStack[uDepth][uDim];
            uChild = (uChild << 1) | dimBit;
        }
        return uChild;
    }
    ACCURACY_DECISION BoxIterator::computeAccuracyDecision(const Iterator& other, NvU32 nAccuracyDist)
    {
        auto uDist = c1.uBox[0] >= c2.uBox[0] ? c1.uBox[0] - c2.uBox[0] : c2.uBox[0] - c1.uBox[0];
        for (NvU32 uDim = 1; uDim < 3; ++uDim)
        {
            uDist = std::max(uDist, c1.uBox[uDim] >= c2.uBox[uDim] ? c1.uBox[uDim] - c2.uBox[uDim] : c2.uBox[uDim] - c1.uBox[uDim]);
        }
        if (uDist < nAccuracyDecision)
            return ACCURACY_TOO_CLOSE;
        // the goal here is to avoid counting same volume twice. ACCURACY_TOO_FAR means that this volume must have been counted on the coarser level
        return uDist / 2 >= nAccuracyDist ? ACCURACY_TOO_FAR : ACCURACY_MATCH;
    }
#endif
};
