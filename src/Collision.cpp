#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <array>
#include <numeric>     
#include <algorithm>   
#include <vector>
#include <chrono>
#include <random>
#include <type_traits>

#include "Collision.h"
#include "Macro.h"

// =======================================================
// Utility functions
// =======================================================

constexpr Real Epsilon = 1e-6f;

static constexpr int perpendicular_axes[3][2] = {{1, 2}, {0, 2}, {0, 1}};

constexpr AxisIndices getAxes(Axis axis) 
{
    int up = static_cast<int>(axis);
    return {up, perpendicular_axes[up][0], perpendicular_axes[up][1]};
}

inline bool isBetween(Real val, Real a, Real b) 
{
    return (val >= std::min(a, b)) && (val <= std::max(a, b));
}

void print(const Real3& v, const char* prefix = "") 
{
    printf("%s (%.3f, %.3f, %.3f)\n", prefix, v.x, v.y, v.z);
}

inline Real2 project(const Real3& v, Axis axis)
{
    static const int map[3][2] = { {1, 2}, {0, 2}, {0, 1} };
    
    const Real* ptr = &v.x;
    return Real2(ptr[map[(int)axis][0]], ptr[map[(int)axis][1]]);
}

inline Real3 unproject(const Real2& v, Axis axis)
{
    static const int inv_map[3][2] = { {1, 2}, {0, 2}, {0, 1} };

    Real3 result(0.0f);
    Real* ptr = &result.x;

    ptr[inv_map[(int)axis][0]] = v.x;
    ptr[inv_map[(int)axis][1]] = v.y;

    return result;
}

inline Real rmin(Real a, Real b) { return (a < b) ? a : b; }

inline Real rmax(Real a, Real b) { return (a > b) ? a : b; }

inline Real3 rmin(const Real3& a, const Real3& b) 
{
    return Real3(rmin(a.x, b.x), rmin(a.y, b.y), rmin(a.z, b.z));
}

inline Real3 rmax(const Real3& a, const Real3& b) 
{
    return Real3(rmax(a.x, b.x), rmax(a.y, b.y), rmax(a.z, b.z));
}

inline AABB AABBUnion(const AABB& b1, const AABB& b2) 
{
    AABB u;
    u.minv = rmin(b1.minv, b2.minv);
    u.maxv = rmax(b1.maxv, b2.maxv);
    return u;
}

inline Real AABBSurfaceArea(const AABB& a) 
{
    Real3 d = a.maxv - a.minv;
    return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
}

inline bool AABBOverlap(const AABB& a, const AABB& b) 
{
    if (a.maxv.x < b.minv.x || a.minv.x > b.maxv.x) return false;
    if (a.maxv.y < b.minv.y || a.minv.y > b.maxv.y) return false;
    if (a.maxv.z < b.minv.z || a.minv.z > b.maxv.z) return false;
    return true;
}

inline bool AABBContains(const AABB& outer, const AABB& inner) 
{
    return (outer.minv.x <= inner.minv.x && outer.maxv.x >= inner.maxv.x) &&
           (outer.minv.y <= inner.minv.y && outer.maxv.y >= inner.maxv.y) &&
           (outer.minv.z <= inner.minv.z && outer.maxv.z >= inner.maxv.z);
}

inline AABB AABBEnlarge(const AABB& aabb, Real margin) 
{
    AABB enlarged;
    enlarged.minv = aabb.minv - Real3(margin, margin, margin);
    enlarged.maxv = aabb.maxv + Real3(margin, margin, margin);
    return enlarged;
}

Real3 AABBCenter(const AABB& aabb)
{
    return (aabb.minv + aabb.maxv) * 0.5f;
}

inline Real3 interpolateSegment(const Segment& seg, Real t) 
{
    return seg.start + t * (seg.end - seg.start);
}

inline Real3 interpolateRay(Real3 origin, Real3 direction, Real t) 
{
    return origin + t * direction;
}

inline Real3 interpolateRay(const Ray& ray, Real t)
{
    return ray.origin + ray.direction * t;
}

inline constexpr bool isPositiveAxis(AxisDir axis)
{
    return (static_cast<int>(axis) & 1) == 0;
}

inline constexpr std::tuple<int, int, int> getAxes(AxisDir axis)
{
    constexpr std::array<std::tuple<int, int, int>, 6> axes_table = {{
        {0, 1, 2},
        {0, 1, 2},
        {1, 2, 0},
        {1, 2, 0},
        {2, 0, 1},
        {2, 0, 1} 
    }};
    
    int index = static_cast<int>(axis);
    
    return axes_table[index];
}

inline AABB clipByPlane(const AABB& aabb, int up, Real up_coord, bool positive)
{
    AABB clip_aabb = aabb;

    if (positive) clip_aabb.minv[up] = std::max(clip_aabb.minv[up], up_coord);
    else          clip_aabb.maxv[up] = std::min(clip_aabb.maxv[up], up_coord);

    return clip_aabb;
}

inline Segment clipByPlane(const Segment& seg, int up, Real up_coord, bool positive)
{
    Segment clip_seg = seg;

    Real den = (seg.end[up] - seg.start[up]);

    if (den < Epsilon) return clip_seg;

    Real t = (up_coord - seg.start[up]) / den;

    if (t > 0.0f && t < 1.0f)
    {
        Real3 p = interpolateSegment(seg, t);
        if (positive)
        {
            if (seg.start[up] < up_coord) clip_seg.start = p;
            else                          clip_seg.end   = p;
        }
        else
        {
            if (seg.start[up] < up_coord) clip_seg.end   = p;
            else                          clip_seg.start = p;
        }
    }

    return clip_seg;
}

inline Real3 closestPointOnAABB(const AABB& aabb, const Real3& point)
{
    return glm::clamp(point, aabb.minv, aabb.maxv);
}

inline bool isSegmentAbovePlane(const Segment& seg, int axis, Real plane_coord)
{
    return seg.start[axis] > plane_coord && seg.end[axis] > plane_coord;
}

inline bool isSegmentBelowPlane(const Segment& seg, int axis, Real plane_coord)
{
    return seg.start[axis] < plane_coord && seg.end[axis] < plane_coord;
}

inline bool isAABBAbovePlane(const AABB& aabb, int axis, Real plane_coord)
{
    return aabb.minv[axis] > plane_coord;
}

inline bool isAABBBelowPlane(const AABB& aabb, int axis, Real plane_coord)
{
    return aabb.maxv[axis] < plane_coord;
}

inline Real2 closestPointOnRectangle(Real2 point, Real2 rec_min, Real2 rec_max)
{
    return Real2{
        std::clamp(point.x, rec_min.x, rec_max.x),
        std::clamp(point.y, rec_min.y, rec_max.y)
    };
}

inline Real2 furthestPointOnRectangle(Real2 point, Real2 rec_min, Real2 rec_max)
{
    Real2 center = (rec_min + rec_max) * 0.5f;
    return Real2{
        (point.x < center.x) ? rec_max.x : rec_min.x,
        (point.y < center.y) ? rec_max.y : rec_min.y
    };
}

inline bool isPointInsideRectangle(Real2 point, Real2 rec_min, Real2 rec_max)
{
    return rec_min.x <= point.x && point.x <= rec_max.x && 
           rec_min.y <= point.y && point.y <= rec_max.y;
}

inline Real2 projectOn(const Real3& p, int u, int v)
{
    return Real2(p[u], p[v]);
}

inline Real3 unProjectOn(const Real2& p, int u, int v)
{
    Real3 vec(0.0f);
    vec[u] = p.x;
    vec[v] = p.y;
    return vec;
}

inline Real dot(Real2 v1, Real2 v2) 
{ 
    return v1.x * v2.x + v1.y * v2.y; 
}

inline Real cross(Real2 a, Real2 b)
{ 
    return a.x * b.y - a.y * b.x; 
};


inline Real3 translateOnAxis(Real3 v, Real t, int axis)
{
    v[axis] += t;
    return v;
}

struct ClosestHit 
{
    std::optional<Real> t;
    Real3 point;
    Real3 normal;

    bool isCloser(Real tv) { return !t || tv < *t; }

    inline void update(Real tv) { t = tv; }

    inline void update(Real tv, Real3 hit_point)
    {
        t     = tv;
        point = hit_point;
    }

    inline void update(Real tv, Real3 hit_point, Real3 hit_normal)
    {
        t      = tv;
        point  = hit_point;
        normal = hit_normal;
    }

    explicit operator bool() const { return t.has_value(); }
    inline bool hasHit() const     { return t.has_value(); }

    inline Real3 getPoint()  { return point; }
    inline Real3 getNormal() { return normal; }
    inline Real getT()       { return *t; }

    inline RayCollisionResult toRayCollisionResult() 
    { 
        return hasHit() ? 
            RayCollisionResult {
                .is_colliding   = true, 
                .entering_point = point, 
                .t              = *t, 
                .contact_normal = normal
            } : 
            RayCollisionResult {
                .is_colliding = false
            }; 
    }

    inline void updateIfCloser(RayCollisionResult&& result)
    {
        if (result.is_colliding == false) return;

        if (isCloser(result.t))
        {
            t      = result.t;
            point  = result.entering_point;
            normal = result.contact_normal;
        }
    }

    inline void updateIfHit(RayCollisionResult result)
    {
        if (result.is_colliding == false) return;

        t      = result.t;
        point  = result.entering_point;
        normal = result.contact_normal;
    }

};

// =======================================================
// AABB Wrap Computation
// =======================================================

AABB computeSegmentAABB(const Segment& seg) 
{
    AABB box;
    box.minv = rmin(seg.start, seg.end);
    box.maxv = rmax(seg.start, seg.end);
    return box;
}

AABB computeSphereAABB(const Sphere& sphere)
{
    AABB box;
    Real3 rvec(sphere.radius, sphere.radius, sphere.radius);
    box.minv = sphere.center - rvec;
    box.maxv = sphere.center + rvec;
    return box;
}

AABB computeAACylinderAABB(const AACylinder& aacyl)
{
    static constexpr Real3 r_mask[3] = 
    {
        {0.0f, 1.0f, 1.0f}, // Axis X
        {1.0f, 0.0f, 1.0f}, // Axis Y
        {1.0f, 1.0f, 0.0f}  // Axis Z
    };

    static constexpr Real3 h_mask[3] = 
    {
        {1.0f, 0.0f, 0.0f}, // Axis X
        {0.0f, 1.0f, 0.0f}, // Axis Y
        {0.0f, 0.0f, 1.0f}  // Axis Z
    };

    int idx = (int)aacyl.axis;
    Real r  = aacyl.radius;
    Real h  = aacyl.height;

    Real3 radial_extent = r_mask[idx] * r;
    
    Real3 height_extent = h_mask[idx] * h;

    Real3 v1 = aacyl.base_center - radial_extent;
    Real3 v2 = aacyl.base_center + radial_extent + height_extent;

    AABB box;
    box.minv = rmin(v1, v2);
    box.maxv = rmax(v1, v2);
    return box;
}

AABB computeAARampAABB(const AARamp& aaramp)
{
    return {aaramp.minv, aaramp.maxv};
}

AABB computeAACapsuleAABB(const AACapsule& cap) 
{
    AABB aabb;
   
    aabb.minv = cap.start - Real3(cap.radius);
    aabb.maxv = cap.start + Real3(cap.radius);
    
    Real3 end = cap.start;
    int idx   = (int)cap.axis;
    end[idx] += cap.height;

    Real3 end_min = end - Real3(cap.radius);
    Real3 end_max = end + Real3(cap.radius);

    aabb.minv = glm::min(aabb.minv, end_min);
    aabb.maxv = glm::max(aabb.maxv, end_max);

    return aabb;
}

AABB computeCapsuleAABB(const Capsule& cap)
{
    Real3 r(cap.radius);
    return {
        glm::min(cap.start, cap.end) - r,
        glm::max(cap.start, cap.end) + r
    };
}

AABB computeAARectangleAABB(const AARectangle& aarect)
{
    return { glm::min(aarect.p1, aarect.p2), glm::max(aarect.p1, aarect.p2)};
}

AABB computeAxialOBBAABB(const AxialOBB& axialobb)
{
    Real3 world_extents;
    
    Real c = std::abs(std::cos(axialobb.angle));
    Real s = std::abs(std::sin(axialobb.angle));

    int ax = (int)axialobb.rotation_axis;
    int i1 = (ax + 1) % 3;
    int i2 = (ax + 2) % 3;

    world_extents[ax] = axialobb.extents[ax];

    world_extents[i1] = axialobb.extents[i1] * c + axialobb.extents[i2] * s;
    world_extents[i2] = axialobb.extents[i1] * s + axialobb.extents[i2] * c;

    return {
        axialobb.center - world_extents,
        axialobb.center + world_extents
    };
}

AABB computeAA4SidedPyramidAABB(const AA4SidedPyramid& aa4pyramid)
{
    auto [up, u, v] = getAxes(aa4pyramid.axis);

    Real3 half_extents(0.0f);
    half_extents[u] = aa4pyramid.half_extents.x;
    half_extents[v] = aa4pyramid.half_extents.y;

    Real3 height(0.0f);
    height[up] = aa4pyramid.height;

    const Real3 bc(aa4pyramid.base_center);

    if (aa4pyramid.height < 0.0f) return {bc + height - half_extents, bc + half_extents};
    else                          return {bc - half_extents, bc + height + half_extents};
}

AABB computeOBBAABB(const OBB& obb)    { ASSERT(false, "TODO"); }
AABB computeRayAABB(const Ray& ray)    { ASSERT(false, "IMPOSSIBLE"); }
AABB computeAABBAABB(const AABB& aabb) { return aabb; }

AABB computeCollisionShapeAABB(const CollisionShape& coll_shape)
{
    switch (coll_shape.type)
    {
        #define X(type_name, union_name) case CollisionShapeType::type_name: return compute ##type_name ##AABB(coll_shape.##union_name);
        COLLISION_TYPES
        #undef X

        default: ASSERT(false, "Unknown CollisionShapeType encountered during AABB computation."); break;
    }

    return AABB{}; 
}

// =======================================================
// Updating Functions
// =======================================================

void updateSphere(Sphere& sphere, const Sphere& rest_sphere, Real3 p)
{
    sphere.radius = rest_sphere.radius;
    sphere.center = rest_sphere.center + p;
}

void updateAACylinder(AACylinder& aacyl, const AACylinder& rest_aacyl, Real3 p)
{
    aacyl = rest_aacyl;
    aacyl.base_center = rest_aacyl.base_center + p;
}

void updateSegment(Segment& seg, const Segment& rest_seg, Real3 p)
{
    seg.start = rest_seg.start + p;
    seg.end   = rest_seg.end + p;
}

void updateAABB(AABB& aabb, const AABB& rest_aabb, Real3 position)
{
    aabb.minv = rest_aabb.minv + position;
    aabb.maxv = rest_aabb.maxv + position;
}

void updateAARamp(AARamp& aaramp, const AARamp& rest_aaramp, Real3 position)
{
    aaramp.dir  = rest_aaramp.dir;
    aaramp.minv = rest_aaramp.minv + position;
    aaramp.maxv = rest_aaramp.maxv + position;
}

void updateAACapsule(AACapsule& aacapsule, const AACapsule& rest_aacapsule, Real3 p)
{
    aacapsule = rest_aacapsule;
    aacapsule.start = rest_aacapsule.start + p;
}

void updateCapsule(Capsule& capsule, const Capsule& rest_capsule, Real3 p)
{
    capsule       = rest_capsule;
    capsule.start = rest_capsule.start + p;
    capsule.end   = rest_capsule.end + p;
}

void updateAARectangle(AARectangle& aarect, const AARectangle& rest_aarect, Real3 p)
{
    aarect    = rest_aarect;
    aarect.p1 = rest_aarect.p1 + p;
    aarect.p2 = rest_aarect.p2 + p;
}

void updateAxialOBB(AxialOBB& axialobb, const AxialOBB& rest_axialobb, Real3 p)
{
    axialobb = rest_axialobb;
    axialobb.center = rest_axialobb.center + p;
}

void updateAA4SidedPyramid(AA4SidedPyramid& aa4pyr, const AA4SidedPyramid& rest_aa4pyr, Real3 p)
{
    aa4pyr = rest_aa4pyr;
    aa4pyr.base_center = p;
}

void updateOBB(OBB& obb, const OBB& rest_obb, Real3 p) { ASSERT(false, "TODO"); }
void updateRay(Ray& ray, const Ray& rest_ray, Real3 p) { ASSERT(false, "IMPOSSIBLE"); }

void updateCollisionShape(CollisionShape& runtime_shape, const CollisionShape& rest_shape, const Real3& pos)
{

    switch(rest_shape.type)
    {
        #define X(type_name, union_name) case CollisionShapeType::type_name: update ##type_name(runtime_shape.##union_name, rest_shape.##union_name, pos); break;
        COLLISION_TYPES
        #undef X

        default: ASSERT(false, "Unknown CollisionShapeType encountered during update."); break;
    }

    runtime_shape.type = rest_shape.type;
}

// SPECIAL CASES

void updateSegment(Segment& seg, Real3 p1, Real3 p2)
{
    seg.start = p1;
    seg.end   = p2;
}

void updateCollisionShape(CollisionShape& runtime_shape, const CollisionShape& rest_shape, const Real3& pos, const Real3& prev_pos)
{
    ASSERT(rest_shape.type == CollisionShapeType::Segment, "if current and prev position are given to updat efunction must be segment");

    updateSegment(runtime_shape.segment, pos, prev_pos);

    runtime_shape.type = rest_shape.type;
}

// =======================================================
// Precalc Computation
// =======================================================

#define DefaultPrecalcComputationFunction(type_name) \
void compute ##type_name ##Precalc(const type_name & shape, type_name ##Precalc & precalc) {  } \

DefaultPrecalcComputationFunction(AABB);
DefaultPrecalcComputationFunction(AARamp);
DefaultPrecalcComputationFunction(Ray);
DefaultPrecalcComputationFunction(Segment);
DefaultPrecalcComputationFunction(OBB);
DefaultPrecalcComputationFunction(AACylinder);
DefaultPrecalcComputationFunction(Sphere);
DefaultPrecalcComputationFunction(Capsule);
DefaultPrecalcComputationFunction(AARectangle);
DefaultPrecalcComputationFunction(AACapsule);
DefaultPrecalcComputationFunction(AxialOBB);
DefaultPrecalcComputationFunction(AA4SidedPyramid);

// =======================================================
// AABB Tree
// =======================================================

// global stack definition
Stack aabb_tree_stack;

AABBTree createAABBTree() {
    AABBTree tree;
    tree.node_count = 0;
    tree.root_idx   = nullAABBTreeIndex;
    return tree;
}

int allocateLeafNode(AABBTree& tree, AABB box, StaticAABBTreeNodeData object_data) 
{
    int idx = tree.node_count++;

    ASSERT(idx < AABB_TREE_MAX_OBJECTS, "number of objects in AABB static tree must be less than AABB_TREE_MAX_OBJECTS");

    Node& node = tree.nodes[idx];
    node.box         = box;
    node.object_data = object_data;
    node.parent_idx  = nullAABBTreeIndex;
    node.is_leaf     = true;

    return idx;
}

int allocateInternalNode(AABBTree& tree) 
{
    int idx = tree.node_count++;

    Node& node = tree.nodes[idx];
    node.parent_idx = nullAABBTreeIndex;
    node.child1     = nullAABBTreeIndex;
    node.child2     = nullAABBTreeIndex;
    node.is_leaf    = false;

    return idx;
}

// TODO: consistency with dimensions type: I would remove dimensions defined with preprocessor, but need to find a way to use enum as int easily

#define XD 0
#define YD 1
#define ZD 2

using Dim = uint8_t;

std::vector<Size> sort_indexes(const std::vector<Real3> &v, Dim dim) {

    std::vector<Size> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    if (dim==XD)
        std::stable_sort(idx.begin(), idx.end(), [&v](Size i1, Size i2) {return v[i1].x < v[i2].x;});
    else if (dim==YD)
        std::stable_sort(idx.begin(), idx.end(), [&v](Size i1, Size i2) {return v[i1].y < v[i2].y;});
    else if (dim==ZD)
        std::stable_sort(idx.begin(), idx.end(), [&v](Size i1, Size i2) {return v[i1].z < v[i2].z;});
    else
        ASSERT(false, "sorting dimension must be 0 or 1 or 2");

    return idx;
}

int constuctStaticAABBTreeAux(AABBTree &tree, std::vector<AABB> &boxs, std::vector<StaticAABBTreeNodeData> &data) 
{
    ASSERT(boxs.size() == data.size(), "number of boxs and data must be equal");
    ASSERT(boxs.size() != 0, "number of boxs/data can't be zero");

    if (boxs.size() == 1) 
    {
        return allocateLeafNode(tree, boxs[0], data[0]);
    }

    Size size = boxs.size();
    
    std::vector<Real3> centroids;
    centroids.reserve(size);

    for (AABB &box : boxs)
        centroids.push_back((box.minv + box.maxv) * 0.5f);

    Real best_cost  = std::numeric_limits<Real>::max();
    Size best_split = 0;
    Dim  best_dim   = 0;

    std::vector<Size> sort_idxs[3];

    for (Dim dim = XD; dim<=ZD; dim++) 
    {
        sort_idxs[dim] = sort_indexes(centroids, dim);

        // TODO: suffix/prefix AABB by accumulating forward and backwrad

        for (Size split=1; split<size; split++) 
        {
            AABB leftAABB  = boxs[sort_idxs[dim][0]];
            AABB rightAABB = boxs[sort_idxs[dim][size-1]];

            for (int i=0; i<split; i++) 
                leftAABB = AABBUnion(leftAABB, boxs[sort_idxs[dim][i]]);
            for (int i=split; i<size; i++) 
                rightAABB = AABBUnion(rightAABB, boxs[sort_idxs[dim][i]]);

            Real cost = AABBSurfaceArea(leftAABB) * (split) + AABBSurfaceArea(rightAABB) * (size - split);

            if (cost < best_cost) 
            {
                best_cost  = cost;
                best_split = split;
                best_dim   = dim;
            }
        }
    }

    std::vector<AABB> left_boxs;
    std::vector<AABB> right_boxs;
    std::vector<StaticAABBTreeNodeData> left_data;
    std::vector<StaticAABBTreeNodeData> right_data;

    for (int i=0; i<best_split; i++) 
    {
        left_boxs.push_back(boxs[sort_idxs[best_dim][i]]);
        left_data.push_back(data[sort_idxs[best_dim][i]]);
    }

    for (int i=best_split; i<size; i++) 
    {
        right_boxs.push_back(boxs[sort_idxs[best_dim][i]]);
        right_data.push_back(data[sort_idxs[best_dim][i]]);
    }

    int node_idx = allocateInternalNode(tree);
    
    int left_node_idx  = constuctStaticAABBTreeAux(tree, left_boxs,  left_data);
    int right_node_idx = constuctStaticAABBTreeAux(tree, right_boxs, right_data);

    ASSERT(tree.nodes[node_idx].is_leaf == false, "");

    tree.nodes[node_idx].child1 = left_node_idx;
    tree.nodes[node_idx].child2 = right_node_idx;
    tree.nodes[node_idx].box    = AABBUnion(tree.nodes[left_node_idx].box, tree.nodes[right_node_idx].box);

    tree.nodes[left_node_idx].parent_idx  = node_idx;
    tree.nodes[right_node_idx].parent_idx = node_idx;

    return node_idx;
}

AABBTree constuctStaticAABBTree(std::vector<AABB> &boxs, std::vector<StaticAABBTreeNodeData> &data) 
{
    ASSERT(boxs.size() == data.size(), "number of boxs and data must be equal");
    ASSERT(boxs.size() != 0, "number of boxs/data can't be zero");

    AABBTree tree = createAABBTree();

    tree.root_idx = constuctStaticAABBTreeAux(tree, boxs, data);

    return tree;
}

void printAABBTree(const AABBTree& tree, int node_idx, int depth) {
    if (node_idx == nullAABBTreeIndex) return;

    const Node& node = tree.nodes[node_idx];

    // Indentazione in base alla profondità
    for (int i = 0; i < depth; i++) 
        std::cout << "  ";

    // Stampa nodo
    if (node.is_leaf) {
        std::cout << "[Leaf] idx=" << node_idx 
                  << " obj=" << node.object_data.object_idx 
                  << " AABB=(" 
                  << node.box.minv.x << "," << node.box.minv.y << "," << node.box.minv.z
                  << " -> "
                  << node.box.maxv.x << "," << node.box.maxv.y << "," << node.box.maxv.z
                  << ")"
                  << "\n";
    } else {
        std::cout << "[Internal] idx=" << node_idx
                  << " AABB=("
                  << node.box.minv.x << "," << node.box.minv.y << "," << node.box.minv.z
                  << " -> "
                  << node.box.maxv.x << "," << node.box.maxv.y << "," << node.box.maxv.z
                  << ")"
                  << "\n";
    }

    // Ricorsione sui figli
    if (!node.is_leaf) {
        printAABBTree(tree, node.child1, depth + 1);
        printAABBTree(tree, node.child2, depth + 1);
    }
}

void queryOverlap(const AABBTree& tree, const AABB& box, QueryOverlapResult<StaticAABBTreeNodeData>& results)
{
    results.clear();

    if (tree.root_idx == nullAABBTreeIndex) return;

    aabb_tree_stack.clear();
    aabb_tree_stack.push(tree.root_idx);

    while (!aabb_tree_stack.empty()) 
    {
        int node_idx = aabb_tree_stack.pop();
        if (node_idx == nullAABBTreeIndex) continue;

        const Node& node = tree.nodes[node_idx];

        if (AABBOverlap(node.box, box)) 
        {
            if (node.is_leaf) 
            {
                results.add(node.object_data);
            } 
            else 
            {
                aabb_tree_stack.push(node.child1);
                aabb_tree_stack.push(node.child2);
            }
        }
    }
}

void queryOverlap(const AABBTree& tree, const Ray& ray,  QueryOverlapResult<StaticAABBTreeNodeData>& results)
{
    results.clear();

    if (tree.root_idx == nullAABBTreeIndex) return;

    aabb_tree_stack.clear();
    aabb_tree_stack.push(tree.root_idx);

    while (!aabb_tree_stack.empty()) 
    {
        int node_idx = aabb_tree_stack.pop();
        if (node_idx == nullAABBTreeIndex) continue;

        const Node& node = tree.nodes[node_idx];

        RayCollisionResult rc_result = computeRayAABBCollision(ray, node.box);

        if (!rc_result.is_colliding) continue;
        
        if (node.is_leaf) 
        {
            results.add(node.object_data);
        } 
        else 
        {
            aabb_tree_stack.push(node.child1);
            aabb_tree_stack.push(node.child2);
        }
    }
}

// =======================================================
// Dynamic AABB Tree 
// =======================================================

Index dynamicAABBTreeAllocateLeafNode(DynamicAABBTree& tree, AABB box, DynamicAABBTreeNodeData data) 
{
    TRACE_SYSTEM();
    Index idx = tree.nodes.allocate();
    data.leaf_idx = idx;

    DynamicAABBTreeNode& node = tree.nodes[idx];

    node.box            = AABBEnlarge(box, 0.3f);
    node.data           = data;
    node.parent_idx     = nullAABBTreeIndex;
    // node.child1         = nullAABBTreeIndex;
    // node.child2         = nullAABBTreeIndex;
    node.is_leaf        = true;
    node.inherited_cost = 0.0f;
    return idx;
}

Index dynamicAABBTreeAllocateInternalNode(DynamicAABBTree& tree)
{
    TRACE_SYSTEM();
    Index idx = tree.nodes.allocate();
    DynamicAABBTreeNode& node = tree.nodes[idx];

    node.parent_idx     = nullAABBTreeIndex;
    node.child1         = nullAABBTreeIndex;
    node.child2         = nullAABBTreeIndex;
    node.is_leaf        = false;
    node.inherited_cost = 0.0f;

    return idx;
}

Index pickBestSibling(DynamicAABBTree& tree, Index leaf_idx)
{
    TRACE_SYSTEM();
    DynamicAABBTreeNode leaf_node = tree.nodes[leaf_idx];

    Index best_idx = tree.root_idx;
    Real best_cost = AABBSurfaceArea(AABBUnion(tree.nodes[tree.root_idx].box, leaf_node.box));

    aabb_tree_stack.clear();
    aabb_tree_stack.push(tree.root_idx);

    while (!aabb_tree_stack.empty()) 
    {
        int node_idx = aabb_tree_stack.pop();
        DynamicAABBTreeNode& node = tree.nodes[node_idx];

        Real inherited_cost = (node.parent_idx != nullAABBTreeIndex)
                            ? tree.nodes[node.parent_idx].inherited_cost
                            : 0.0f;

        AABB union_aabb = AABBUnion(node.box, leaf_node.box);
        Real union_sa   = AABBSurfaceArea(union_aabb);
        Real node_sa    = AABBSurfaceArea(node.box);

        node.inherited_cost = (union_sa - node_sa) + inherited_cost;
        Real cost = union_sa + inherited_cost;

        if (cost < best_cost) 
        {
            best_cost = cost;
            best_idx = node_idx;
        }

        if (!node.is_leaf) {
            Real leaf_sa = AABBSurfaceArea(leaf_node.box);
            Real low_bound_cost = leaf_sa + node.inherited_cost;

            if (low_bound_cost < best_cost) {
                aabb_tree_stack.push(node.child1);
                aabb_tree_stack.push(node.child2);
            }
        }
    }

    return best_idx;
}


// external interface
DynamicAABBTree createDynamicAABBTree() 
{
    DynamicAABBTree tree;
    tree.root_idx = nullAABBTreeIndex;
    return tree;
}

Index dynamicAABBTreeInsertLeaf(DynamicAABBTree& tree, AABB box, Entity entity)
{
    TRACE_SYSTEM();
    DynamicAABBTreeNodeData data = {entity, nullAABBTreeIndex};
    Index leaf_idx = dynamicAABBTreeAllocateLeafNode(tree, box, data);

    ASSERT(leaf_idx != nullAABBTreeIndex, "Failed to allocate leaf node in dynamic AABB tree");
    ASSERT(tree.nodes[leaf_idx].data.leaf_idx != nullAABBTreeIndex, "Leaf node data not properly set");

    if (tree.root_idx == nullAABBTreeIndex) 
    {
        tree.root_idx = leaf_idx;
        return leaf_idx;
    }

    Index sibling = pickBestSibling(tree, leaf_idx);

    Index old_parent = tree.nodes[sibling].parent_idx;
    Index new_parent = dynamicAABBTreeAllocateInternalNode(tree);

    DynamicAABBTreeNode& parent = tree.nodes[new_parent];
    parent.parent_idx = old_parent;
    parent.box = AABBUnion(box, tree.nodes[sibling].box);

    if (old_parent != nullAABBTreeIndex) 
    {
        DynamicAABBTreeNode& p = tree.nodes[old_parent];
        if (p.child1 == sibling) p.child1 = new_parent;
        else                     p.child2 = new_parent;
    } 
    else 
    {
        tree.root_idx = new_parent;
    }
    
    parent.child1 = sibling;
    parent.child2 = leaf_idx;

    tree.nodes[sibling].parent_idx  = new_parent;
    tree.nodes[leaf_idx].parent_idx = new_parent;

    // Refit tree upwards
    Index idx = new_parent;
    while (idx != nullAABBTreeIndex) 
    {
        DynamicAABBTreeNode& n  = tree.nodes[idx];
        DynamicAABBTreeNode& c1 = tree.nodes[n.child1];
        DynamicAABBTreeNode& c2 = tree.nodes[n.child2];

        n.box = AABBUnion(c1.box, c2.box);
        idx = n.parent_idx;
    }

    return leaf_idx;
}

void dynamicAABBTreeRemoveLeaf(DynamicAABBTree& tree, Index leaf_idx)
{
    TRACE_SYSTEM();
    DynamicAABBTreeNode& leaf_node = tree.nodes[leaf_idx];

    if (leaf_node.parent_idx == nullAABBTreeIndex)
    {
        tree.nodes.deallocate(leaf_idx);
        tree.root_idx = nullAABBTreeIndex;
        return;
    }

    Index parent_idx = leaf_node.parent_idx;
    DynamicAABBTreeNode& parent_node = tree.nodes[parent_idx];

    Index sibling_idx;
    if (parent_node.child1 == leaf_idx) sibling_idx = parent_node.child2;
    else                                sibling_idx = parent_node.child1;
    DynamicAABBTreeNode &sibling_node = tree.nodes[sibling_idx];

    Index grandparent_idx = parent_node.parent_idx;

    if (grandparent_idx != nullAABBTreeIndex)
    {
        DynamicAABBTreeNode &grandparent_node = tree.nodes[grandparent_idx];

        if (grandparent_node.child1 == parent_idx) grandparent_node.child1 = sibling_idx;
        else                                       grandparent_node.child2 = sibling_idx;

        sibling_node.parent_idx = grandparent_idx;
    }
    else
    {
        tree.root_idx           = sibling_idx;
        sibling_node.parent_idx = nullAABBTreeIndex;
    }

    tree.nodes.deallocate(leaf_idx);
    tree.nodes.deallocate(parent_idx);

    Index idx = grandparent_idx;
    while (idx != nullAABBTreeIndex) 
    {
        DynamicAABBTreeNode& n  = tree.nodes[idx];
        DynamicAABBTreeNode& c1 = tree.nodes[n.child1];
        DynamicAABBTreeNode& c2 = tree.nodes[n.child2];

        n.box = AABBUnion(c1.box, c2.box);
        idx   = n.parent_idx;
    }
}

Index dynamicAABBTreeUpdateLeaf(DynamicAABBTree& tree, Index leaf_idx, AABB new_box) 
{
    TRACE_SYSTEM();
    DynamicAABBTreeNode& leaf_node = tree.nodes[leaf_idx];

    if (AABBContains(leaf_node.box, new_box)) 
    {
        return leaf_idx;
    }

    DynamicAABBTreeNodeData data = tree.nodes[leaf_idx].data;

    dynamicAABBTreeRemoveLeaf(tree, leaf_idx);

    Index new_leaf_idx = dynamicAABBTreeInsertLeaf(tree, new_box, data.entity);

    return new_leaf_idx;
}


void queryOverlap(const DynamicAABBTree& tree, const AABB& box, QueryOverlapResult<DynamicAABBTreeNodeData>& results)
{
    TRACE_SYSTEM();
    results.clear();

    if (tree.root_idx == nullAABBTreeIndex) return;

    aabb_tree_stack.clear();
    aabb_tree_stack.push(tree.root_idx);

    while (!aabb_tree_stack.empty()) 
    {
        Index node_idx = aabb_tree_stack.pop();
        if (node_idx == nullAABBTreeIndex) continue;

        const DynamicAABBTreeNode& node = tree.nodes[node_idx];

        if (AABBOverlap(node.box, box)) 
        {
            if (node.is_leaf) 
            {
                results.add(node.data);
            } 
            else 
            {
                aabb_tree_stack.push(node.child1);
                aabb_tree_stack.push(node.child2);
            }
        }
    }
}

void queryOverlap(const DynamicAABBTree& tree, const Ray& ray, QueryOverlapResult<DynamicAABBTreeNodeData>& results)
{
    results.clear();

    if (tree.root_idx == nullAABBTreeIndex) return;

    aabb_tree_stack.clear();
    aabb_tree_stack.push(tree.root_idx);

    while (!aabb_tree_stack.empty()) 
    {
        int node_idx = aabb_tree_stack.pop();
        if (node_idx == nullAABBTreeIndex) continue;

        const DynamicAABBTreeNode& node = tree.nodes[node_idx];

        RayCollisionResult rc_result = computeRayAABBCollision(ray, node.box);

        if (!rc_result.is_colliding) continue;
        
        if (node.is_leaf) 
        {
            results.add(node.data);
        } 
        else 
        {
            aabb_tree_stack.push(node.child1);
            aabb_tree_stack.push(node.child2);
        }
    }
}

// =======================================================
// Collision Detection
// =======================================================

// =======================================================
// AARamp Helper
// =======================================================

static constexpr int AARampWedgeIndices[3*2] = {0, 1, 2,  0, 1, 3};

inline std::array<Real3, 4> computeAARampWedgeVertices(const AARamp& aaramp)
{
    std::array<Real3, 4> vs;

    Real3 p0(aaramp.minv.x, aaramp.minv.y, aaramp.minv.z);
    Real3 p1(aaramp.maxv.x, aaramp.minv.y, aaramp.minv.z);
    Real3 p2(aaramp.maxv.x, aaramp.maxv.y, aaramp.minv.z);
    Real3 p3(aaramp.minv.x, aaramp.maxv.y, aaramp.minv.z);
    Real3 p4(aaramp.minv.x, aaramp.minv.y, aaramp.maxv.z);
    Real3 p5(aaramp.maxv.x, aaramp.minv.y, aaramp.maxv.z);
    Real3 p6(aaramp.maxv.x, aaramp.maxv.y, aaramp.maxv.z);
    Real3 p7(aaramp.minv.x, aaramp.maxv.y, aaramp.maxv.z);

    switch(aaramp.dir) 
    {
        case AARampDirection::PosX: vs[0] = p5; vs[1] = p3; vs[2] = p0; vs[3] = p6; break;
        case AARampDirection::NegX: vs[0] = p4; vs[1] = p2; vs[2] = p7; vs[3] = p1; break;
        case AARampDirection::PosY: vs[0] = p1; vs[1] = p7; vs[2] = p0; vs[3] = p6; break;
        case AARampDirection::NegY: vs[0] = p3; vs[1] = p5; vs[2] = p4; vs[3] = p2; break;
        default: ASSERT(false, "Unknown AARampDirection");
    }

    return vs;
}

inline Real getAARampPointHeight(const AARamp& aaramp, Real2 point)
{
    Real height = aaramp.maxv.z - aaramp.minv.z;
    Real2 diff  = Real2(aaramp.maxv.x - aaramp.minv.x, aaramp.maxv.y - aaramp.minv.y);
    Real2 tv    = (point - Real2(aaramp.minv.x, aaramp.minv.y)) / diff;

    float t;
    switch(aaramp.dir) 
    {
        case AARampDirection::PosX: t = tv.x; break;
        case AARampDirection::PosY: t = tv.y; break;
        case AARampDirection::NegX: t = 1.0f - tv.x; break;
        case AARampDirection::NegY: t = 1.0f - tv.y; break;
    }

    return aaramp.minv.z + height * glm::clamp(t, 0.0f, 1.0f);
}

// =======================================================
// AA4SidedPyramid Helper
// =======================================================

static constexpr int AA4SidedPyramidSidesIndices[3 * 4] = {
    0, 1, 4, 
    1, 2, 4, 
    2, 3, 4, 
    3, 0, 4
};

inline std::array<Real3, 5> computeAA4SidedPyramidVertices(const AA4SidedPyramid& pyr)
{
    auto [up, u, v] = getAxes(pyr.axis);
    std::array<Real3, 5> vertices;

    vertices[0] = pyr.base_center;
    vertices[0][u] -= pyr.half_extents.x; 
    vertices[0][v] -= pyr.half_extents.y;

    vertices[1] = pyr.base_center;
    vertices[1][u] += pyr.half_extents.x; 
    vertices[1][v] -= pyr.half_extents.y;

    vertices[2] = pyr.base_center;
    vertices[2][u] += pyr.half_extents.x; 
    vertices[2][v] += pyr.half_extents.y;

    vertices[3] = pyr.base_center;
    vertices[3][u] -= pyr.half_extents.x; 
    vertices[3][v] += pyr.half_extents.y;

    vertices[4] = pyr.base_center;
    vertices[4][up] += pyr.height;

    return vertices;
}

// ==============================================
// AATetrahedron Helper
// ==============================================

inline std::array<Real3, 4> calculateAATetrahedronVertices(const AATetrahedron& tetra)
{
    std::array<Real3, 4> p; 
    p.fill(tetra.corner);

    p[1].x += tetra.extents.x;
    p[2].y += tetra.extents.y;
    p[3].z += tetra.extents.z;

    return p;
}

static constexpr std::array<int, 4*3> AATetrahedronFacesIndices = { 0, 1, 2,  0, 1, 3,  0, 2, 3,  1, 2, 3 };

// =======================================================
// End Helper
// =======================================================

// =======================================================
// AABB AACapsule first collision functions
// =======================================================

Real3 closestPointPointSegment(Real3 p, const Segment& seg) 
{
    Real3 ab = seg.end - seg.start;
    Real3 ap = p - seg.start;
    
    Real t     = glm::dot(ap, ab);
    Real denom = glm::dot(ab, ab);

    Real factor = std::clamp(t / std::max(denom, Epsilon), 0.0f, 1.0f);

    return seg.start + factor * ab;
}

Segment getAACapsuleSegment(const AACapsule& aacaps)
{
    const int i  = (int)aacaps.axis;
    const Real h = aacaps.height;

    Real3 start = aacaps.start;
    Real3 end   = aacaps.start;

    Real val1 = start[i];
    Real val2 = start[i] + h;

    start[i] = std::min(val1, val2);
    end[i]   = std::max(val1, val2);

    return {start, end};
}

void rotate90AroundAxis(Real3& v, Axis axis)
{
    static constexpr int idx1[] = { 1, 2, 0 }; 
    static constexpr int idx2[] = { 2, 0, 1 };

    const int i = (int)axis;
    const int a = idx1[i];
    const int b = idx2[i];

    const Real tmp = v[a];
    v[a] = v[b];
    v[b] = -tmp;
}

// =======================================================

inline Range projectAABB(const Real3& center, const Real3& extents, const Real3& axis)
{
    Real p_center = glm::dot(center, axis);
    Real r = extents.x * std::abs(axis.x) + 
             extents.y * std::abs(axis.y) + 
             extents.z * std::abs(axis.z);
    return { p_center - r, p_center + r };
}

template<std::size_t N>
inline Range projectPoints(const std::array<Real3, N> points, const Real3& axis)
{
    Real min_proj = glm::dot(points[0], axis);
    Real max_proj = min_proj;

    for (int i = 1; i < N; ++i) 
    {
        Real proj = glm::dot(points[i], axis);
        min_proj  = std::min(min_proj, proj);
        max_proj  = std::max(max_proj, proj);
    }

    return { min_proj, max_proj };
}

template<std::size_t N>
inline CollisionResult resolveSAT(
    const std::array<Real3, N>& axes, 
    const std::array<Range, N>& ranges1, 
    const std::array<Range, N>& ranges2)
{
    Real min_push = std::numeric_limits<Real>::max();
    int min_axis  = -1;

    for (int i = 0; i < N; ++i) 
    {
        if (ranges1[i].max < ranges2[i].min || ranges2[i].max < ranges1[i].min) return {false};

        Real pushRight = ranges2[i].max - ranges1[i].min; 
        Real pushLeft  = ranges2[i].min - ranges1[i].max;

        Real push = (std::abs(pushLeft) < std::abs(pushRight)) ? pushLeft : pushRight;

        if (std::abs(push) < std::abs(min_push)) 
        {
            min_push = push;
            min_axis = i;
        }
    }

    return CollisionResult{true, min_push * axes[min_axis]};
}


#define NullComputeAABBCollisionDefinition(type_name)                                         \
CollisionResult computeAABB ##type_name ##Collision(const AABB& aabb, const type_name& other) \
{ ASSERT(false, "UNREACHABLE: ComputeAABBCollision unavailable for " #type_name); }           \

NullComputeAABBCollisionDefinition(AACapsule);
NullComputeAABBCollisionDefinition(OBB);
NullComputeAABBCollisionDefinition(Segment);
NullComputeAABBCollisionDefinition(Ray);
NullComputeAABBCollisionDefinition(Capsule);
NullComputeAABBCollisionDefinition(Sphere);

CollisionResult computeAABBAABBCollision(const AABB& aabb1, const AABB& aabb2) 
{
    if (aabb1.maxv.x < aabb2.minv.x || aabb1.minv.x > aabb2.maxv.x) return {false};
    if (aabb1.maxv.y < aabb2.minv.y || aabb1.minv.y > aabb2.maxv.y) return {false};
    if (aabb1.maxv.z < aabb2.minv.z || aabb1.minv.z > aabb2.maxv.z) return {false};

    Real3 center1 = (aabb1.minv + aabb1.maxv) * 0.5f;
    Real3 center2 = (aabb2.minv + aabb2.maxv) * 0.5f;
    Real3 half1   = (aabb1.maxv - aabb1.minv) * 0.5f;
    Real3 half2   = (aabb2.maxv - aabb2.minv) * 0.5f;

    Real3 delta   = center2 - center1;
    Real3 overlap = (half1 + half2) - glm::abs(delta);

    Real3 mtv;

    if (overlap.x < overlap.y && overlap.x < overlap.z)
        mtv = Real3((delta.x > 0 ? -overlap.x : overlap.x), 0.0f, 0.0f);
    else if (overlap.y < overlap.z) 
        mtv = Real3(0.0f, (delta.y > 0 ? -overlap.y : overlap.y), 0.0f);
    else 
        mtv = Real3(0.0f, 0.0f, (delta.z > 0 ? -overlap.z : overlap.z));

    return {true, mtv};
}

CollisionResult computeAABBAACylinderCollision(const AABB& aabb, const AACylinder& aacyl)
{
    CollisionResult result;
    result.is_colliding = false;

    Axis axis    = aacyl.axis;
    int axis_idx = (int)axis;

    Real min_a = rmin(aacyl.base_center[axis_idx], aacyl.base_center[axis_idx] + aacyl.height);
    Real max_a = rmax(aacyl.base_center[axis_idx], aacyl.base_center[axis_idx] + aacyl.height);

    if (aabb.minv[axis_idx] >= max_a) return result;
    if (aabb.maxv[axis_idx] <= min_a) return result;

    Real2 rec_min = project(aabb.minv, axis);
    Real2 rec_max = project(aabb.maxv, axis);

    Real2 circ_center = project(aacyl.base_center, axis);
    Real  circ_radius = aacyl.radius;

    Real dx = std::max(0.0f, std::max(rec_min.x - circ_center.x, circ_center.x - rec_max.x));
    Real dy = std::max(0.0f, std::max(rec_min.y - circ_center.y, circ_center.y - rec_max.y));

    Real dsqrd = (dx * dx + dy * dy);

    if (dsqrd > aacyl.radius*aacyl.radius) return result;

    result.is_colliding = true;

    Real dist = sqrt(dsqrd);
    Real LB_collision_depth = aacyl.radius - dist;

    Real AA_collision_resolution = 0.0f;

    Real min_aabb = aabb.minv[axis_idx];
    Real max_aabb = aabb.maxv[axis_idx];

    bool min_between = min_aabb > min_a && min_aabb < max_a;
    bool max_between = max_aabb > min_a && max_aabb < max_a;

    if (min_between && !max_between)
    {
        AA_collision_resolution = max_a - min_aabb;
    }
    else if (!min_between && max_between)
    {
        AA_collision_resolution = min_a - max_aabb;
    }
    else
    {
        Real max_cyl_min_aabb = max_a - min_aabb;
        Real min_cyl_max_aabb = min_a - max_aabb;

        AA_collision_resolution = (max_cyl_min_aabb < -min_cyl_max_aabb) ? max_cyl_min_aabb : min_cyl_max_aabb;
    }

    if (LB_collision_depth > abs(AA_collision_resolution))
    {
        Real3 mtv     = Real3(0.0f);
        mtv[axis_idx] = AA_collision_resolution;
        result.mtv    = mtv;
        return result;
    }

    // TODO: aacylinder aabb collision: probably could optimize by resolving on only one axis
    Real2 closest_p;
    closest_p.x = glm::max(rec_min.x, glm::min(circ_center.x, rec_max.x));
    closest_p.y = glm::max(rec_min.y, glm::min(circ_center.y, rec_max.y));

    Real2 diff = closest_p - circ_center;

    if (dsqrd != 0.0f) [[likely]]
    {
        diff = glm::normalize(diff) * (aacyl.radius - dist);
        result.mtv = unproject(diff, axis);
        return result;
    }

    ASSERT(false, "case where circle center is inside rectangle: should never happen");
}

CollisionResult computeAABBAARectangleCollision(const AABB& aabb, const AARectangle& aarect)
{
    CollisionResult result;
    result.is_colliding = false;

    Axis axis = aarect.axis;
    int ax    = (int) axis;

    Real h = std::abs(aarect.p1[ax]-aarect.p2[ax]);

    Real2 p1 = project(aarect.p1, axis);
    Real2 p2 = project(aarect.p2, axis);

    Real3 axis1 = unproject(glm::normalize(p1-p2), axis);
    Real3 axis2 = axis1;
    rotate90AroundAxis(axis2, axis);

    Real3 aabb_center  = (aabb.maxv + aabb.minv) * 0.5f;
    Real3 aabb_extents = (aabb.maxv - aabb.minv) * 0.5f;

    Range aabb_ax1 = projectAABB(aabb_center, aabb_extents, axis1);
    Range aabb_ax2 = projectAABB(aabb_center, aabb_extents, axis2);
    Range aabb_ax3 = {aabb.minv[ax], aabb.maxv[ax]};

    Range aarect_ax1 = { glm::dot(axis1, aarect.p1), glm::dot(axis1, aarect.p2) };
    Range aarect_ax2 = { glm::dot(axis2, aarect.p1), glm::dot(axis2, aarect.p2) };
    Range aarect_ax3 = { aarect.p1[ax], aarect.p2[ax]};

    static auto rangeOverlap = [](const Range& range1, const Range& range2) -> bool
    {
        Real min1 = std::min(range1.min, range1.max);
        Real max1 = std::max(range1.min, range1.max);
        Real min2 = std::min(range2.min, range2.max);
        Real max2 = std::max(range2.min, range2.max);
        return min1 <= max2 && max1 >= min2;
    };

    if (rangeOverlap(aabb_ax1, aarect_ax1) && 
        rangeOverlap(aabb_ax2, aarect_ax2) && 
        rangeOverlap(aabb_ax3, aarect_ax3))
    {
        result.is_colliding = true;

        Real v     = aarect_ax2.min;
        Real min_a = aabb_ax2.min; 
        Real max_a = aabb_ax2.max;

        if (std::abs(v-min_a) < std::abs(v-max_a)) result.mtv = axis2 * (v-min_a);
        else                                       result.mtv = axis2 * (v-max_a);
    }

    return result;
}

CollisionResult computeAABBAxialOBBCollision(const AABB& aabb, const AxialOBB& axialobb)
{
    static auto getRangeMTV = [](const std::array<Real, 2>& r1, const std::array<Real, 2>& r2) -> Real
    {
        if (r1[0] > r2[1] || r2[0] > r1[1]) return 0.0f;

        Real pushRight = r2[1] - r1[0]; 
        Real pushLeft  = r2[0] - r1[1];

        return (std::abs(pushLeft) < std::abs(pushRight)) ? pushLeft : pushRight;
    };

    static auto rangeOverlap = [](const std::array<Real, 2>& range1, const std::array<Real, 2>& range2) -> bool
    {
        return range1[0] <= range2[1] && range1[1] >= range2[0];
    };

    static auto project2DAABB = [](const Real2& center, const Real2& extents, const Real2& axis) -> std::array<Real, 2> 
    {
        Real p_center = glm::dot(center, axis);
        Real r = extents.x * std::abs(axis.x) + extents.y * std::abs(axis.y);
        return { p_center - r, p_center + r };
    };

    CollisionResult result;
    result.is_colliding = false;

    Axis axis = axialobb.rotation_axis;
    int axis_idx = (int) axis;

    Real3 rot_axis(0.0f);
    rot_axis[axis_idx] = 1.0f;

    std::array<Real, 2> aabb_rotaxis_rng     = { aabb.minv[axis_idx], aabb.maxv[axis_idx] };
    std::array<Real, 2> axialobb_rotaxis_rng = { axialobb.center[axis_idx] - axialobb.extents[axis_idx], 
                                                 axialobb.center[axis_idx] + axialobb.extents[axis_idx] };
    if (rangeOverlap(aabb_rotaxis_rng, axialobb_rotaxis_rng) == false) return result;

    Real angle = axialobb.angle;

    Real2 proj_aabb_minv    = project(aabb.minv, axis);
    Real2 proj_aabb_maxv    = project(aabb.maxv, axis);
    Real2 proj_aabb_center  = (proj_aabb_maxv + proj_aabb_minv) * 0.5f;
    Real2 proj_aabb_extents = (proj_aabb_maxv - proj_aabb_minv) * 0.5f;

    std::array<Real2, 2> rotated_proj_axis;

    rotated_proj_axis[0] = Real2(std::cos(angle), std::sin(angle));
    rotated_proj_axis[1] = Real2(std::cos(angle + glm::half_pi<Real>()), std::sin(angle + glm::half_pi<Real>()));

    std::array<Real, 2> aabb_axis1 = {proj_aabb_minv.x, proj_aabb_maxv.x}; 
    std::array<Real, 2> aabb_axis2 = {proj_aabb_minv.y, proj_aabb_maxv.y}; 
    std::array<Real, 2> aabb_axis3 = project2DAABB(proj_aabb_center, proj_aabb_extents, rotated_proj_axis[0]);
    std::array<Real, 2> aabb_axis4 = project2DAABB(proj_aabb_center, proj_aabb_extents, rotated_proj_axis[1]);

    Real2 proj_axial_center  = project(axialobb.center, axis);
    Real2 proj_axial_extents = project(axialobb.extents, axis);

    Real2 rotated_proj_axial_center = Real2(proj_axial_center.x * cos(angle) - proj_axial_center.y * sin(angle), 
                                            proj_axial_center.x * sin(angle) + proj_axial_center.y * cos(angle));

    std::array<Real, 2> axial_obb_axis1 = project2DAABB(rotated_proj_axial_center, proj_axial_extents, rotated_proj_axis[0]);
    std::array<Real, 2> axial_obb_axis2 = project2DAABB(rotated_proj_axial_center, proj_axial_extents, rotated_proj_axis[1]);
    std::array<Real, 2> axial_obb_axis3 = {glm::dot(proj_axial_center, rotated_proj_axis[0]) - proj_axial_extents.x, 
                                           glm::dot(proj_axial_center, rotated_proj_axis[0]) + proj_axial_extents.x};
    std::array<Real, 2> axial_obb_axis4 = {glm::dot(proj_axial_center, rotated_proj_axis[1]) - proj_axial_extents.y, 
                                           glm::dot(proj_axial_center, rotated_proj_axis[1]) + proj_axial_extents.y};

    std::array<Real, 5> depths = {
        getRangeMTV(aabb_rotaxis_rng, axialobb_rotaxis_rng), 
        getRangeMTV(aabb_axis1, axial_obb_axis1),
        getRangeMTV(aabb_axis2, axial_obb_axis2),
        getRangeMTV(aabb_axis3, axial_obb_axis3),
        getRangeMTV(aabb_axis4, axial_obb_axis4)};

    std::array<Real3, 5> sep_axis = {
        rot_axis, 
        unproject(Real2(1.0f, 0.0f),    axis), 
        unproject(Real2(0.0f, 1.0f),    axis),
        unproject(rotated_proj_axis[0], axis),
        unproject(rotated_proj_axis[1], axis),
    };

    static auto findAbsoluteMin = [](std::array<Real, 5> vals) -> int
    {
        Real minv = std::abs(vals[0]);
        int index = 0;

        for (int i=1; i<5; i++)
        {
            if (std::abs(vals[i]) < minv) 
            {
                minv  = std::abs(vals[i]);
                index = i;
            }
        }

        return index;
    };

    if (depths[1] != 0.0f && depths[2] != 0.0f && depths[3] != 0.0f && depths[4] != 0.0f)
    {
        int index = findAbsoluteMin(depths);

        result.is_colliding = true;
        result.mtv = sep_axis[index] * depths[index];
    }   

    return result;
}

CollisionResult computeAABBAA4SidedPyramidCollision(const AABB& aabb, const AA4SidedPyramid& aa4pyr)
{
    auto [up, u, v] = getAxes(aa4pyr.axis);

    std::array<Real3, 5> p;
    p[0] = p[1] = p[2] = p[3] = p[4] = aa4pyr.base_center;
    p[0][u]  -= aa4pyr.half_extents.x; p[0][v] -= aa4pyr.half_extents.y;
    p[1][u]  += aa4pyr.half_extents.x; p[1][v] -= aa4pyr.half_extents.y;
    p[2][u]  += aa4pyr.half_extents.x; p[2][v] += aa4pyr.half_extents.y;
    p[3][u]  -= aa4pyr.half_extents.x; p[3][v] += aa4pyr.half_extents.y;
    p[4][up] += aa4pyr.height;

    Real3 pyr_edges[4] = { p[0]-p[4], p[1]-p[4], p[2]-p[4], p[3]-p[4] };

    std::array<Real3, 9> axes = {
        GlobalAxes[0],
        GlobalAxes[1],
        GlobalAxes[2],
        glm::normalize(glm::cross(GlobalAxes[u],  pyr_edges[0])), // normal of the faces
        glm::normalize(glm::cross(GlobalAxes[v],  pyr_edges[1])), // normal of the faces
        glm::normalize(glm::cross(GlobalAxes[u],  pyr_edges[2])), // normal of the faces
        glm::normalize(glm::cross(GlobalAxes[v],  pyr_edges[3])), // normal of the faces
        glm::normalize(glm::cross(GlobalAxes[up], pyr_edges[0])), // normal cross edges 
        glm::normalize(glm::cross(GlobalAxes[up], pyr_edges[1]))  // normal cross edges
    };

    std::array<Range, 9> aa4pyr_ranges;

    const Real3& bc = aa4pyr.base_center;

    auto [min_up, max_up] = std::minmax(bc[up], p[4][up]);
    aa4pyr_ranges[up] = {min_up, max_up};

    aa4pyr_ranges[u] = {
        bc[u] - aa4pyr.half_extents.x,
        bc[u] + aa4pyr.half_extents.x
    };

    aa4pyr_ranges[v] = {
        bc[v] - aa4pyr.half_extents.y,
        bc[v] + aa4pyr.half_extents.y
    };

    aa4pyr_ranges[3] = projectPoints(p, axes[3]);
    aa4pyr_ranges[4] = projectPoints(p, axes[4]);
    aa4pyr_ranges[5] = projectPoints(p, axes[5]);
    aa4pyr_ranges[6] = projectPoints(p, axes[6]);
    aa4pyr_ranges[7] = projectPoints(p, axes[7]);
    aa4pyr_ranges[8] = projectPoints(p, axes[8]);

    // =======================================================

    Real3 aabb_center  = (aabb.maxv + aabb.minv) * 0.5f;
    Real3 aabb_extents = (aabb.maxv - aabb.minv) * 0.5f;

    std::array<Range, 9> aabb_ranges = {{
        {aabb.minv[0], aabb.maxv[0]},
        {aabb.minv[1], aabb.maxv[1]},
        {aabb.minv[2], aabb.maxv[2]}, 
        projectAABB(aabb_center, aabb_extents, axes[3]), 
        projectAABB(aabb_center, aabb_extents, axes[4]),
        projectAABB(aabb_center, aabb_extents, axes[5]),
        projectAABB(aabb_center, aabb_extents, axes[6]),
        projectAABB(aabb_center, aabb_extents, axes[7]),
        projectAABB(aabb_center, aabb_extents, axes[8])
    }};

    return resolveSAT(axes, aabb_ranges, aa4pyr_ranges);
}

CollisionResult computeAABBAARampCollision(const AABB& aabb, const AARamp& ramp) 
{
    ASSERT(false, "TODO: CollisionResult computeAABBAARampCollision(const AABB& aabb, const AARamp& ramp)");
}


CollisionResult computeAABBShapeCollision(const AABB& aabb, const CollisionShape& shape)
{
    switch (shape.type) 
    {
        #define X(type_name, union_name) case CollisionShapeType::type_name: return computeAABB ##type_name ##Collision(aabb, shape.union_name);
        COLLISION_TYPES
        #undef X

        default: ASSERT(false, "Unknown CollisionShapeType");
    }
}

// =======================================================

#define DefaultComputeAACapsuleCollisionDefinition(type_name)                                           \
CollisionResult computeAACapsule ##type_name ##Collision(const AACapsule& caps, const type_name& other) \
{ return computeAABB ##type_name ##Collision(computeAACapsuleAABB(caps), other); }                      \

#define NullComputeAACapsuleCollisionDefinition(type_name)                                              \
CollisionResult computeAACapsule ##type_name ##Collision(const AACapsule& caps, const type_name& other) \
{ ASSERT(false, "UNREACHABLE: ComputeAACapsuleCollision unavailable for " #type_name); }                \


NullComputeAACapsuleCollisionDefinition(OBB);
NullComputeAACapsuleCollisionDefinition(Segment);
NullComputeAACapsuleCollisionDefinition(Ray);

DefaultComputeAACapsuleCollisionDefinition(AARamp);
DefaultComputeAACapsuleCollisionDefinition(AA4SidedPyramid);
DefaultComputeAACapsuleCollisionDefinition(AxialOBB);
DefaultComputeAACapsuleCollisionDefinition(AARectangle);
DefaultComputeAACapsuleCollisionDefinition(AACylinder);
DefaultComputeAACapsuleCollisionDefinition(AABB);

CollisionResult computeAACapsuleAACapsuleCollision(const AACapsule& aacaps1, const AACapsule& aacaps2)
{
    Segment seg1 = getAACapsuleSegment(aacaps1);
    Segment seg2 = getAACapsuleSegment(aacaps2);

    Axis ax1 = aacaps1.axis;
    Axis ax2 = aacaps2.axis;

    Real3 p1, p2;

    if (ax1 == ax2)
    {
        int i = (int)ax1;

        Real overlap_start = std::max(seg1.start[i], seg2.start[i]);
        Real overlap_end   = std::min(seg1.end[i],   seg2.end[i]);

        Real best_i = (overlap_start + overlap_end) * 0.5f;

        p1 = seg1.start;
        p2 = seg2.start;

        p1[i] = std::clamp(best_i, seg1.start[i], seg1.end[i]);
        p2[i] = std::clamp(best_i, seg2.start[i], seg2.end[i]);
    }
    else 
    {
        p1 = seg1.start;
        p2 = seg2.start;

        int i1 = (int)ax1;
        int i2 = (int)ax2;

        p1[i1] = std::clamp(seg2.start[i1], seg1.start[i1], seg1.end[i1]);
        p2[i2] = std::clamp(seg1.start[i2], seg2.start[i2], seg2.end[i2]);
    }

    Real squared_dist   = glm::dot(p2-p1, p2-p1);
    Real squared_radius = (aacaps1.radius + aacaps2.radius) * (aacaps1.radius + aacaps2.radius);

    if (squared_dist > squared_radius) return {false};

    if (squared_dist < Epsilon) [[unlikely]] { ASSERT(false, "should do something special for overlapping collision points"); }

    Real length      = std::sqrt(squared_dist);
    Real penetration = (aacaps1.radius + aacaps2.radius) - length;

    Real3 mtv = ((p1-p2) / length) * penetration;

    return {true, mtv};
}

CollisionResult computeAACapsuleCapsuleCollision(const AACapsule& aacaps, const Capsule& capsule)
{
    CollisionResult result;
    result.is_colliding = false;

    Axis axis    = aacaps.axis;
    int axis_idx = (int) axis;

    Real min_a = aacaps.start[axis_idx];
    Real max_a = min_a + aacaps.height;
    if (min_a > max_a) std::swap(min_a, max_a);

    Real2 ori   = project(aacaps.start, axis);

    Real2 start = project(capsule.start, axis) - ori;
    Real2 end   = project(capsule.end, axis)   - ori;

    Real2 AB = end - start;
    Real length_sq_2D = glm::dot(AB, AB);

    if (length_sq_2D < Epsilon) [[unlikely]] ASSERT(false, "impossible");

    Real t = glm::dot(-start, AB) / length_sq_2D;
    t      = std::clamp(t, 0.0f, 1.0f);

    Real3 p_capsule = (capsule.end - capsule.start) * t + capsule.start;

    Real t_aacapsule = (max_a - p_capsule[axis_idx]) / (max_a - min_a);
    t_aacapsule      = std::clamp(t_aacapsule, 0.0f, 1.0f);

    Real3 p_aacapsule     = unproject(ori, axis);
    p_aacapsule[axis_idx] = min_a + (max_a - min_a) * t_aacapsule;

    Real length_sq = glm::dot(p_capsule - p_aacapsule, p_capsule - p_aacapsule);

    Real rad_sq = (aacaps.radius + capsule.radius) * (aacaps.radius + capsule.radius);

    if (length_sq > rad_sq) return result;

    result.is_colliding = true;

    Real length = sqrt(length_sq);
    Real3 dir   = (p_aacapsule - p_capsule) / length;
    Real depth  = (aacaps.radius + capsule.radius) - length;

    result.mtv = dir * depth;

    return result;
}

CollisionResult computeAACapsuleSphereCollision(const AACapsule& aacaps, const Sphere& sphere)
{
    CollisionResult result;
    result.is_colliding = false;

    // EXPLANATION: basically project sphere center on the axis whose capsule segment is aligned 
    //              and then clamp between min and max value

    const int i = (int)aacaps.axis;

    Real min_a = std::min(aacaps.start[i], aacaps.start[i] + aacaps.height);
    Real max_a = std::max(aacaps.start[i], aacaps.start[i] + aacaps.height);

    Real center_a = sphere.center[i];

    Real3 p = aacaps.start;
    p[i] = std::clamp(center_a, min_a, max_a);
    
    Real3 delta     = p-sphere.center;
    Real dist_sq    = glm::dot(delta, delta);
    Real cum_radius = (aacaps.radius + sphere.radius);
    Real radius_sq  = cum_radius * cum_radius;

    if (dist_sq > radius_sq) return result;

    result.is_colliding = true;

    Real length = sqrt(dist_sq);

    ASSERT(length > Epsilon, "Impossible shoud not happen");

    result.mtv = (delta / length) * (cum_radius - length);

    return result;
}


CollisionResult computeAACapsuleShapeCollision(const AACapsule& aacaps, const CollisionShape& shape)
{
    switch (shape.type) 
    {
        #define X(type_name, union_name) case CollisionShapeType::type_name: return computeAACapsule ##type_name ##Collision(aacaps, shape.union_name);
        COLLISION_TYPES
        #undef X
        default: ASSERT(false, "Unknown CollisionShapeType");
    }
}

// =======================================================

bool testRectangleCircleCollision(const Real2 min, const Real2 max, const Real2 center, const Real radius)
{
    Real dx = std::max(0.0f, std::max(min.x - center.x, center.x - max.x));
    Real dy = std::max(0.0f, std::max(min.y - center.y, center.y - max.y));
    return (dx * dx + dy * dy) < (radius * radius);
}

#define NullTestAABBCollisionDefinition(type_name)                               \
bool testAABB ##type_name ##Collision(const AABB& aabb, const type_name& other)  \
{ ASSERT(false, "UNREACHABLE: testAABBShapeCollision unavailable for " #type_name); } \

NullTestAABBCollisionDefinition(OBB);
NullTestAABBCollisionDefinition(Capsule);
NullTestAABBCollisionDefinition(AARectangle);
NullTestAABBCollisionDefinition(AACapsule);
NullTestAABBCollisionDefinition(AxialOBB);
NullTestAABBCollisionDefinition(AA4SidedPyramid);

bool testAABBAABBCollision(const AABB& aabb1, const AABB& aabb2)
{
    return AABBOverlap(aabb1, aabb2);
}

bool testAABBAARampCollision(const AABB& aabb, const AARamp& aaramp) 
{
    return AABBOverlap(aabb, AABB{aaramp.minv, aaramp.maxv});
}

bool testAABBSphereCollision(const AABB& aabb, const Sphere& sphere)
{
    Real3 closest    = glm::clamp(sphere.center, aabb.minv, aabb.maxv);
    Real3 diff       = sphere.center - closest;
    Real distance_sq = glm::dot(diff, diff);

    return distance_sq <= (sphere.radius * sphere.radius);
}

bool testAABBAACylinderCollision(const AABB& aabb, const AACylinder& aacyl)
{
    ASSERT(aacyl.radius >= 0.0f, "AACylinder radius must be positive");

    Axis axis    = aacyl.axis;
    int axis_idx = (int)axis;

    Real min_a = rmin(aacyl.base_center[axis_idx], aacyl.base_center[axis_idx] + aacyl.height);
    Real max_a = rmax(aacyl.base_center[axis_idx], aacyl.base_center[axis_idx] + aacyl.height);

    if (aabb.minv[axis_idx] >= max_a) return false;
    if (aabb.maxv[axis_idx] <= min_a) return false;

    Real2 rec_min = project(aabb.minv, axis);
    Real2 rec_max = project(aabb.maxv, axis);

    Real2 circ_center = project(aacyl.base_center, axis);
    Real  circ_radius = aacyl.radius;

    return testRectangleCircleCollision(rec_min, rec_max, circ_center, circ_radius);
}

bool testAABBRayCollision(const AABB& aabb, const Ray& ray)
{
    return computeAABBRayCollision(aabb, ray).is_colliding;
}

bool testAABBSegmentCollision(const AABB& aabb, const Segment& seg)
{
    return computeAABBSegmentCollision(aabb, seg).is_colliding;
}


bool testAABBShapeCollision(const AABB& aabb, const CollisionShape& shape) 
{
    switch (shape.type) 
    {
        #define X(type_name, union_name) case CollisionShapeType::type_name: return testAABB ##type_name ##Collision(aabb, shape.union_name);
        COLLISION_TYPES
        #undef X

        default: ASSERT(false, "Unknown CollisionShapeType");
    }
    return false;
}

// =======================================================
// RAY/SEGMENT - TRIANGLE COLLISION
// =======================================================

bool checkRayTriCollision(const Ray& ray, const Real3& v0, const Real3& v1, const Real3& v2, float& out_t)
{
    constexpr Real EPS = Epsilon; 

    Real3 edge1 = v1 - v0;
    Real3 edge2 = v2 - v0;
    Real3 h     = glm::cross(ray.direction, edge2);
    Real det    = glm::dot(edge1, h);

    if (std::abs(det) < EPS) return false; 

    Real3 s = ray.origin - v0;
    Real u  = glm::dot(s, h);

    if (det > 0.0f) 
    {
        if (u < 0.0f || u > det) return false;
    } 
    else 
    {
        if (u > 0.0f || u < det) return false;
    }

    Real3 q = glm::cross(s, edge1);
    Real v  = glm::dot(ray.direction, q);

    if (det > 0.0f) 
    {
        if (v < 0.0f || u + v > det) return false;
    } 
    else 
    {
        if (v > 0.0f || u + v < det) return false;
    }

    Real inv_det = 1.0f / det;
    Real t = inv_det * glm::dot(edge2, q);

    if (t > EPS) 
    {
        out_t = t;
        return true; 
    }

    return false;
}

bool checkRayOrSegmentTrisCollision(const Ray& ray, Real3 *vertices, unsigned int num_tris, Real3& entering_point, bool is_segment) 
{
    Real closest_t = std::numeric_limits<Real>::max(); // 2.0f;
    bool found     = false;

    for (int i = 0; i < num_tris; i++) 
    {
        Real curr_t;
        
        if (checkRayTriCollision(ray, vertices[i*3], vertices[i*3+1], vertices[i*3+2], curr_t))
        {
            if (curr_t > 0.0f && (!is_segment || curr_t <= 1.0f)) 
            {
                if (curr_t < closest_t) 
                {
                    closest_t = curr_t;
                    found = true;
                }
            }
        }
    }

    if (found) entering_point = ray.origin + ray.direction * closest_t;

    return found;
}

bool checkSegmentTrisCollision(const Segment& seg, Real3 *vertices, unsigned int num_tris, Real3& entering_point)
{
    Ray seg_ray = {seg.start, seg.end - seg.start};
    return checkRayOrSegmentTrisCollision(seg_ray, vertices, num_tris, entering_point, true);
}

bool checkRayTrisCollision(const Ray& ray, Real3 *vertices, unsigned int num_tris, Real3& entering_point)
{
    return checkRayOrSegmentTrisCollision(ray, vertices, num_tris, entering_point, false);
}


SegmentCollisionResult computeSegmentTrianglesCollision(const Segment& seg, const Real3 *vertices, const int *indices, unsigned int num_tris)
{
    ClosestHit closest_hit;

    Ray ray = {seg.start, seg.end - seg.start};
    Real t;

    for (int i = 0; i < num_tris; i++) 
    {
        const Real3& v0 = vertices[indices[i * 3 + 0]];
        const Real3& v1 = vertices[indices[i * 3 + 1]];
        const Real3& v2 = vertices[indices[i * 3 + 2]];

        if (checkRayTriCollision(ray, v0, v1, v2, t))
            if (t > 0.0f && t <= 1.0f && closest_hit.isCloser(t)) closest_hit.update(t);
    }

    if (closest_hit.hasHit()) return {true, interpolateSegment(seg, closest_hit.getT())}; 

    return {false};
}

RayCollisionResult computeRayTrianglesCollision(const Ray& ray, const Real3 *vertices, const int *indices, unsigned int num_tris)
{
    ClosestHit closest_hit;
    Real3 hit_normal(0.0f);

    Real t;

    for (int i = 0; i < num_tris; i++) 
    {
        const Real3& v0 = vertices[indices[i * 3 + 0]];
        const Real3& v1 = vertices[indices[i * 3 + 1]];
        const Real3& v2 = vertices[indices[i * 3 + 2]];

        if (checkRayTriCollision(ray, v0, v1, v2, t))
        {
            if (t > 0.0f && closest_hit.isCloser(t)) 
            {
                hit_normal = glm::cross(v1 - v0, v2 - v0);
                closest_hit.update(t);
            }
        }
    }

    if (closest_hit.hasHit()) 
    {
        hit_normal = glm::normalize(hit_normal);
        if (glm::dot(hit_normal, ray.direction) > 0.0f) hit_normal = -hit_normal;
        Real t = closest_hit.getT();
        return {true, interpolateRay(ray, t), t, hit_normal};
    }

    return {false};
}


// =======================================================
// END RAY/SEGMENT - TRIANGLE COLLISION
// =======================================================

struct CircleIntersection 
{
    bool is_intersecting;
    Real t;
};

struct SphereIntersection
{
    bool is_intersecting;
    Real t;
};

struct PlaneIntersection 
{
    bool is_intersecting;
    Real t;
};

CircleIntersection compute2DSegmentCircleIntersection(Real2 seg_a, Real2 seg_b, Real2 center, Real radius)
{
    CircleIntersection res;
    res.is_intersecting = false;

    static auto dot2D = [](Real2 a, Real2 b) -> Real 
    {
        return a.x * b.x + a.y * b.y;
    };

    Real2 d = seg_b - seg_a;
    Real2 f = seg_a - center;

    Real a = dot2D(d, d);

    if (a < Epsilon) 
    {
        if (dot2D(f, f) <= radius * radius) 
        {
            res.is_intersecting = true;
            res.t = 0;
        }
        return res;
    }

    Real b = 2.0f * dot2D(f, d);
    Real c = dot2D(f, f) - radius*radius;

    Real discriminant = b * b - 4.0f * a * c;

    if (discriminant < Epsilon) 
    {
        return res; 
    }

    discriminant = sqrt(discriminant);

    Real t1 = (-b - discriminant) / (2.0f * a);
    Real t2 = (-b + discriminant) / (2.0f * a);

    Real t_hit = -1.0f;

    if (t1 >= 0.0f && t1 <= 1.0f) 
    {
        t_hit = t1;
    } 
    else if (t2 >= 0.0f && t2 <= 1.0f) 
    {
        t_hit = t2;
    }

    if (t_hit != -1.0f) 
    {
        res.is_intersecting = true;
        res.t = t_hit;
    }

    return res;
}

PlaneIntersection  computeSegmentPlaneIntersection(const Segment& seg, Real3 plane_normal, Real3 plane_point) 
{
    PlaneIntersection res = { false, 0.0f };

    Real3 ab = seg.end - seg.start;
    
    Real denom = glm::dot(plane_normal, ab);

    if (glm::abs(denom) > Epsilon) 
    {
        Real numer = glm::dot(plane_normal, plane_point - seg.start);
        Real t = numer / denom;

        if (t >= 0.0f && t <= 1.0f) 
        {
            res.is_intersecting = true;
            res.t = t;
        }
    }

    return res;
}

SphereIntersection computeSegmentSphereIntersection(const Segment& seg, Real3 center, Real radius) 
{
    SphereIntersection result = { false, 0.0f };

    Real3 d = seg.end   - seg.start;
    Real3 f = seg.start - center;

    Real a = glm::dot(d, d);
    Real b = 2.0f * glm::dot(f, d);
    Real c = glm::dot(f, f) - (radius * radius);

    if (c <= 0.0f) 
    {
        result.is_intersecting = true;
        result.t = 0;
        return result;
    }

    if (a <= Epsilon) return result;

    Real discriminant = b * b - 4 * a * c;

    if (discriminant < 0) return result;

    discriminant = std::sqrt(discriminant);
    Real t1 = (-b - discriminant) / (2.0f * a);
    Real t2 = (-b + discriminant) / (2.0f * a);

    if (t1 >= 0.0f && t1 <= 1.0f) 
    {
        result.is_intersecting = true;
        result.t = t1;
        return result;
    }

    if (t2 >= 0.0f && t2 <= 1.0f) 
    {
        result.is_intersecting = true;
        result.t = t2;
        return result;
    }

    return result;
}

Real squaredDistancePointAASegment(Real3 p, Real3 base, Real height, Axis axis)
{
    int i   = (int)axis;
    Real a1 = base[i];
    Real a2 = base[i] + height;

    Real min_a = std::min(a1, a2);
    Real max_a = std::max(a1, a2);

    Real clamped_a = std::max(min_a, std::min(p[i], max_a));

    Real3 diff = p - base; 
    diff[i]    = p[i] - clamped_a; 

    return glm::dot(diff, diff);
}

Real3 projectOnPlane(Real3 ori, Real3 n, Real3 p)
{
    return p - glm::dot(n, (p-ori)) * n;
}

Real3 constraintToSegment(Real3 p, Real3 start, Real3 end)
{
    Real3 dir = end-start; 
    Real t = glm::dot(p - start, dir) / glm::dot(dir, dir);
    t      = glm::clamp(t, 0.0f, 1.0f);
    return (dir) * t + start;
}


// =======================================================
// SEGMENT COLLISION
// =======================================================

#define NullComputeSegmentCollisionDefinition(type_name)                                                  \
SegmentCollisionResult computeSegment ##type_name ##Collision(const Segment& seg, const type_name& other) \
{ ASSERT(false, "UNREACHABLE: computeSegmentCollision unavailable for " #type_name); }                    \

NullComputeSegmentCollisionDefinition(OBB);
NullComputeSegmentCollisionDefinition(Segment);
NullComputeSegmentCollisionDefinition(Ray);

SegmentCollisionResult computeSegmentAARampCollision(const Segment& seg, const AARamp& aaramp) 
{
    AABB ramp_aabb = computeAARampAABB(aaramp);  
    auto aabb_res  = computeSegmentAABBCollision(seg, ramp_aabb);

    if (!aabb_res.is_colliding) return {false};

    Real2 proj(aabb_res.entering_point.x, aabb_res.entering_point.y);

    Real slope_z = getAARampPointHeight(aaramp, proj);

    if (aabb_res.entering_point.z <= slope_z) return aabb_res;

    std::array<Real3, 4> vertices = computeAARampWedgeVertices(aaramp);

    return computeSegmentTrianglesCollision(seg, vertices.data(), AARampWedgeIndices, 2);
}

SegmentCollisionResult computeSegmentSphereCollision(const Segment& seg, const Sphere& sphere)
{
    Real3 d = seg.end   - seg.start;
    Real3 f = seg.start - sphere.center;

    Real a = glm::dot(d, d);
    Real b = 2.0f * glm::dot(f, d);
    Real c = glm::dot(f, f) - (sphere.radius * sphere.radius);

    if (a < Epsilon) [[unlikely]] return {false};
    if (c <= 0.0f)   [[unlikely]] return {true, seg.start};

    Real discriminant = b * b - 4 * a * c;

    if (discriminant < 0) return {false};

    discriminant = std::sqrt(discriminant);

    Real t1 = (-b - discriminant) / (2.0f * a);

    if (t1 >= 0.0f && t1 <= 1.0f) return {true, seg.start + t1 * d};

    Real t2 = (-b + discriminant) / (2.0f * a);

    if (t2 >= 0.0f && t2 <= 1.0f) return {true, seg.start + t2 * d};

    return {false};
}

SegmentCollisionResult computeSegmentAABBCollision(const Segment& seg, const AABB& aabb) 
{
    // start inside
    if (seg.start.x >= aabb.minv.x && seg.start.x <= aabb.maxv.x &&
        seg.start.y >= aabb.minv.y && seg.start.y <= aabb.maxv.y &&
        seg.start.z >= aabb.minv.z && seg.start.z <= aabb.maxv.z)
        return {true, seg.start};

    Real3 dir = seg.end - seg.start;
    Real tmin = 0.0f;
    Real tmax = 1.0f;

    for (int i = 0; i < 3; ++i) 
    {
        if (std::abs(dir[i]) >= Epsilon) 
        {
            Real ood = 1.0f / dir[i];
            Real t1 = (aabb.minv[i] - seg.start[i]) * ood;
            Real t2 = (aabb.maxv[i] - seg.start[i]) * ood;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return {false};
        } 
        else if (seg.start[i] < aabb.minv[i] || seg.start[i] > aabb.maxv[i]) return {false};
    }

    return {true, interpolateSegment(seg, tmin)};
}

SegmentCollisionResult computeSegmentAARectangleCollision(const Segment& seg, const AARectangle& rect)
{
    SegmentCollisionResult result;
    result.is_colliding = false;

    Axis axis    = rect.axis;
    int axis_idx = (int) axis;

    Real2 proj_seg_start = project(seg.start, axis);
    Real2 proj_seg_end   = project(seg.end,   axis);
    Real2 rect_p1        = project(rect.p1,   axis); 
    Real2 rect_p2        = project(rect.p2,   axis);

    Real2 r = proj_seg_start - proj_seg_end;
    Real2 s = rect_p1 - rect_p2;

    Real rxs  = cross(r, s);
    
    if (glm::abs(rxs) < Epsilon) return result;

    Real2 qp  = proj_seg_start - rect_p1;
    Real qpxr = cross(qp, r);

    Real t = cross(qp, s) / rxs;
    Real u = qpxr / rxs;

    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) 
    {
        Real3 hit_point = seg.start + t * (seg.end - seg.start);

        if (isBetween(hit_point[axis_idx], rect.p1[axis_idx], rect.p2[axis_idx]))
        {
            result.is_colliding   = true;
            result.entering_point = hit_point;
            return result;
        }
    }

    return result;
}

SegmentCollisionResult computeSegmentAACylinderCollision(const Segment& seg, const AACylinder& aacyl)
{
    SegmentCollisionResult result;
    result.is_colliding = false;

    if (aacyl.axis == Axis::Z)
    {
        Real z1 = aacyl.base_center.z;
        Real z2 = aacyl.base_center.z + aacyl.height; 

        Real z_min = std::min(z1, z2);
        Real z_max = std::max(z1, z2);

        Real2 proj_seg_a  = Real2(seg.start.x, seg.start.y);
        Real2 proj_seg_b  = Real2(seg.end.x, seg.end.y);
        Real2 proj_center = Real2(aacyl.base_center.x, aacyl.base_center.y);

        Real2 s2 = proj_seg_a - proj_center;

        // start inside test
        if (seg.start.z >= z_min && 
            seg.start.z <= z_max && 
            glm::dot(s2, s2) <= aacyl.radius * aacyl.radius) 
        {
            result.is_colliding   = true;
            result.entering_point = seg.start;
            return result;
        }

        CircleIntersection circ_result = compute2DSegmentCircleIntersection(proj_seg_a, proj_seg_b, proj_center, aacyl.radius);

        Real3 tube_intersection = circ_result.t * (seg.end - seg.start) + seg.start;

        bool tube_intersection_is_valid = 
                circ_result.is_intersecting && 
                tube_intersection.z >= z_min && 
                tube_intersection.z <= z_max;

        if (seg.start.z > z_min && seg.start.z < z_max)
        {
            result.is_colliding   = tube_intersection_is_valid;
            result.entering_point = seg.start + (seg.end - seg.start) * circ_result.t;

            return result;
        }

        Real3 plane_normal = Real3(0.0f, 0.0f, 1.0f);
        Real3 plane_point;

        if (seg.start.z <= z_min)
            plane_point = Real3(aacyl.base_center.x, aacyl.base_center.y, z_min);
        else if (seg.start.z >= z_max)
            plane_point = Real3(aacyl.base_center.x, aacyl.base_center.y, z_max);

        PlaneIntersection plane_result = computeSegmentPlaneIntersection(seg, plane_normal, plane_point);

        bool face_intersection_is_valid = false;
        Real3 plane_hit_point = seg.start + (seg.end - seg.start) * plane_result.t;

        face_intersection_is_valid = 
            plane_result.is_intersecting && 
            glm::dot(plane_hit_point - plane_point, plane_hit_point - plane_point) <= aacyl.radius*aacyl.radius;

        if (tube_intersection_is_valid && face_intersection_is_valid)
        {
            Real t = std::min(plane_result.t, circ_result.t);

            result.is_colliding   = true;
            result.entering_point = seg.start + (seg.end - seg.start) * t;
        }
        else if (face_intersection_is_valid)
        {
            result.is_colliding   = true;
            result.entering_point = seg.start + (seg.end - seg.start) * plane_result.t;
        }
        else if (tube_intersection_is_valid)
        {
            result.is_colliding   = true;
            result.entering_point = seg.start + (seg.end - seg.start) * circ_result.t;
        }

        return result;
    }
    else
    {
        ASSERT(false, "TODO");
    }

    return result;
}

SegmentCollisionResult computeSegmentAACapsuleCollision(const Segment& seg, const AACapsule& aacapsule)
{
    SegmentCollisionResult result;
    result.is_colliding = false;

    Axis axis = aacapsule.axis;
    int axis_idx = (int)axis;

    Real a1 = aacapsule.start[axis_idx];                   
    Real a2 = aacapsule.start[axis_idx] + aacapsule.height; 

    Real a_min = std::min(a1, a2);
    Real a_max = std::max(a1, a2);

    Real radius_sq    = aacapsule.radius*aacapsule.radius;
    Real2 proj_seg_a  = project(seg.start, axis);      
    Real2 proj_seg_b  = project(seg.end, axis);         
    Real2 proj_center = project(aacapsule.start, axis); 

    if (squaredDistancePointAASegment(seg.start, aacapsule.start, aacapsule.height, aacapsule.axis) < radius_sq)
        return {true, seg.start};


    // Tube collision

    CircleIntersection circ_result = compute2DSegmentCircleIntersection(proj_seg_a, proj_seg_b, proj_center, aacapsule.radius);

    Real3 tube_intersection = circ_result.t * (seg.end - seg.start) + seg.start;

    bool tube_intersection_is_valid = 
            circ_result.is_intersecting && 
            tube_intersection[axis_idx] >= a_min && 
            tube_intersection[axis_idx] <= a_max;

    // Spheres collision

    SphereIntersection sphere_result;

    Real3 center_min     = aacapsule.start;
    center_min[axis_idx] = a_min;
    SphereIntersection sphere_result_min = computeSegmentSphereIntersection(seg, center_min, aacapsule.radius);

    Real3 center_max     = aacapsule.start;
    center_max[axis_idx] = a_max;
    SphereIntersection sphere_result_max = computeSegmentSphereIntersection(seg, center_max, aacapsule.radius);

    if (sphere_result_min.is_intersecting && sphere_result_max.is_intersecting)
    {
        if (sphere_result_min.t < sphere_result_max.t)
        {
            sphere_result = sphere_result_min;
        }
        else
        {
            sphere_result = sphere_result_max;
        }
    }
    else if (sphere_result_min.is_intersecting)
    {
        sphere_result = sphere_result_min;
    }
    else if (sphere_result_max.is_intersecting)
    {
        sphere_result = sphere_result_max;
    }

    // Final result

    Real t = circ_result.t;
    bool found_collision = tube_intersection_is_valid;

    if (tube_intersection_is_valid && sphere_result.is_intersecting)
    {
        found_collision = true;
        t = std::min(circ_result.t, sphere_result.t);
    }
    else if (sphere_result.is_intersecting)
    {
        found_collision = true;
        t = sphere_result.t;
    }

    if (found_collision == false) return result;

    result.is_colliding   = true;
    result.entering_point = seg.start + (seg.end - seg.start) * t;

    return result;
}

SegmentCollisionResult computeSegmentAxialOBBCollision(const Segment& seg, const AxialOBB& axialobb)
{
    auto [rot_axis, u, v] = getAxes(axialobb.rotation_axis);

    Real angle         = axialobb.angle;
    Real cos_angle     = cos(angle);
    Real sin_angle     = sin(angle);
    Real cos_neg_angle =  cos_angle;
    Real sin_neg_angle = -sin_angle; 

    Segment rot_seg = {seg.start - axialobb.center, seg.end - axialobb.center};

    Real v1 = rot_seg.start[u];
    Real v2 = rot_seg.start[v];

    rot_seg.start[u] = v1 * cos_neg_angle - v2 * sin_neg_angle;
    rot_seg.start[v] = v1 * sin_neg_angle + v2 * cos_neg_angle;

    v1 = rot_seg.end[u];
    v2 = rot_seg.end[v];

    rot_seg.end[u] = v1 * cos_neg_angle - v2 * sin_neg_angle;
    rot_seg.end[v] = v1 * sin_neg_angle + v2 * cos_neg_angle;

    auto aabb_result = computeSegmentAABBCollision(rot_seg, {-axialobb.extents, axialobb.extents});

    if (aabb_result.is_colliding == false) return {false};

    SegmentCollisionResult result;
    result.is_colliding = true;

    v1 = aabb_result.entering_point[u];
    v2 = aabb_result.entering_point[v];

    result.entering_point = aabb_result.entering_point;
    result.entering_point[u] = v1 * cos_angle - v2 * sin_angle;
    result.entering_point[v] = v1 * sin_angle + v2 * cos_angle;

    result.entering_point += axialobb.center;

    return result;
}

SegmentCollisionResult computeSegmentAA4SidedPyramidCollision(const Segment& seg, const AA4SidedPyramid& pyr)
{
    auto [up, u, v] = getAxes(pyr.axis);

    bool start_below = 
        (pyr.height > 0.0f) ? 
            (seg.start[up] < pyr.base_center[up]) : 
            (seg.start[up] > pyr.base_center[up]);

    Real delta = (seg.end[up] - seg.start[up]);

    if (std::abs(delta) > Epsilon && start_below)
    {
        Real t = (pyr.base_center[up] - seg.start[up]) / (seg.end[up] - seg.start[up]);

        if (t > 0.0f && t < 1.0f) 
        {
            Real3 hit_point = interpolateSegment(seg, t);
            Real3 abs_diff  = glm::abs(hit_point - pyr.base_center);

            if (abs_diff[u] < pyr.half_extents.x && abs_diff[v] < pyr.half_extents.y) return {true, hit_point};
        }
    }

    std::array<Real3, 5> vertices = computeAA4SidedPyramidVertices(pyr);

    return computeSegmentTrianglesCollision(seg, vertices.data(), AA4SidedPyramidSidesIndices, 4);
}

SegmentCollisionResult computeSegmentCapsuleCollision(const Segment& seg, const Capsule& capsule)
{
    SegmentCollisionResult result;
    result.is_colliding = false;

    Real3 ori = seg.start;
    Real3 n = glm::normalize(seg.end - seg.start);

    Segment cap_seg      = {capsule.start, capsule.end};
    Segment proj_cap_seg = {projectOnPlane(ori, n, cap_seg.start), projectOnPlane(ori, n, cap_seg.end)};

    Real3 proj_cap_dir = proj_cap_seg.end - proj_cap_seg.start;

    Real proj_cap_sq_length = glm::dot(proj_cap_dir, proj_cap_dir);

    Real t;

    if (proj_cap_sq_length < Epsilon) [[unlikely]] 
        t = 0;
    else 
        Real t = glm::dot(ori - proj_cap_seg.start, proj_cap_dir) / proj_cap_sq_length;

    t = glm::clamp(t, 0.0f, 1.0f);

    Real3 cap_seg_unbound = (capsule.end - capsule.start) * t + capsule.start;

    Real3 p_seg = constraintToSegment(cap_seg_unbound, seg.start, seg.end);
    Real3 p_cap = constraintToSegment(p_seg, capsule.start, capsule.end);

    Real3 diff = p_cap - p_seg;

    Real dist_sq = glm::dot(diff, diff);

    if (dist_sq > capsule.radius * capsule.radius) return result;

    result.is_colliding   = true;
    result.entering_point = p_seg;

    return result;
}


SegmentCollisionResult computeSegmentShapeCollision(const Segment& seg, const CollisionShape& coll_shape)
{
    using Type = CollisionShapeType;
    const auto& s = coll_shape;

    switch(s.type)
    {
        #define X(type_name, union_name) case Type::type_name: return computeSegment ##type_name ##Collision(seg, s.union_name);
        COLLISION_TYPES
        #undef X

        default: ASSERT(false, "Segment collision not implemented for this shape");
    }

    return {};
}

// SPECIAL CASES

bool computeFastSegmentsAABBCollision(const FastSegment* segs, size_t num_segments, const AABB& aabb)
{
    for (size_t i = 0; i < num_segments; ++i)
    {
        const FastSegment& s = segs[i];

        float t1 = (aabb.minv.x - s.start.x) * s.inv_dir.x;
        float t2 = (aabb.maxv.x - s.start.x) * s.inv_dir.x;
        float tmin = std::min(t1, t2);
        float tmax = std::max(t1, t2);

        t1 = (aabb.minv.y - s.start.y) * s.inv_dir.y;
        t2 = (aabb.maxv.y - s.start.y) * s.inv_dir.y;
        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));

        t1 = (aabb.minv.z - s.start.z) * s.inv_dir.z;
        t2 = (aabb.maxv.z - s.start.z) * s.inv_dir.z;
        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));

        if (tmax >= std::max(0.0f, tmin) && tmin <= 1.0f)
        {
            return true;
        }
    }

    return false;
}

// TODO: see implentation in: http://www.r-5.org/files/books/computers/algo-list/realtime-3d/Christer_Ericson-Real-Time_Collision_Detection-EN.pdf page: 183

// =======================================================
// RAY COLLISION
// =======================================================

#define NullComputeRayCollisionDefinition(type_name)                                          \
RayCollisionResult computeRay ##type_name ##Collision(const Ray& ray, const type_name& other) \
{ return {false}; }                                                                           \
// { ASSERT(false, "UNREACHABLE: computeRayCollision unavailable for " #type_name); }         \

RayCollisionResult computeRayAABBCollision(const Ray& ray, const AABB& box)
{
    Real tmin = 0.0f;
    Real tmax = std::numeric_limits<Real>::max();
    int normal_axis = 0;

    for (int i = 0; i < 3; ++i) 
    {
        Real origin = ray.origin[i];
        Real dir    = ray.direction[i];
        Real minb   = box.minv[i];
        Real maxb   = box.maxv[i];

        if (std::abs(dir) < Epsilon) 
        {
            if (origin < minb || origin > maxb) return {false}; 
        } 
        else 
        {
            Real invd = 1.0f / dir;
            Real t1   = (minb - origin) * invd;
            Real t2   = (maxb - origin) * invd;

            if (t1 > t2) std::swap(t1, t2);

            if (tmin < t1)
            {
                tmin        = t1;
                normal_axis = i;
            }

            tmax = std::min(tmax, t2);

            if (tmin > tmax) return {false}; 
        }
    }

    if (tmax < 0.0f) return {false}; 

    Real mult = (ray.direction[normal_axis] > 0.0f) ? -1.0f : 1.0f;

    return {true, interpolateRay(ray, tmin), tmin, GlobalAxes[normal_axis] * mult};
}

RayCollisionResult computeRayAARampCollision(const Ray& ray, const AARamp& aaramp)
{
    AABB ramp_aabb = computeAARampAABB(aaramp);  
    auto aabb_res  = computeRayAABBCollision(ray, ramp_aabb);

    if (!aabb_res.is_colliding) return {false};

    Real2 proj(aabb_res.entering_point.x, aabb_res.entering_point.y);

    Real slope_z = getAARampPointHeight(aaramp, proj);

    if (aabb_res.entering_point.z <= slope_z) return aabb_res;

    std::array<Real3, 4> vertices = computeAARampWedgeVertices(aaramp);

    return computeRayTrianglesCollision(ray, vertices.data(), AARampWedgeIndices, 2);
}

RayCollisionResult computeRaySphereCollision(const Ray& ray, const Sphere& sphere)
{
    Real3 oc = ray.origin - sphere.center;

    Real a = glm::dot(ray.direction, ray.direction);
    Real b = 2.0f * glm::dot(oc, ray.direction);
    Real c = glm::dot(oc, oc) - sphere.radius * sphere.radius;

    Real discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f) return {false};

    Real sqrt_disc = std::sqrt(discriminant);
    Real t = (-b - sqrt_disc) / (2.0f * a);

    if (t < 0.0f) 
    {
        t = (-b + sqrt_disc) / (2.0f * a);
        if (t < 0.0f) return {false}; 
    }

    Real3 hit_point = interpolateRay(ray, t);
    Real3 normal    = glm::normalize(hit_point - sphere.center);

    return {true, hit_point, t, normal};
}

RayCollisionResult computeRayAACylinderCollision(const Ray& ray, const AACylinder& cyl)
{
    ASSERT(cyl.height > 0.0f, "AAcylinder height must be positive");

    auto [up, u, v] = getAxes(cyl.axis);

    ClosestHit closest_hit;

    Real2 circle_center(projectOn(cyl.base_center, u, v));
    Real2 ray_ori(projectOn(ray.origin,    u, v));
    Real2 ray_dir(projectOn(ray.direction, u, v));

    Real2 oc = ray_ori - circle_center;

    Real radius_sq = cyl.radius * cyl.radius;
    Real a = glm::dot(ray_dir, ray_dir);
    Real b = glm::dot(oc, ray_dir);
    Real c = glm::dot(oc, oc) - radius_sq;

    Real discriminant = b * b - a * c;

    if (discriminant > 0.0f)
    {
        Real sqrt_disc = std::sqrt(discriminant);
        Real inv_a     = 1.0f / a;

        Real t = (-b - sqrt_disc) * inv_a;

        if (t > 0.0f)
        {
            Real3 hit_point = interpolateRay(ray, t);

            if (hit_point[up] > cyl.base_center[up] && hit_point[up] < cyl.base_center[up] + cyl.height) 
            {
                Real2 hit_normal_2d = ((ray_ori + ray_dir * t) - circle_center) / cyl.radius;

                closest_hit.update(t, hit_point, unProjectOn(hit_normal_2d, u, v));
            }
        }
    }

    static auto intersectCap = [&](Real cap_height, Real3 cap_normal)
    {
        if (std::abs(ray.direction[up]) < Epsilon) return;
        
        Real tc = (cap_height - ray.origin[up]) / ray.direction[up];

        if (tc > 0.0f && closest_hit.isCloser(tc))
        {
            Real3 hit_point = interpolateRay(ray, tc);
            Real2 diff      = projectOn(hit_point, u, v) - circle_center;

            if (glm::dot(diff, diff) <= radius_sq) closest_hit.update(tc, hit_point, cap_normal);
        }
    };

    if (ray.origin[up] < cyl.base_center[up])              
        intersectCap(cyl.base_center[up], GlobalAxes[up] * (-1.0f));
    else if (ray.origin[up] > cyl.base_center[up] + cyl.height) 
        intersectCap(cyl.base_center[up] + cyl.height, GlobalAxes[up]);

    return closest_hit.toRayCollisionResult();
}

RayCollisionResult computeRayAARectangleCollision(const Ray& ray, const AARectangle& rec)
{
    auto [up, u, v] = getAxes(rec.axis);

    Real2 p1_proj = projectOn(rec.p1, u, v);
    Real2 p2_proj = projectOn(rec.p2, u, v);

    Real2 seg_start = p1_proj;
    Real2 seg_dir   = p2_proj - p1_proj;

    Real2 ray_ori = projectOn(ray.origin,    u, v);
    Real2 ray_dir = projectOn(ray.direction, u, v);

    Real cross_ray_seg = cross(ray_dir, seg_dir); 

    if (std::abs(cross_ray_seg) < Epsilon) return {false}; 

    Real2 diff = seg_start - ray_ori;  

    Real inv_cross = 1.0f / cross_ray_seg;
    Real s = cross(diff, ray_dir) * inv_cross; 
    Real t = cross(diff, seg_dir) * inv_cross; 

    if (s >= 0.0f && s <= 1.0f && t >= 0.0f) 
    {
        Real3 hit_point = interpolateRay(ray, t);

        if (isBetween(hit_point[up], rec.p1[up], rec.p2[up]))
        {
            Real3 normal = unProjectOn(glm::normalize(seg_dir), u, v);
            rotate90AroundAxis(normal, rec.axis);
            if (glm::dot(ray.origin, normal) > 0.0) 
                normal = (-1.0f) * normal;
            return {true, hit_point, t, normal};
        }
    }

    return {false};
}

RayCollisionResult computeRayAA4SidedPyramidCollision(const Ray& ray, const AA4SidedPyramid& pyr)
{
    auto [up, u, v] = getAxes(pyr.axis);

    bool start_below = 
        (pyr.height > 0.0f) ? 
            (ray.origin[up] < pyr.base_center[up]) : 
            (ray.origin[up] > pyr.base_center[up]);

    Real delta = ray.direction[up];

    if (start_below && std::abs(delta) > Epsilon)
    {
        Real t = (pyr.base_center[up] - ray.origin[up]) / delta;

        if (t > 0.0f) 
        {
            Real3 hit_point = interpolateRay(ray, t);
            Real3 abs_diff  = glm::abs(hit_point - pyr.base_center);

            if (abs_diff[u] < pyr.half_extents.x && abs_diff[v] < pyr.half_extents.y) return {true, hit_point};
        }
    }

    std::array<Real3, 5> vertices = computeAA4SidedPyramidVertices(pyr);

    return computeRayTrianglesCollision(ray, vertices.data(), AA4SidedPyramidSidesIndices, 4);
}

RayCollisionResult computeRayAACapsuleCollision(const Ray& ray, const AACapsule& caps) 
{ 
    ASSERT(caps.height > 0.0f, "AAcylinder height must be positive");

    auto [up, u, v] = getAxes(caps.axis);

    ClosestHit closest_hit;

    Real2 circle_center(projectOn(caps.start, u, v));
    Real2 ray_ori(projectOn(ray.origin,    u, v));
    Real2 ray_dir(projectOn(ray.direction, u, v));

    Real2 oc = ray_ori - circle_center;

    Real radius_sq = caps.radius * caps.radius;
    Real a = glm::dot(ray_dir, ray_dir);
    Real b = glm::dot(oc, ray_dir);
    Real c = glm::dot(oc, oc) - radius_sq;

    Real discriminant = b * b - a * c;

    if (discriminant > 0.0f)
    {
        Real sqrt_disc = std::sqrt(discriminant);
        Real inv_a     = 1.0f / a;

        Real t = (-b - sqrt_disc) * inv_a;

        if (t > 0.0f)
        {
            Real3 hit_point = interpolateRay(ray, t);

            if (hit_point[up] > caps.start[up] && hit_point[up] < caps.start[up] + caps.height) 
            {
                Real2 hit_normal_2d = ((ray_ori + ray_dir * t) - circle_center) / caps.radius;
                closest_hit.update(t, hit_point, unProjectOn(hit_normal_2d, u, v));
            }
        }
    }

    closest_hit.updateIfCloser(
        computeRaySphereCollision(
            ray, {   
                .center = caps.start, 
                .radius = caps.radius
            }));

    closest_hit.updateIfCloser(
        computeRaySphereCollision(
            ray, {
                .center = translateOnAxis(caps.start, caps.height, up), 
                .radius = caps.radius
            }));

    return closest_hit.toRayCollisionResult();
}

inline Ray transformRayToLocalAxial(const Ray& ray, const Real3& center, int u, int v, Real cos_angle, Real sin_angle)
{
    Ray local_ray = { ray.origin - center, ray.direction };

    auto rotateBack2D = [&](Real& val_u, Real& val_v) 
    {
        Real old_u = val_u;
        Real old_v = val_v;
        val_u = old_u *   cos_angle  - old_v * (-sin_angle);
        val_v = old_u * (-sin_angle) + old_v *   cos_angle;
    };

    rotateBack2D(local_ray.origin[u],    local_ray.origin[v]);
    rotateBack2D(local_ray.direction[u], local_ray.direction[v]);

    return local_ray;
}

inline Real3 transformLocalAxialDirectionToGlobal(const Real3& dir, const Real3 center, int u, int v, Real cos_angle, Real sin_angle)
{
    Real3 global_dir = dir;

    global_dir[u] = dir[u] * cos_angle - dir[v] * sin_angle;
    global_dir[v] = dir[u] * sin_angle + dir[v] * cos_angle;

    return global_dir;
}

RayCollisionResult computeRayAxialOBBCollision(const Ray& ray, const AxialOBB& axialobb)
{
    auto [rot_axis, u, v] = getAxes(axialobb.rotation_axis);

    Real angle     = axialobb.angle;
    Real cos_angle = cos(angle);
    Real sin_angle = sin(angle);

    Ray rot_ray = transformRayToLocalAxial(ray, axialobb.center, u, v, cos_angle, sin_angle);

    auto aabb_result = computeRayAABBCollision(rot_ray, {-axialobb.extents, axialobb.extents});

    if (aabb_result.is_colliding == false) return {false};

    Real3 hit_point  = interpolateRay(ray, aabb_result.t);
    Real3 hit_normal = transformLocalAxialDirectionToGlobal(aabb_result.contact_normal, axialobb.center, u, v, cos_angle, sin_angle);

    return {true, hit_point, aabb_result.t, hit_normal};
}

RayCollisionResult computeRayCapsuleCollision(const Ray& ray, const Capsule& caps)
{
    Real3 dir   = caps.end - caps.start;
    Real height = glm::length(dir);
    if (height < Epsilon) return computeRaySphereCollision(ray, {caps.start, caps.radius});

    Real3 axis_z = dir / height;

    Real3 arbitrary = (std::abs(axis_z.z) < 0.999f) ? Real3(0,0,1) : Real3(1,0,0);
    Real3 axis_x    = glm::normalize(glm::cross(arbitrary, axis_z));
    Real3 axis_y    = glm::cross(axis_z, axis_x);

    Real3x3 world_to_local = glm::transpose(Real3x3(axis_x, axis_y, axis_z));

    Ray local_ray;
    local_ray.origin    = world_to_local * (ray.origin - caps.start);
    local_ray.direction = world_to_local * ray.direction;

    AACapsule aacaps;
    aacaps.start  = Real3(0.0f);
    aacaps.radius = caps.radius;
    aacaps.height = height;
    aacaps.axis   = Axis::Z;

    auto res = computeRayAACapsuleCollision(local_ray, aacaps);

    if (res.is_colliding) 
    {
        res.entering_point = interpolateRay(ray, res.t); 
        res.contact_normal = glm::inverse(world_to_local) * res.contact_normal;
        res.contact_normal = glm::normalize(res.contact_normal);
    }

    return res;
}

NullComputeRayCollisionDefinition(Ray);
NullComputeRayCollisionDefinition(Segment);
NullComputeRayCollisionDefinition(OBB);
NullComputeRayCollisionDefinition(AAEllipsoid);
NullComputeRayCollisionDefinition(AACone);
NullComputeRayCollisionDefinition(AAHollowCylinder);
NullComputeRayCollisionDefinition(Point);

RayCollisionResult computeRayShapeCollision(const Ray& ray, const CollisionShape& coll_shape)
{
    using Type = CollisionShapeType;
    const auto& s = coll_shape;

    switch(s.type)
    {
        #define X(type_name, union_name) case Type::type_name: return computeRay ##type_name ##Collision(ray, s.union_name);
        COLLISION_TYPES
        #undef X

        default: ASSERT(false, "Segment collision not implemented for this shape");
    }

    return {};
}

// =======================================================
// Capsule first collision functions
// =======================================================

Real squaredDistancePointSegment(Real3 p, const Segment& seg)
{
    Real3 ab = seg.end - seg.start;
    Real3 ap = p - seg.start;
    Real3 bp = p - seg.end;

    Real t = glm::dot(ap, ab);

    if (t <= 0.0f) return glm::dot(ap, ap);

    Real denom = glm::dot(ab, ab);

    if (t >= denom) return glm::dot(bp, bp);

    return glm::dot(ap, ap) - (t * t / denom);
}

Real squaredDistanceSegmentSegment(const Segment& seg1, const Segment& seg2)
{
    Real3 d1 = seg1.end - seg1.start;    // Direction vector of segment S1
    Real3 d2 = seg2.end - seg2.start;    // Direction vector of segment S2
    Real3 r  = seg1.start - seg2.start; 
    Real a   = glm::dot(d1, d1);         // Squared length of segment S1, always nonnegative
    Real e   = glm::dot(d2, d2);         // Squared length of segment S2, always nonnegative

    if (a <= Epsilon && e <= Epsilon) // Both segment degenerates into a point
    {
        return glm::dot(r, r);
    }
    else if (a <= Epsilon) // First segment degenerates into a point
    {
        return squaredDistancePointSegment(seg1.start, seg2);
    }
    else if (e <= Epsilon) // Second segment degenerates into a point
    {
        return squaredDistancePointSegment(seg2.start, seg1);
    }

    Real f   = glm::dot(d2, r);
    Real c   = glm::dot(d1, r);

    // The general nondegenerate case starts here
    Real b     = glm::dot(d1, d2);
    Real denom = a*e-b*b;  // Always nonnegative

    // If segments not parallel, compute closest point on L1 to L2 and
    // clamp to segment S1. Else pick arbitrary s (here 0)
    Real s;
    if (denom > Epsilon) 
    { 
        s = glm::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
    } 
    else 
    {
        s = 0.0f; // Lines are parallel; any s is a starting point
    }

    // Compute point on L2 closest to S1(s) using
    // t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e
    Real t = (b*s + f) / e;

    // If t in [0,1] done. Else clamp t, recompute s for the new value
    // of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a
    // and clamp s to [0, 1]
    if (t < 0.0f) 
    {
        t = 0.0f;
        s = glm::clamp(-c / a, 0.0f, 1.0f);
    } 
    else if (t > 1.0f) 
    {
        t = 1.0f;
        s = glm::clamp((b - c) / a, 0.0f, 1.0f);
    }

    Real3 c1 = seg1.start + d1 * s;
    Real3 c2 = seg2.start + d2 * t;
    return glm::dot(c1 - c2, c1 - c2);
}

// =======================================================
// AAEllipsoid
// =======================================================

AABB computeAAEllipsoidAABB(const AAEllipsoid& shape) 
{ 
    return {shape.center - shape.half_extents, shape.center + shape.half_extents};
}

void updateAAEllipsoid(AAEllipsoid& runtime_shape, const AAEllipsoid& rest_shape, Real3 p) 
{ 
    runtime_shape = rest_shape;
    runtime_shape.center += p;
}

CollisionResult computeAABBAAEllipsoidCollision(const AABB& aabb, const AAEllipsoid& ellipsoid) 
{  
    Real3 closest = glm::clamp(ellipsoid.center, aabb.minv, aabb.maxv);
    
    Real3 diff   = (closest - ellipsoid.center) / ellipsoid.half_extents; 
    Real dist_sq = glm::dot(diff, diff);

    if (dist_sq > 1.0f) return {false}; 

    Real penetration = 1.0f - std::sqrt(dist_sq);
    Real3 normal     = glm::normalize(diff);
    Real3 mtv        = normal * penetration * glm::length(ellipsoid.half_extents * normal);

    return {true, mtv};
}

DefaultComputeAACapsuleCollisionDefinition(AAEllipsoid);
DefaultPrecalcComputationFunction(AAEllipsoid);

NullTestAABBCollisionDefinition(AAEllipsoid);

SegmentCollisionResult computeSegmentAAEllipsoidCollision(const Segment& seg, const AAEllipsoid& ellipsoid)
{
    Real3 start = (seg.start - ellipsoid.center) / ellipsoid.half_extents;
    Real3 end   = (seg.end   - ellipsoid.center) / ellipsoid.half_extents;

    Real3 d = end - start;
    Real a  = glm::dot(d, d);
    Real b  = 2.0f * glm::dot(start, d);
    Real c  = glm::dot(start, start) - 1.0f;
    
    Real discriminant = b*b - 4.0f*a*c;
    if (discriminant < 0.0f) return {false};
    
    Real t = (-b - std::sqrt(discriminant)) / (2.0f * a);

    if (t < 0.0f || t > 1.0f) return {false};
    
    Real3 hit_point_scaled = start + t * d;
    Real3 hit_point = hit_point_scaled * ellipsoid.half_extents + ellipsoid.center;

    return {true, hit_point};
}

// NullComputeRayCollisionDefinition(AAEllipsoid);
// CollisionResult computeAACapsuleAAEllipsoidCollision(const AACapsule& aacaps, const AAEllipsoid& other) { ... }
// bool testAABBAAEllipsoidCollision(const AABB& aabb, const AAEllipsoid& other) { ... }
// RayCollisionResult computeRayAAEllipsoidCollision(const Ray& ray, const AAEllipsoid& other) { ... }

// ==============================================
// AACone
// ==============================================

AABB computeAAConeAABB(const AACone& aacone) 
{
    auto [up, u, v] = getAxes(aacone.axis);

    Real3 min = aacone.base_center;
    min[u] -= aacone.radius;
    min[v] -= aacone.radius;

    Real3 max = aacone.base_center;
    max[u] += aacone.radius;
    max[v] += aacone.radius;

    auto [min_height, max_height] = std::minmax({0.0f, aacone.height});
    min[up] += min_height;
    max[up] += max_height;

    return {min, max};
}

void updateAACone(AACone& runtime_shape, const AACone& rest_shape, Real3 p) 
{ 
    runtime_shape = rest_shape;
    runtime_shape.base_center += p;
}

CollisionResult computeAABBAAConeCollision(const AABB& aabb, const AACone& cone) 
{
    auto [up, u, v] = getAxes(cone.axis);
    
    Real apex_coord = cone.base_center[up] + cone.height;
    Real base_coord = cone.base_center[up];
    
    Real aabb_min_up = aabb.minv[up];
    Real aabb_max_up = aabb.maxv[up];
    
    if (cone.height > 0.0f) 
    {
        if (aabb_max_up < base_coord || aabb_min_up > apex_coord) return {false};
    }
    else
    { 
        if (aabb_max_up < apex_coord || aabb_min_up > base_coord) return {false};
    }

    Real plane_height;
    Real below_base_mtv;
    Real over_apex_mtv;

    if (cone.height > 0.0f) 
    {
        below_base_mtv = base_coord - aabb_max_up;
        over_apex_mtv  = apex_coord - aabb_min_up;
        plane_height = std::max(aabb_min_up, base_coord);
    }
    else                    
    {
        below_base_mtv = base_coord - aabb_min_up;
        over_apex_mtv  = apex_coord - aabb_max_up;
        plane_height   = std::min(aabb_max_up, base_coord);
    }

    Real height_from_apex = std::abs(plane_height - apex_coord);
    Real circle_radius    = cone.radius * height_from_apex / std::abs(cone.height);
    Real2 circle_center   = Real2(cone.base_center[u], cone.base_center[v]);

    Real2 rec_min = Real2(aabb.minv[u], aabb.minv[v]);
    Real2 rec_max = Real2(aabb.maxv[u], aabb.maxv[v]);

    if (isPointInsideRectangle(circle_center, rec_min, rec_max))
    {
        CollisionResult result;
        result.is_colliding = true;
        result.mtv          = Real3(0.0f);

        result.mtv[up] = std::abs(below_base_mtv) < std::abs(over_apex_mtv) ?
                            below_base_mtv :
                            over_apex_mtv;
        return result;
    }

    Real2 closest_point = closestPointOnRectangle(circle_center, rec_min, rec_max) - circle_center;
    
    if (glm::dot(closest_point, closest_point) > circle_radius * circle_radius) return {false};

    CollisionResult result;
    result.is_colliding = true;
    result.mtv = Real3(0.0f);

    Real length     = glm::length(closest_point);
    Real mtv_length = (circle_radius - length);

    if (std::abs(below_base_mtv) < mtv_length)
    {
        result.mtv[up] = below_base_mtv;
    }
    else
    {
        Real2 radial_direction = closest_point / length;
        Real cone_slope = cone.radius / std::abs(cone.height);
        
        Real normal_scale = 1.0f / std::sqrt(1.0f + cone_slope * cone_slope);
        
        Real radial_component = mtv_length * normal_scale;
        Real axial_component  = mtv_length * cone_slope * normal_scale;
        
        result.mtv[u]  = radial_direction.x * radial_component;
        result.mtv[v]  = radial_direction.y * radial_component;
        result.mtv[up] = (cone.height > 0.0f) ? axial_component : -axial_component;
    }

    return result;
}

DefaultComputeAACapsuleCollisionDefinition(AACone);
DefaultPrecalcComputationFunction(AACone);

SegmentCollisionResult computeSegmentAAConeCollision(const Segment& seg, const AACone& cone)
{
    auto [up, u, v] = getAxes(cone.axis);
    
    // Transform segment to cone space (apex at origin, pointing down -axis)
    Real3 apex = cone.base_center;
    apex[up] += cone.height;
    
    Real3 p0 = seg.start - apex;
    Real3 p1 = seg.end   - apex;

    // Extract axial and radial components
    Real h0 = -p0[up];  // Height along cone axis (negated so base is positive)
    Real h1 = -p1[up];
    Real r0 = std::sqrt(p0[u]*p0[u] + p0[v]*p0[v]);  // Radial distance
    Real r1 = std::sqrt(p1[u]*p1[u] + p1[v]*p1[v]);
    
    // Cone equation: r = (radius/height) * h, for h in [0, height]
    Real cone_slope = cone.radius / std::abs(cone.height);

    // Ray direction
    Real3 d   = p1 - p0;
    Real dh   = h1 - h0;
    Real dr_u = d[u];
    Real dr_v = d[v];

    // Solve for intersection with infinite cone surface
    // (r0 + t*dr)^2 = (cone_slope * (h0 + t*dh))^2
    // Expanding: r0^2 + 2*r0*dr*t + dr^2*t^2 = slope^2 * (h0^2 + 2*h0*dh*t + dh^2*t^2)
    Real r0_sq     = r0*r0;
    Real dr_sq     = dr_u*dr_u + dr_v*dr_v;
    Real r0_dot_dr = p0[u]*dr_u + p0[v]*dr_v;
    Real slope_sq  = cone_slope * cone_slope;
    
    Real a = dr_sq - slope_sq * dh * dh;
    Real b = 2.0f * (r0_dot_dr - slope_sq * h0 * dh);
    Real c = r0_sq - slope_sq * h0 * h0;
    
    Real discriminant = b*b - 4.0f*a*c;
    if (discriminant < 0.0f) return {false};  // No intersection with infinite cone

    Real sqrt_disc = std::sqrt(discriminant);
    Real t1 = (-b - sqrt_disc) / (2.0f * a);
    Real t2 = (-b + sqrt_disc) / (2.0f * a);

    ClosestHit closest_hit;

    // Check side intersections
    for (Real t : {t1, t2}) 
    {
        if (t < 0.0f || t > 1.0f) continue;
        Real h = h0 + t * dh;
        if (h < 0.0f || h > std::abs(cone.height)) continue;
        
        if (closest_hit.isCloser(t)) closest_hit.update(t, seg.start + t * (seg.end - seg.start));
    }

    // Check base intersection
    Real base_plane = cone.base_center[up];
    Real denom = seg.end[up] - seg.start[up];
    if (std::abs(denom) > Epsilon && isBetween(base_plane, seg.start[up], seg.end[up]))
    {
        Real bt = (base_plane - seg.start[up]) / denom;

        if (bt >= 0.0f && bt <= 1.0f && closest_hit.isCloser(bt))
        {
            Real3 hit_point = seg.start + bt * (seg.end - seg.start);

            Real radial_dist_sq = (hit_point[u] - cone.base_center[u]) * (hit_point[u] - cone.base_center[u]) +
                                  (hit_point[v] - cone.base_center[v]) * (hit_point[v] - cone.base_center[v]);
            
            if (radial_dist_sq <= cone.radius * cone.radius) closest_hit.update(bt, hit_point);
        }
    }

    return closest_hit ? SegmentCollisionResult{true, closest_hit.getPoint()} : SegmentCollisionResult{false};
}

NullTestAABBCollisionDefinition(AACone);

// NullComputeRayCollisionDefinition(AACone);
// bool testAABBAAConeCollision(const AABB& aabb, const AACone& other) { ... }
// RayCollisionResult computeRayAAConeCollision(const Ray& ray, const AACone& other) { ... }

// ==============================================
// AAHollowCylinder
// ==============================================

AABB computeAAHollowCylinderAABB(const AAHollowCylinder& shape) 
{ 
    auto [up, u, v] = getAxes(shape.axis);

    Real3 min = shape.base_center;
    min[u] -= shape.outer_radius;
    min[v] -= shape.outer_radius;

    Real3 max = shape.base_center;
    max[u]  += shape.outer_radius;
    max[v]  += shape.outer_radius;
    max[up] += shape.height;

    return {min, max};
}

void updateAAHollowCylinder(AAHollowCylinder& runtime_shape, const AAHollowCylinder& rest_shape, Real3 p) 
{ 
    runtime_shape = rest_shape;
    runtime_shape.base_center += p;
}

CollisionResult computeAABBAAHollowCylinderCollision(const AABB& aabb, const AAHollowCylinder& cyl)
{ 
    ASSERT(cyl.height > 0.0f, "height must be positive");
    ASSERT(cyl.inner_radius > 0.0f, "inner radius must be positive");
    ASSERT(cyl.outer_radius > cyl.inner_radius, "outer radius must be bigger the inner radius");

    auto [up, u, v] = getAxes(cyl.axis);

    Real cyl_min_up = cyl.base_center[up];
    Real cyl_max_up = cyl_min_up + cyl.height;

    if (aabb.maxv[up] <= cyl_min_up || aabb.minv[up] >= cyl_max_up) return {false};

    Real2 rec_min = projectOn(aabb.minv, u, v);
    Real2 rec_max = projectOn(aabb.maxv, u, v);

    Real2 circle_center = projectOn(cyl.base_center, u, v);

    Real2 furthest = furthestPointOnRectangle(circle_center, rec_min, rec_max) - circle_center;

    // it is all contained inside the the inner hollow area
    if (glm::dot(furthest, furthest) < cyl.inner_radius * cyl.inner_radius) return {false};

    Real2 closest = closestPointOnRectangle(circle_center, rec_min, rec_max) - circle_center;

    // it is all outside the the outer radius
    if (glm::dot(closest, closest) > cyl.outer_radius * cyl.outer_radius) return {false};

    Real furthest_length = glm::length(furthest);
    Real closest_length  = glm::length(closest);

    Real inner_correction_depth = cyl.inner_radius - furthest_length;
    Real outer_correction_depth = cyl.outer_radius - closest_length;

    Real up_correction_depth   = cyl_max_up - aabb.minv[up];
    Real down_correction_depth = cyl_min_up - aabb.maxv[up];

    CollisionResult result;
    result.is_colliding = true;
    result.mtv = Real3(0.0f);

    if (std::abs(up_correction_depth) < std::abs(inner_correction_depth) && 
        std::abs(up_correction_depth) < std::abs(outer_correction_depth) &&
        std::abs(up_correction_depth) < std::abs(down_correction_depth))    
    {
        result.mtv[up] = up_correction_depth;
        return result;
    }
    else 
    if (std::abs(down_correction_depth) < std::abs(inner_correction_depth) && 
        std::abs(down_correction_depth) < std::abs(outer_correction_depth))
    {
        result.mtv[up] = down_correction_depth;
        return result;
    }

    Real2 mtv2d(0.0f);

    if (std::abs(inner_correction_depth) < std::abs(outer_correction_depth))
    {
        Real2 inner_correction_direction = furthest / furthest_length;
        mtv2d = inner_correction_direction * inner_correction_depth;
    }
    else
    {
        Real2 outer_correction_direction = closest / closest_length;
        mtv2d = outer_correction_direction * outer_correction_depth;
    }

    result.mtv[u] = mtv2d.x;
    result.mtv[v] = mtv2d.y;
    
    return result;
}

DefaultComputeAACapsuleCollisionDefinition(AAHollowCylinder);
DefaultPrecalcComputationFunction(AAHollowCylinder);

NullTestAABBCollisionDefinition(AAHollowCylinder);

SegmentCollisionResult computeSegmentAAHollowCylinderCollision(const Segment& seg, const AAHollowCylinder& cyl)
{ 
    ASSERT(cyl.height > 0.0f, "height must be positive");
    ASSERT(cyl.inner_radius > 0.0f, "inner radius must be positive");
    ASSERT(cyl.outer_radius > cyl.inner_radius, "outer radius must be bigger the inner radius");

    auto [up, u, v] = getAxes(cyl.axis);

    Real2 inn_cricle_center(projectOn(cyl.base_center, u, v));

    Real2 seg_start(projectOn(seg.start, u, v) - inn_cricle_center);
    Real2 seg_end(projectOn(seg.end, u, v) - inn_cricle_center);

    Real2 seg_dir = seg_end - seg_start;

    Real inn_rad_sq = cyl.inner_radius * cyl.inner_radius;
    Real out_rad_sq = cyl.outer_radius * cyl.outer_radius;

    Real seg_start_sq = dot(seg_start, seg_start);
    Real seg_end_sq   = dot(seg_end, seg_end);

    if (seg_start_sq <= inn_rad_sq && seg_end_sq <= inn_rad_sq) return {false};

    ClosestHit closest_hit;

    Real a = dot(seg_dir, seg_dir);
    Real b = 2.0f * dot(seg_start, seg_dir);
    Real c_inn = seg_start_sq - inn_rad_sq;
    Real c_out = seg_start_sq - out_rad_sq;

    auto checkIntersectionCylinderShell = [&](Real c) 
    {
        Real discriminant = b*b - 4.0f*a*c;

        if (discriminant > 0.0f) 
        {
            Real sqrt_disc = std::sqrt(discriminant);
            Real t1 = (-b - sqrt_disc) / (2.0f * a);
            Real t2 = (-b + sqrt_disc) / (2.0f * a);

            for (Real t : {t1, t2}) 
            {
                if (t < 0.0f || t > 1.0f) continue;

                Real3 hit_point = interpolateSegment(seg, t);
                Real h = hit_point[up] - cyl.base_center[up];

                if (h <= 0.0f || h >= cyl.height) continue;

                if (closest_hit.isCloser(t)) closest_hit.update(t, hit_point);
            }
        }
    };

    auto checkIntersectionHollowCap = [&](Real up_coord)
    {
        Real t = (up_coord - seg.start[up]) / (seg.end[up] - seg.start[up]);

        if (t < 0.0f || t > 1.0f && closest_hit.isCloser(t)) return;

        Real3 hit_point = interpolateSegment(seg, t);

        Real3 hit_point_local = hit_point - cyl.base_center;

        Real lenght_sq = hit_point_local[u] * hit_point_local[u] + hit_point_local[v] * hit_point_local[v];

        if (lenght_sq >= inn_rad_sq && lenght_sq <= out_rad_sq) closest_hit.update(t, hit_point);
    };

    checkIntersectionCylinderShell(c_inn);
    checkIntersectionCylinderShell(c_out);

    checkIntersectionHollowCap(cyl.base_center[up]);
    checkIntersectionHollowCap(cyl.base_center[up] + cyl.height);

    return closest_hit ? SegmentCollisionResult{true, closest_hit.getPoint()} : SegmentCollisionResult{false};
}

// NullComputeRayCollisionDefinition(AAHollowCylinder);
// bool testAABBAAHollowCylinderCollision(const AABB& aabb, const AAHollowCylinder& other) { ... }
// RayCollisionResult computeRayAAHollowCylinderCollision(const Ray& ray, const AAHollowCylinder& other) { ... }

// ==============================================
// AAHemisphere
// ==============================================

void updateAAHemisphere(AAHemisphere& aahemi, const AAHemisphere& rest_aahemi, Real3 p)
{
    aahemi = rest_aahemi;
    aahemi.center = rest_aahemi.center + p;
}

AABB computeAAHemisphereAABB(const AAHemisphere& aahemisphere)
{
    auto [up, u, v] = getAxes(aahemisphere.axis);
    
    Real r  = aahemisphere.radius;
    Real3 c = aahemisphere.center;
    
    AABB box;
    box.minv = c - r; 
    box.maxv = c + r;
    
    if (isPositiveAxis(aahemisphere.axis)) 
        box.minv[up] = c[up];
    else 
        box.maxv[up] = c[up];
    
    return box;
}

CollisionResult computeAABBAAHemisphereCollision(const AABB& aabb, const AAHemisphere& hemisphere) 
{
    ASSERT(hemisphere.radius >= 0.0f, "AAHemisphere radius must be positive");

    auto [up, u, v]  = getAxes(hemisphere.axis);
    bool is_positive = isPositiveAxis(hemisphere.axis);

    Real center_up = hemisphere.center[up];
    Real radius    = hemisphere.radius;

    if (is_positive ? isAABBBelowPlane(aabb, up, center_up)          : isAABBAbovePlane(aabb, up, center_up))          return {false};
    if (is_positive ? isAABBAbovePlane(aabb, up, center_up + radius) : isAABBBelowPlane(aabb, up, center_up - radius)) return {false};

    AABB clip_aabb = clipByPlane(aabb, up, center_up, is_positive);

    Real up_depth = 0.0f;

    if (is_positive) up_depth = center_up - aabb.maxv[up];
    else             up_depth = center_up - aabb.minv[up];

    bool up_correction_is_valid = (is_positive && aabb.minv[up] < center_up) || (!is_positive && aabb.maxv[up] > center_up);

    Real3 closest  = closestPointOnAABB(clip_aabb, hemisphere.center) - hemisphere.center;
    Real dist_sq   = glm::dot(closest, closest);
    Real radius_sq = radius * radius;

    if (dist_sq > radius_sq) return {false};

    Real length    = std::sqrt(dist_sq);
    Real mtv_depth = hemisphere.radius - length;

    if (up_correction_is_valid == false) return {true, closest / length * mtv_depth};

    if (std::abs(mtv_depth) < std::abs(up_depth)) return {true, closest / length * mtv_depth};

    Real3 mtv(0.0f);
    mtv[up] = up_depth;
    return {true, mtv};
}

DefaultComputeAACapsuleCollisionDefinition(AAHemisphere);
DefaultPrecalcComputationFunction(AAHemisphere);

bool testAABBAAHemisphereCollision(const AABB& aabb, const AAHemisphere& hemisphere)
{
    ASSERT(hemisphere.radius >= 0.0f, "AAHemisphere radius must be positive");

    auto [up, u, v]  = getAxes(hemisphere.axis);
    bool is_positive = isPositiveAxis(hemisphere.axis);

    Real center_up = hemisphere.center[up];
    Real radius    = hemisphere.radius;

    if (is_positive ? isAABBBelowPlane(aabb, up, center_up)          : isAABBAbovePlane(aabb, up, center_up))          return {false};
    if (is_positive ? isAABBAbovePlane(aabb, up, center_up + radius) : isAABBAbovePlane(aabb, up, center_up - radius)) return {false};

    AABB clip_aabb = clipByPlane(aabb, up, center_up, is_positive);

    Sphere sphere;
    sphere.center = hemisphere.center;
    sphere.radius = radius;

    return testAABBSphereCollision(clip_aabb, sphere);
}

SegmentCollisionResult computeSegmentAAHemisphereCollision(const Segment& seg, const AAHemisphere& hemisphere)
{
    auto [up, u, v]  = getAxes(hemisphere.axis);
    bool positive = isPositiveAxis(hemisphere.axis);

    Real center_up = hemisphere.center[up];
    Real radius    = hemisphere.radius;

    if (positive ? 
        isSegmentBelowPlane(seg, up, center_up) : 
        isSegmentAbovePlane(seg, up, center_up))          
            return {false};

    if (positive ? 
        isSegmentAbovePlane(seg, up, center_up + radius) : 
        isSegmentBelowPlane(seg, up, center_up - radius)) 
            return {false};

    Segment clip_seg = clipByPlane(seg, up, center_up, !positive);

    Real3 d = clip_seg.end   - clip_seg.start;
    Real3 f = clip_seg.start - hemisphere.center;

    Real a = glm::dot(d, d);
    Real b = 2.0f * glm::dot(f, d);
    Real c = glm::dot(f, f) - (radius * radius);

    if (c <= 0.0f) /* && std::abs(f[up]) < Epsilon) */ return {true, clip_seg.start};

    Real discriminant = b * b - 4 * a * c;

    if (discriminant < 0) return {false};

    discriminant = std::sqrt(discriminant);
    Real t1 = (-b - discriminant) / (2.0f * a);
    Real t2 = (-b + discriminant) / (2.0f * a);

    return {false};
}

RayCollisionResult computeRayAAHemisphereCollision(const Ray& ray, const AAHemisphere& hemisphere)
{
    auto [up, u, v] = getAxes(hemisphere.axis);
    bool positive   = isPositiveAxis(hemisphere.axis);

    auto sphere_res = computeRaySphereCollision(ray, {.center = hemisphere.center, .radius = hemisphere.radius});

    if (sphere_res.is_colliding == false) return {false};

    if (positive  && sphere_res.entering_point[up] > hemisphere.center[up]) return sphere_res;
    if (!positive && sphere_res.entering_point[up] < hemisphere.center[up]) return sphere_res;

    Real denom = ray.direction[up];

    if (std::abs(denom) < Epsilon) return {false};

    Real t = (hemisphere.center[up] - ray.origin[up]) / denom;

    if (t < 0.0f) [[unlikely]] return {false};

    Real3 hp       = interpolateRay(ray, t);
    Real3 diff     = hp - hemisphere.center;
    Real radius_sq = hemisphere.radius * hemisphere.radius;

    if (glm::dot(diff, diff) > radius_sq) return {false};

    return {true, hp, t, positive ? GlobalAxesNeg[up] : GlobalAxes[up]};
}

// ==============================================
// AATrapezoid
// ==============================================

inline SegmentCollisionResult computeSegmentRectangleCollision(const Segment& seg, Real3 rec_center, Real2 extents, Axis axis)
{
    auto [up, u, v] = getAxes(axis);

    Real denom = seg.end[up] - seg.start[up];
    
    if (std::abs(denom) < Epsilon) return {false};

    if (isBetween(rec_center[up], seg.start[up], seg.end[up]))
    {
        Real t       = (rec_center[up] - seg.start[up]) / denom;
        Real3 hp     = interpolateSegment(seg, t);
        Real3 hp_rel = hp - rec_center;

        if (std::abs(hp_rel[u]) <= extents.x && std::abs(hp_rel[v]) <= extents.y) return {true, hp};
    }
    
    return {false};
};

AABB computeAATrapezoidAABB(const AATrapezoid& shape) 
{
    auto [up, u, v] = getAxes(shape.axis);

    Real3 max_half(0.0f);
    max_half[u] = std::max(shape.half_extents.x, shape.top_half_extents.x);
    max_half[v] = std::max(shape.half_extents.y, shape.top_half_extents.y);

    Real3 h_vec(0.0f);
    h_vec[up] = shape.height;

    const Real3& bc = shape.base_center;

    return { bc - max_half, bc + h_vec + max_half };
}

void updateAATrapezoid(AATrapezoid& runtime_shape, const AATrapezoid& rest_shape, Real3 p) 
{
    runtime_shape = rest_shape;
    runtime_shape.base_center += p;
}

DefaultComputeAACapsuleCollisionDefinition(AATrapezoid);
NullTestAABBCollisionDefinition(AATrapezoid);

static constexpr std::array<int, 8*3> AATrapezoidSidesIndices = {
    1, 4, 0,  1, 4, 5,
    5, 2, 1,  5, 2, 6,
    6, 3, 2,  6, 3, 7,
    7, 0, 3,  7, 0, 4,
};

inline std::array<Real3, 8> calculateAATrapezoidVertices(const AATrapezoid& trapz)
{
    auto [up, u, v] = getAxes(trapz.axis);

    std::array<Real3, 8> p; 
    p.fill(trapz.base_center);

    p[0][u] -= trapz.half_extents.x; p[0][v] -= trapz.half_extents.y;
    p[1][u] += trapz.half_extents.x; p[1][v] -= trapz.half_extents.y;
    p[2][u] += trapz.half_extents.x; p[2][v] += trapz.half_extents.y;
    p[3][u] -= trapz.half_extents.x; p[3][v] += trapz.half_extents.y;

    p[4][u] -= trapz.top_half_extents.x; p[4][v] -= trapz.top_half_extents.y;
    p[5][u] += trapz.top_half_extents.x; p[5][v] -= trapz.top_half_extents.y;
    p[6][u] += trapz.top_half_extents.x; p[6][v] += trapz.top_half_extents.y;
    p[7][u] -= trapz.top_half_extents.x; p[7][v] += trapz.top_half_extents.y;

    p[4][up] = p[5][up] = p[6][up] = p[7][up] = trapz.base_center[up] + trapz.height;

    return p;
}

void computeAATrapezoidPrecalc(const AATrapezoid &trapz, AATrapezoidPrecalc &precalc) 
{
    auto [up, u, v]  = getAxes(trapz.axis);

    precalc.vertices = calculateAATrapezoidVertices(trapz);
    
    std::array<Real3, 4> edges_dir = { 
        precalc.vertices[0]-precalc.vertices[4], 
        precalc.vertices[1]-precalc.vertices[5], 
        precalc.vertices[2]-precalc.vertices[6], 
        precalc.vertices[3]-precalc.vertices[7] 
    };

    precalc.sat_axes = {
        GlobalAxes[0],
        GlobalAxes[1],
        GlobalAxes[2],
        glm::normalize(glm::cross(GlobalAxes[u],  edges_dir[0])), // normal of the faces
        glm::normalize(glm::cross(GlobalAxes[v],  edges_dir[1])), // normal of the faces
        glm::normalize(glm::cross(GlobalAxes[u],  edges_dir[2])), // normal of the faces
        glm::normalize(glm::cross(GlobalAxes[v],  edges_dir[3])), // normal of the faces
        glm::normalize(glm::cross(GlobalAxes[up], edges_dir[0])), // normal cross edges 
        glm::normalize(glm::cross(GlobalAxes[up], edges_dir[1]))  // normal cross edges
    };

    const Real3& bc = trapz.base_center;

    auto [min_up, max_up] = std::minmax(precalc.vertices[0][up], precalc.vertices[4][up]);
    precalc.ranges[up] = {min_up, max_up};

    precalc.ranges[u] = {
        bc[u] - std::max(trapz.half_extents.x, trapz.top_half_extents.x),
        bc[u] + std::max(trapz.half_extents.x, trapz.top_half_extents.x)
    };

    precalc.ranges[v] = {
        bc[v] - std::max(trapz.half_extents.y, trapz.top_half_extents.y),
        bc[v] + std::max(trapz.half_extents.y, trapz.top_half_extents.y)
    };

    precalc.ranges[3] = projectPoints(precalc.vertices, precalc.sat_axes[3]);
    precalc.ranges[4] = projectPoints(precalc.vertices, precalc.sat_axes[4]);
    precalc.ranges[5] = projectPoints(precalc.vertices, precalc.sat_axes[5]);
    precalc.ranges[6] = projectPoints(precalc.vertices, precalc.sat_axes[6]);
    precalc.ranges[7] = projectPoints(precalc.vertices, precalc.sat_axes[7]);
    precalc.ranges[8] = projectPoints(precalc.vertices, precalc.sat_axes[8]);
}

CollisionResult computeAABBAATrapezoidCollision(const AABB& aabb, const AATrapezoid& trapz)
{
    ASSERT(trapz.height>0.0f, "AATrapezoid height must be positive");

    AATrapezoidPrecalc precalc;
    computeAATrapezoidPrecalc(trapz, precalc);

    Real3 aabb_center  = (aabb.maxv + aabb.minv) * 0.5f;
    Real3 aabb_extents = (aabb.maxv - aabb.minv) * 0.5f;

    std::array<Range, 9> aabb_ranges = {{
        {aabb.minv[0], aabb.maxv[0]},
        {aabb.minv[1], aabb.maxv[1]},
        {aabb.minv[2], aabb.maxv[2]},
        projectAABB(aabb_center, aabb_extents, precalc.sat_axes[3]), 
        projectAABB(aabb_center, aabb_extents, precalc.sat_axes[4]),
        projectAABB(aabb_center, aabb_extents, precalc.sat_axes[5]),
        projectAABB(aabb_center, aabb_extents, precalc.sat_axes[6]),
        projectAABB(aabb_center, aabb_extents, precalc.sat_axes[7]),
        projectAABB(aabb_center, aabb_extents, precalc.sat_axes[8])
    }};

    return resolveSAT(precalc.sat_axes, aabb_ranges, precalc.ranges);
}

SegmentCollisionResult computeSegmentAATrapezoidCollision(const Segment& seg, const AATrapezoid& trapz)
{
    ASSERT(trapz.height > 0.0f, "AATrapezoid height must be positive");

    auto [up, u, v] = getAxes(trapz.axis);

    auto checkUpPlane = [&](Real plane_height, Real2 extents) -> SegmentCollisionResult 
    {
        if (isBetween(plane_height, seg.start[up], seg.end[up]) == false) return {false};

        Real denom = seg.end[up] - seg.start[up];
        
        if (std::abs(denom) < Epsilon) return {false};

        Real t       = (plane_height - seg.start[up]) / denom;
        Real3 hp     = interpolateSegment(seg, t);
        Real3 hp_rel = hp - trapz.base_center;

        if (std::abs(hp_rel[u]) <= extents.x && std::abs(hp_rel[v]) <= extents.y) return {true, hp};

        return {false};
    };

    Real low_plane_up = trapz.base_center[up];
    Real top_plane_up = trapz.base_center[up] + trapz.height;

    auto resBase = checkUpPlane(low_plane_up, trapz.half_extents);
    if (resBase.is_colliding) return resBase;

    auto resTop = checkUpPlane(top_plane_up, trapz.top_half_extents);
    if (resTop.is_colliding) return resTop;

    // check the sides panels using triangles

    std::array<Real3, 8> vertices = calculateAATrapezoidVertices(trapz);

    return computeSegmentTrianglesCollision(seg, vertices.data(), AATrapezoidSidesIndices.data(), 8);
}

RayCollisionResult computeRayAATrapezoidCollision(const Ray& ray, const AATrapezoid& trapz)
{
    ASSERT(trapz.height > 0.0f, "AATrapezoid height must be positive");

    auto [up, u, v] = getAxes(trapz.axis);

    auto checkUpPlane = [&](Real plane_height, Real2 extents, Real3 normal) -> RayCollisionResult 
    {
        Real denom = ray.direction[up];

        if (std::abs(denom) < Epsilon) return {false};

        Real t = (plane_height - ray.origin[up]) / denom;
        if (t < 0.0f) return {false};

        Real3 hp     = interpolateRay(ray, t);
        Real3 hp_rel = hp - trapz.base_center;

        if (std::abs(hp_rel[u]) <= extents.x && std::abs(hp_rel[v]) <= extents.y) return {true, hp, t, normal};

        return {false};
    };

    ClosestHit closest_hit;

    Real low_plane_up = trapz.base_center[up];
    Real top_plane_up = trapz.base_center[up] + trapz.height;

    if (ray.origin[up] < low_plane_up) 
        closest_hit.updateIfHit(checkUpPlane(low_plane_up, trapz.half_extents, GlobalAxesNeg[up]));
    else if (ray.origin[up] > top_plane_up) 
        closest_hit.updateIfHit(checkUpPlane(top_plane_up, trapz.top_half_extents, GlobalAxes[up]));

    std::array<Real3, 8> vertices = calculateAATrapezoidVertices(trapz);

    closest_hit.updateIfCloser(computeRayTrianglesCollision(ray, vertices.data(), AATrapezoidSidesIndices.data(), 8));

    return closest_hit.toRayCollisionResult();
}

// ==============================================
// Point
// ==============================================

constexpr Real PointAABBExtent = 0.05;

AABB computePointAABB(const Point& shape) 
{ 
    return {-Real3(PointAABBExtent), Real3(PointAABBExtent)};
}

void updatePoint(Point& runtime_shape, const Point& rest_shape, Real3 p) 
{ 
    runtime_shape.p = p;
}

NullComputeAABBCollisionDefinition(Point);
NullComputeAACapsuleCollisionDefinition(Point);
NullTestAABBCollisionDefinition(Point);
NullComputeSegmentCollisionDefinition(Point);

// ==============================================
// AATetrahedron
// ==============================================

AABB computeAATetrahedronAABB(const AATetrahedron& shape) 
{ 
    Real3 cext = shape.corner + shape.extents;
    return {glm::min(shape.corner, cext), glm::max(shape.corner, cext)};
}

void updateAATetrahedron(AATetrahedron& runtime_shape, const AATetrahedron& rest_shape, Real3 p) 
{ 
    runtime_shape = rest_shape;
    runtime_shape.corner += p;
}

CollisionResult computeAABBAATetrahedronCollision(const AABB& aabb, const AATetrahedron& tetra) 
{
    std::array<Real3, 4> tetra_vs = calculateAATetrahedronVertices(tetra);

    std::array<Real3, 7> sat_axes = {
        GlobalAxes[0],
        GlobalAxes[1],
        GlobalAxes[2],
        glm::normalize(glm::cross(tetra_vs[2]-tetra_vs[1], tetra_vs[3]-tetra_vs[1])), 
        glm::normalize(glm::cross(GlobalAxes[2], tetra_vs[1]-tetra_vs[2])),
        glm::normalize(glm::cross(GlobalAxes[0], tetra_vs[2]-tetra_vs[3])),
        glm::normalize(glm::cross(GlobalAxes[1], tetra_vs[3]-tetra_vs[1])),
    };

    AABB tetra_aabb = computeAATetrahedronAABB(tetra);

    std::array<Range, 7> tetra_ranges;

    tetra_ranges[0] = {tetra_aabb.minv[0], tetra_aabb.maxv[0]};
    tetra_ranges[1] = {tetra_aabb.minv[1], tetra_aabb.maxv[1]};
    tetra_ranges[2] = {tetra_aabb.minv[2], tetra_aabb.maxv[2]};
    tetra_ranges[3] = projectPoints(tetra_vs, sat_axes[3]);
    tetra_ranges[4] = projectPoints(tetra_vs, sat_axes[4]);
    tetra_ranges[5] = projectPoints(tetra_vs, sat_axes[5]);
    tetra_ranges[6] = projectPoints(tetra_vs, sat_axes[6]);

    Real3 aabb_center  = (aabb.maxv + aabb.minv) * 0.5f;
    Real3 aabb_extents = (aabb.maxv - aabb.minv) * 0.5f;

    std::array<Range, 7> aabb_ranges = {{
        {aabb.minv[0], aabb.maxv[0]},
        {aabb.minv[1], aabb.maxv[1]},
        {aabb.minv[2], aabb.maxv[2]},
        projectAABB(aabb_center, aabb_extents, sat_axes[3]), 
        projectAABB(aabb_center, aabb_extents, sat_axes[4]),
        projectAABB(aabb_center, aabb_extents, sat_axes[5]),
        projectAABB(aabb_center, aabb_extents, sat_axes[6]),
    }};

    return resolveSAT(sat_axes, aabb_ranges, tetra_ranges);
}

DefaultComputeAACapsuleCollisionDefinition(AATetrahedron);

SegmentCollisionResult computeSegmentAATetrahedronCollision(const Segment& seg, const AATetrahedron& tetra) 
{
    std::array<Real3, 4> vertices = calculateAATetrahedronVertices(tetra);

    return computeSegmentTrianglesCollision(seg, vertices.data(), AATetrahedronFacesIndices.data(), 4);
}

RayCollisionResult computeRayAATetrahedronCollision(const Ray& ray, const AATetrahedron& tetra) 
{ 
    std::array<Real3, 4> vertices = calculateAATetrahedronVertices(tetra);

    return computeRayTrianglesCollision(ray, vertices.data(), AATetrahedronFacesIndices.data(), 4);
}

NullTestAABBCollisionDefinition(AATetrahedron);
