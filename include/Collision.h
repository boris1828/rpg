#pragma once
#include <glm/glm.hpp>
#include <type_traits>

#include "Macro.h"
#include "Mesh.h"
#include "Types.h"

// TOCHECK: if removing these vs code throws untrue errors 
using Real3  = glm::vec3;
using Real   = float;

enum class Axis : uint8_t
{
    X = 0,
    Y = 1,
    Z = 2
};

enum class AxisDir : uint8_t 
{
    Xp, Xn,
    Yp, Yn,
    Zp, Zn
};

struct Range { Real min, max; };

static constexpr Real3 GlobalAxes[3] = { 
    Real3(1.0f, 0.0f, 0.0f), 
    Real3(0.0f, 1.0f, 0.0f), 
    Real3(0.0f, 0.0f, 1.0f) 
};

static constexpr Real3 GlobalAxesNeg[3] = { 
    Real3(- 1.0f, 0.0f, 0.0f), 
    Real3(0.0f, - 1.0f, 0.0f), 
    Real3(0.0f, 0.0f, - 1.0f) 
};

// =======================================================
// Collision Shape Types
// =======================================================


#define COLLISION_TYPES            \
    X(AABB, aabb)                  \
    X(AARamp, aaramp)              \
    X(Ray, ray)                    \
    X(Segment, segment)            \
    X(OBB, obb)                    \
    X(AACylinder, aacylinder)      \
    X(AAHemisphere, aahemisphere)  \
    X(Sphere, sphere)              \
    X(Capsule, capsule)            \
    X(AARectangle, aarect)         \
    X(AACapsule, aacapsule)        \
    X(AxialOBB, axialobb)          \
    X(AA4SidedPyramid, aa4pyramid) \
    X(AAEllipsoid, aaellipsoid)    \
    X(AACone, aacone)              \
    X(AAHollowCylinder, aahollcyl) \
    X(AATrapezoid, aatrapezoid)    \
    X(Point, point)                \
    X(AATetrahedron, aatetra)      \

#define X(type_name, union_member) \
    struct type_name;              \
    struct type_name##Precalc;     \
COLLISION_TYPES
#undef X

// =======================================================
// Shape Structs
// =======================================================

struct AABB 
{
    Real3 minv, maxv;

    inline void expand(Real3 v)
    {
        minv = glm::min(minv, v);
        maxv = glm::max(maxv, v);
    }

    inline void enlarge(Real d)
    {
        minv -= d;
        maxv += d;
    }
};

enum class AARampDirection { PosX, PosY, NegX, NegY };

struct AARamp 
{
    Real3 minv, maxv;
    AARampDirection dir;
};

struct AARectangle
{
    Axis axis;
    Real3 p1, p2;
};

constexpr size_t sizeOfAARecatngle = sizeof(AARectangle);

struct Ray 
{
    Real3 origin, direction;
};

struct Segment 
{
    Real3 start, end;
};

struct FastSegment 
{
    Real3 start;
    Real3 inv_dir; 
};

struct AACylinder
{
    Real3 base_center;
    Real  radius;
    Real  height;
    Axis  axis;
};

struct AAHemisphere
{
    Real3   center;
    Real    radius;
    AxisDir axis;
};

struct Sphere
{
    Real3 center;
    Real  radius;
};

struct OBB 
{
    // TODO: implement OBB structure
};

// TODO: can quantize angle with a uint16_t
struct AxialOBB 
{
    Real3 center;
    Real3 extents;  
    Real  angle;    
    Axis  rotation_axis; 
};

constexpr size_t sizeofAxialOBB = sizeof(AxialOBB);

struct Capsule
{
    Real3 start, end;
    Real radius;
};

struct AACapsule
{
    Axis axis;
    Real3 start;
    Real height;
    Real radius;
};

struct AA4SidedPyramid
{
    Real3 base_center;
    Real2 half_extents;
    Real height;
    Axis axis;
};

constexpr size_t sizeofAA4SidedPyramid = sizeof(AA4SidedPyramid);

struct AAHollowCylinder
{
    Real3 base_center;
    Real inner_radius;
    Real outer_radius;
    Real height;
    Axis axis;
};

struct AAEllipsoid
{
    Real3 center;
    Real3 half_extents;
};

struct AACone
{
    Real3 base_center;
    Real radius;
    Real height;
    Axis axis;
};

struct AATrapezoid
{
    Real3 base_center;
    Real2 half_extents;
    Real2 top_half_extents;
    Real height; 
    Axis axis;
};

constexpr size_t sizeofAATrapezoid = sizeof(AATrapezoid);

struct Point
{
    Real3 p;
};

struct AATetrahedron
{
    Real3 corner;
    Real3 extents;
};

inline std::array<Real3, 4> calculateAATetrahedronVertices(const AATetrahedron& tetra);

// =======================================================
// Precalc Structs
// =======================================================

#define EmptyPrecalc(type_name) struct type_name##Precalc { /* No cached data */ };

EmptyPrecalc(AABB);
EmptyPrecalc(AARamp);
EmptyPrecalc(Ray);
EmptyPrecalc(Segment);
EmptyPrecalc(OBB);
EmptyPrecalc(AACylinder);
EmptyPrecalc(AAHemisphere);
EmptyPrecalc(Sphere);
EmptyPrecalc(Capsule);
EmptyPrecalc(AARectangle);
EmptyPrecalc(AACapsule);
EmptyPrecalc(AxialOBB);
EmptyPrecalc(AA4SidedPyramid);
EmptyPrecalc(AAEllipsoid);
EmptyPrecalc(AACone);
EmptyPrecalc(AAHollowCylinder);
EmptyPrecalc(Point);
EmptyPrecalc(AATetrahedron);

struct AATrapezoidPrecalc
{
    std::array<Real3, 8> vertices;
    std::array<Real3, 9> sat_axes;
    std::array<Range, 9> ranges;
};

constexpr size_t sizeofAATrapezoidPrecalc = sizeof(AATrapezoidPrecalc);

constexpr size_t sizeofAABBPrecalc = sizeof(AABBPrecalc);

// =======================================================
// Collision Shape API
// =======================================================

#define X(type_name, union_name) \
ASSERT_STRUCT_IS_DEFINED(type_name); ASSERT_STRUCT_IS_DEFINED(type_name##Precalc);
COLLISION_TYPES
#undef X

enum class CollisionShapeType : uint16_t
{
    #define X(type_name, union_member) type_name,
    COLLISION_TYPES
    #undef X
};

using PrecalcIndex = uint16_t;
constexpr PrecalcIndex nullPrecalcIndex = std::numeric_limits<PrecalcIndex>::max();

struct CollisionShape
{
    CollisionShapeType type;
    PrecalcIndex precalc_idx = nullPrecalcIndex;
    union
    {
        #define X(type_name, union_member) type_name union_member;
        COLLISION_TYPES
        #undef X
    };
};

constexpr size_t sizeofCollisionShape = sizeof(CollisionShape);

static const char* getCollisionShapeTypeName(CollisionShapeType type) 
{
    switch(type) 
    {
        #define X(TypeName, UnionName) case CollisionShapeType::TypeName: return #TypeName;
        COLLISION_TYPES
        #undef X
        default: return "Unknown";
    }
}

#define X(type_name, union_member) AABB compute ##type_name ##AABB(const type_name & union_member);
COLLISION_TYPES
#undef X

AABB computeCollisionShapeAABB(const CollisionShape& coll_shape);

#define X(type_name, union_member) void update ##type_name (type_name & union_member, const type_name & rest_##union_member, Real3 p);
COLLISION_TYPES
#undef X

void updateCollisionShape(CollisionShape& runtime_shape, const CollisionShape& rest_shape, const Real3& pos, const Real3& prev_pos);
void updateCollisionShape(CollisionShape& runtime_shape, const CollisionShape& rest_shape, const Real3& pos);

void updateSegment(Segment& seg, Real3 p1, Real3 p2);

#define X(type_name, union_member) void compute ##type_name ##Precalc(const type_name & shape, type_name ##Precalc & precalc);
COLLISION_TYPES
#undef X

// =======================================================
// Precalc Holder
// =======================================================

template<typename T>
struct PrecalcPool
{
    std::vector<T> data;
    PrecalcIndex size;
    CollisionShapeType type;
    const char* type_name;

    PrecalcPool() : size(0) { data.reserve(10); }

    void setType(CollisionShapeType t) 
    { 
        type = t; 
        type_name = getCollisionShapeTypeName(t);
    }

    PrecalcIndex add(const T& precalc)
    {
        ASSERT(size < std::numeric_limits<PrecalcIndex>::max(), std::string("PrecalcPool overflow for type: ") + type_name);
        data.push_back(precalc);
        return size++;
    }

    const T& get(PrecalcIndex idx) const
    {
        ASSERT(idx < size, std::string("invalid index for Precalc: ") + type_name);
        return data[idx];
    }
};

template<typename T>
requires (sizeof(T) == 1 && std::is_empty_v<T>)
struct PrecalcPool<T>
{
    CollisionShapeType type;

    PrecalcPool() {}

    void setType(CollisionShapeType t) { type = t; }

    PrecalcIndex add(const T& precalc)
    {
        const char* type_name = getCollisionShapeTypeName(type);
        ASSERT(false, std::string("Cannot add to empty PrecalcPool for type: ") + type_name);
        return 0;
    }

    const T& get(PrecalcIndex idx) const
    {
        const char* type_name = getCollisionShapeTypeName(type);
        ASSERT(false, std::string("Cannot get from empty PrecalcPool for type: ") + type_name);
        static T dummy;
        return dummy;
    }
};

struct CollisionShapePrecalcData
{
private:
    #define X(type_name, union_member) PrecalcPool<type_name##Precalc> pool_##type_name;
    COLLISION_TYPES
    #undef X

    CollisionShapePrecalcData()
    {
        #define X(type_name, union_member) pool_##type_name.setType(CollisionShapeType::type_name);
        COLLISION_TYPES
        #undef X
    }

public:
    CollisionShapePrecalcData(const CollisionShapePrecalcData&)            = delete;
    CollisionShapePrecalcData& operator=(const CollisionShapePrecalcData&) = delete;
    CollisionShapePrecalcData(CollisionShapePrecalcData&&)                 = delete;
    CollisionShapePrecalcData& operator=(CollisionShapePrecalcData&&)      = delete;

    static CollisionShapePrecalcData& instance()
    {
        static CollisionShapePrecalcData instance;
        return instance;
    }

    template<typename T>
    PrecalcPool<T>& getPool();

    #define X(type_name, union_member) \
        template<> PrecalcPool<type_name##Precalc>& getPool<type_name##Precalc>() { return pool_##type_name; }
    COLLISION_TYPES
    #undef X
};

inline bool isValid(PrecalcIndex idx) { return idx != nullPrecalcIndex; }

// =======================================================
// Precalc Holder
// =======================================================

// =======================================================
// Utilities
// =======================================================

inline Real  rmin(Real a, Real b);
inline Real  rmax(Real a, Real b);
inline Real3 rmin(const Real3& a, const Real3& b);
inline Real3 rmax(const Real3& a, const Real3& b);

inline AABB  AABBUnion(const AABB& b1, const AABB& b2);
inline Real  AABBSurfaceArea(const AABB& a);
inline bool  AABBOverlap(const AABB& a, const AABB& b);
Real3 AABBCenter(const AABB& aabb);

inline Real2 project(const Real3& v, Axis axis);
inline Real3 unproject(const Real2& v, Axis axis);

struct AxisIndices { int up, u, v; };
constexpr AxisIndices getAxes(Axis axis);

// =======================================================
// AABB Tree 
// =======================================================

#define QUERY_OVERLAP_MAX_RESULTS 8

template <typename T>
struct QueryOverlapResult 
{
    T data[QUERY_OVERLAP_MAX_RESULTS];
    Size count = 0;

    void add(const T& d) 
    {
        if (count < QUERY_OVERLAP_MAX_RESULTS) 
        {
            data[count++] = d;
        }
        else 
        {
            ASSERT(false, "QueryOverlapResult out of space");
        }
    }

    void clear() 
    {
        count = 0;
    }

    Size size() const { return count; }
    bool empty() const { return count == 0; }

    T& operator[](Size i) {
        ASSERT(i < count, "Out of bounds");
        return data[i];
    }

    const T& operator[](Size i) const {
        ASSERT(i < count, "Out of bounds");
        return data[i];
    }

    T* begin()             { return data; }
    T* end()               { return data + count; }
    const T* begin() const { return data; }
    const T* end()   const { return data + count; }
};

using Index = int16_t;

#define AABB_TREE_MAX_OBJECTS 1024
#define nullAABBTreeIndex     -1

struct StaticAABBTreeNodeData 
{
    Entity   entity; 
    uint16_t object_idx;  
};

struct Node 
{
    AABB    box;
    int16_t parent_idx;

    union 
    {
        struct 
        {
            int16_t child1;
            int16_t child2;
        };
        StaticAABBTreeNodeData object_data;
    };

    bool is_leaf;
};

struct AABBTree 
{
    Node nodes[AABB_TREE_MAX_OBJECTS];
    int node_count;
    int root_idx;
};

constexpr size_t sizeOfAABBTree = sizeof(AABBTree);

// Stack used for pickBestSibling()
struct Stack 
{
    int data[AABB_TREE_MAX_OBJECTS];
    int top;
    Stack() : top(-1) {}
    void push(int v) { data[++top] = v; }
    int  pop()       { return data[top--]; }
    bool empty()     { return top == -1; }
    void clear()     {        top =  -1; }
};

// global stack declaration
extern Stack aabb_tree_stack;

// ---- AABB Tree functions ----
int allocateLeafNode(AABBTree& tree, AABB box, StaticAABBTreeNodeData object_data);
int allocateInternalNode(AABBTree& tree);

// external interface
AABBTree createAABBTree();

AABBTree constuctStaticAABBTree(std::vector<AABB> &boxs, std::vector<StaticAABBTreeNodeData> &data);
void printAABBTree(const AABBTree& tree, int node_idx, int depth = 0);

void queryOverlap(const AABBTree& tree, const AABB& box, QueryOverlapResult<StaticAABBTreeNodeData>& results);
void queryOverlap(const AABBTree& tree, const Ray& ray,  QueryOverlapResult<StaticAABBTreeNodeData>& results);

// =======================================================
// Dynamic AABB Tree 
// =======================================================

template <typename T, std::size_t N>
struct FreeList 
{
    T data[N];
    bool used[N]       = { false };
    Index last_checked = 0;
    Size num_nodes     = 0;

    Index allocate() 
    {
        for (Index i = 0; i < N; ++i) 
        {
            Index idx = (last_checked + i) % N;
            if (!used[idx]) 
            {
                used[idx] = true;
                last_checked = (idx + 1) % N;
                num_nodes++;
                return idx;
            }
        }
        ASSERT(false, "FreeList out of space");
        return -1;
    }

    void deallocate(Index idx) 
    {
        ASSERT(idx >= 0 && idx < static_cast<Index>(N), "Invalid FreeList index");
        ASSERT(used[idx], "Double free detected in FreeList");
        used[idx] = false;
        num_nodes--;
    }

    T& operator[](Index idx) 
    {
        if (!used[idx]) 
        {
            std::cout << "FreeList access error at index " << idx << std::endl;
        }
        ASSERT(idx >= 0 && idx < static_cast<Index>(N), "Invalid FreeList index");
        ASSERT(used[idx], "Accessing unallocated FreeList index");
        return data[idx];
    }

    const T& operator[](Index idx) const
    {
        if (!used[idx]) 
        {
            std::cout << "FreeList access error at index " << idx << std::endl;
        }
        ASSERT(idx >= 0 && idx < static_cast<Index>(N), "Invalid FreeList index");
        ASSERT(used[idx], "Accessing unallocated FreeList index");
        return data[idx];
    }
};

struct DynamicAABBTreeNodeData 
{
    Entity entity;
    Index  leaf_idx;
};

struct DynamicAABBTreeNode 
{
    AABB  box;
    Index parent_idx;

    union 
    { 
        struct
        {
            Index child1;
            Index child2;
        };
        DynamicAABBTreeNodeData data;
    };
    
    bool  is_leaf;
    Real inherited_cost;
};

struct DynamicAABBTree
{
    FreeList<DynamicAABBTreeNode, AABB_TREE_MAX_OBJECTS> nodes;
    Index root_idx;

    bool indexIsValid(Index idx) const 
    {
        return idx != nullAABBTreeIndex && idx >= 0 && idx < static_cast<Index>(AABB_TREE_MAX_OBJECTS) && nodes.used[idx];
    }
};

constexpr size_t sizeOfDynamicAABBTreeNodeData = sizeof(DynamicAABBTreeNodeData);
constexpr size_t sizeOfDynamicAABBTreeNode     = sizeof(DynamicAABBTreeNode);
constexpr size_t sizeOfDynamicAABBTree         = sizeof(DynamicAABBTree);

Index dynamicAABBTreeAllocateLeafNode(DynamicAABBTree& tree, AABB box, DynamicAABBTreeNodeData data);
Index dynamicAABBTreeAllocateInternalNode(DynamicAABBTree& tree);
Index pickBestSibling(DynamicAABBTree& tree, Index leaf_idx);

// external interface
DynamicAABBTree createDynamicAABBTree();

Index dynamicAABBTreeInsertLeaf(DynamicAABBTree& tree, AABB box, Entity entity);
void  dynamicAABBTreeRemoveLeaf(DynamicAABBTree& tree, Index leaf_idx);
Index dynamicAABBTreeUpdateLeaf(DynamicAABBTree& tree, Index leaf_idx, AABB new_box);

void queryOverlap(const DynamicAABBTree& tree, const AABB& box, QueryOverlapResult<DynamicAABBTreeNodeData>& results);
void queryOverlap(const DynamicAABBTree& tree, const Ray& ray,  QueryOverlapResult<DynamicAABBTreeNodeData>& results);

// =======================================================
// Collision Testing/Computing API
// =======================================================

struct SegmentCollisionResult 
{
    bool  is_colliding;
    Real3 entering_point;
};

struct RayCollisionResult 
{
    bool  is_colliding;
    Real3 entering_point;
    Real  t;
    Real3 contact_normal = GlobalAxes[2];
};

struct CollisionResult 
{
    bool  is_colliding;
    Real3 mtv;
};

/*
struct PointCollisionResult
{
    bool is_colliding;
    Real3 mtv;
    Real mtv_length;
};
*/

// =======================================================
// AABB / AACapsule first collision functions
// =======================================================

#define X(type_name, union_member) CollisionResult computeAACapsule ##type_name ##Collision(const AACapsule& , const type_name & );
COLLISION_TYPES
#undef X

CollisionResult computeAACapsuleShapeCollision(const AACapsule& aacaps, const CollisionShape& shape);

#define X(type_name, union_member) CollisionResult computeAABB ##type_name ##Collision(const AABB& , const type_name & );
COLLISION_TYPES
#undef X

CollisionResult computeAABBShapeCollision(const AABB& aabb, const CollisionShape& shape);

#define X(type_name, union_member) bool testAABB ##type_name ##Collision(const AABB& , const type_name & );
COLLISION_TYPES
#undef X

bool testAABBShapeCollision(const AABB& aabb, const CollisionShape& shape);

// =======================================================
// Ray first collision functions
// =======================================================

#define X(type_name, union_member) RayCollisionResult computeRay ##type_name ##Collision(const Ray& , const type_name & );
COLLISION_TYPES
#undef X

RayCollisionResult computeRayShapeCollision(const Ray& ray, const CollisionShape& coll_shape);

// =======================================================
// Segment first collision functions
// =======================================================

SegmentCollisionResult computeSegmentShapeCollision(const Segment& seg, const CollisionShape& coll_shape);

#define X(type_name, union_name) SegmentCollisionResult computeSegment ##type_name ##Collision(const Segment& , const type_name & );
COLLISION_TYPES
#undef X

bool computeFastSegmentsAABBCollision(const FastSegment* segs, size_t num_segments, const AABB& aabb);

// =======================================================