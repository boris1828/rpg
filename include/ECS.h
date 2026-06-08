#pragma once

#include <stdint.h>
#include <unordered_map>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <set>
#include <vector>
#include <filesystem>
#include <map>
#include <array>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <source_location>
#include <optional>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Mesh.h"
#include "InputManager.h"
#include "Collision.h"
#include "Macro.h"
#include "Types.h"
#include "Render.h"
#include "Utility.h"
#include "Component.h"

struct Registry;
enum class ComponentPoolID : uint8_t;
enum class ComponentMapID  : uint8_t;

template<typename E>
constexpr std::enable_if_t<std::is_enum_v<E>, std::size_t>
toIndex(E value)
{
    return static_cast<std::size_t>(value);
}

// MESH MANAGER

enum class AnimState : uint8_t { idle, moving, COUNT };

static constexpr size_t AnimStateCount = static_cast<size_t>(AnimState::COUNT);

struct AnimStateInfo
{   
    Mesh*     mesh;
    AnimState state;
    uint8_t   fps;
    uint8_t   step_per_frame;
};

struct AnimationStateMeshMap
{
    AnimStateInfo states[static_cast<int>(AnimState::COUNT)] = {0};
    bool used[static_cast<int>(AnimState::COUNT)]            = {false};
    AnimState default_state = AnimState::idle;

    const AnimStateInfo& get(AnimState state) const
    {
        const size_t idx = toIndex(state);

        ASSERT(idx < AnimStateCount, "AnimState out of bounds");
        ASSERT(used[idx],            "Animation state not defined in AnimationStateMeshMap");

        return states[idx];
    }

    const AnimStateInfo& getDefault() const 
    {
        return get(default_state);
    }

    void insert(AnimState state, Mesh* mesh, uint8_t fps)
    {
        const size_t idx = toIndex(state);

        ASSERT(idx < AnimStateCount, "AnimState out of bounds");
        ASSERT(mesh != nullptr,      "Cannot insert null mesh into AnimationStateMeshMap");
        ASSERT(!used[idx],           "Animation state already defined (duplicate insert)");
        ASSERT(fps > 0,              "FPS must be greater than zero");

        ASSERT(mesh->m_type == MeshType::animated, "Mesh used in animation state must be of type animated");

        // ASSERT(60 % fps == 0, "FPS must divide game tick rate (60) evenly"); // TOCHECK: might be useful

        AnimStateInfo& info = states[idx];
        info.mesh           = mesh;
        info.state          = state;
        info.fps            = fps;
        info.step_per_frame = 60 / fps;

        used[idx] = true;
    }

    bool hasState(AnimState state) const
    {
        const size_t idx = toIndex(state);

        ASSERT(idx < AnimStateCount, "AnimState out of bounds");

        return used[idx];
    }

    // TODO: animation state meshe: add fallback state to get to when unsopported or unknown state (should avoid to enter undefined state though)
};

#define CYLINDER_COLLISION_SHAPE_SEGMENTS 30
#define SPHERE_COLLISION_SHAPE_SEGMENTS 30

struct MeshManager 
{
    std::vector<Mesh*>           meshes;
    std::map<std::string, Mesh*> name_mesh_map;

    Mesh status_bar_mesh;

    Mesh x_wfmesh;
    Mesh zaahemisphere_wfmesh;
    Mesh sphere_wfmesh;
    Mesh zaacapsule_wfmesh;
    Mesh zaacylinder_wfmesh;
    Mesh zacapsulebody_wfmesh;
    Mesh aabb_wfmesh;
    Mesh centered_aabb_wfmesh;
    Mesh zarect_wfmesh;
    Mesh za4pyramid_wfmesh;
    Mesh zacone_wfmesh;
    Mesh xysquare_wfmesh;

    DebugLineRenderer debug_line;

    AnimationStateMeshMap player_state_mesh_map;

    MeshManager() : 
        status_bar_mesh(loadOBJStatic("status_bar.obj")),
        x_wfmesh(createXWireframeMesh()), 
        zaahemisphere_wfmesh(createZAAHemisphereWireframeMesh(SPHERE_COLLISION_SHAPE_SEGMENTS)),
        sphere_wfmesh(createSphereWireframeMesh(SPHERE_COLLISION_SHAPE_SEGMENTS)), 
        zaacapsule_wfmesh(createZAACapsuleWireframeMesh(SPHERE_COLLISION_SHAPE_SEGMENTS)),
        zaacylinder_wfmesh(createZAlignedCylinderWireframeMesh(CYLINDER_COLLISION_SHAPE_SEGMENTS)), 
        zacapsulebody_wfmesh(createZAlignedCapsuleBodyWireframeMesh(8)),
        aabb_wfmesh(createAABBWireframeMesh()), 
        centered_aabb_wfmesh(createCenteredAABBWireframeMesh()),
        zarect_wfmesh(createZARectangleWireframeMesh()),
        za4pyramid_wfmesh(createZA4SidedPyramidWireframeMesh()), 
        zacone_wfmesh(createZAConeWireframeMesh(CYLINDER_COLLISION_SHAPE_SEGMENTS)), 
        xysquare_wfmesh(createXYSquareWireframeMesh())
    {};

    Mesh* loadMesh(const std::string& path) 
    {
        Mesh* mesh = loadOBJ(path);

        std::filesystem::path p(path);
        std::string name = p.stem().string();

        ASSERT(p.has_extension(), "File has no extension");
        ASSERT(p.extension() == ".obj", "File extension must be .obj");
        ASSERT(mesh != nullptr, "Mesh should be loaded correctly");

        meshes.push_back(mesh);
        name_mesh_map[name] = mesh;

        return mesh;
    }

    Mesh* loadAnimatedMesh(const std::string &directory_path, const std::string &prefix, FrameIndex num_frames) 
    {
        Mesh* mesh = loadAnimatedOBJ(directory_path, prefix, num_frames);

        //TODO: assertion: add assertion on number of files, extensione etc...
        ASSERT(mesh != nullptr, "Mesh should be loaded correctly");

        meshes.push_back(mesh);
        name_mesh_map[directory_path] = mesh;

        return mesh;
    }

    Mesh* operator[](const std::string& name)
    {   
        ASSERT(name_mesh_map.find(name) != name_mesh_map.end(), "Namd mesh couldn't be find");
        return name_mesh_map[name];
    }

    size_t addMesh(Mesh *mesh) 
    {
        meshes.push_back(mesh);
        return meshes.size()-1;
    }

    ~MeshManager() 
    {
        for (Mesh* mesh : meshes) 
        {
            delete mesh;
        }
    }
};

// SHADER MANAGER

#define NUM_SHADERS 9

struct ShaderManager
{
    Shader def_shader;
    Shader wireframe_shader;
    Shader particle_shader;
    Shader snow_shader;
    Shader fog_shader;
    Shader trajectory_shader;
    Shader shooting_range_shader;
    Shader effect_sphere_shader;
    Shader digital_grid_shader;

    Shader* all_shaders[NUM_SHADERS];

    ShaderManager() :
        def_shader({{GL_VERTEX_SHADER, "basic.vert"}, {GL_FRAGMENT_SHADER, "basic.frag"}}), 
        wireframe_shader({{GL_VERTEX_SHADER, "wireframe.vert"}, {GL_FRAGMENT_SHADER, "wireframe.frag"}}),
        particle_shader({{GL_VERTEX_SHADER, "particle.vert"}, {GL_GEOMETRY_SHADER, "particle.geom"}, {GL_FRAGMENT_SHADER, "particle.frag"}}),
        snow_shader({{GL_VERTEX_SHADER, "snow.vert"}, {GL_GEOMETRY_SHADER, "snow.geom"}, {GL_FRAGMENT_SHADER, "snow.frag"}}),
        fog_shader({{GL_VERTEX_SHADER, "fog.vert"}, {GL_FRAGMENT_SHADER, "fog.frag"}}), 
        trajectory_shader({{GL_VERTEX_SHADER, "trajectory.vert"}, {GL_GEOMETRY_SHADER, "trajectory.geom"}, {GL_FRAGMENT_SHADER, "trajectory.frag"}}), 
        shooting_range_shader("shooting_range"), 
        effect_sphere_shader("effect_sphere"), 
        digital_grid_shader("digital_grid")
    {
        def_shader.use();
        def_shader.setVec3("lightDir", glm::normalize(Real3(-1.0f, -1.0f, -1.0f)));
        def_shader.setFloat("ambientStrength", 0.3f);
        def_shader.setFloat("diffuseStrength", 0.7f);
        def_shader.setFloat("uAlpha", 1.0f);

        all_shaders[0] = &def_shader;
        all_shaders[1] = &wireframe_shader; 
        all_shaders[2] = &particle_shader; 
        all_shaders[3] = &snow_shader;
        all_shaders[4] = &fog_shader;
        all_shaders[5] = &trajectory_shader;
        all_shaders[6] = &shooting_range_shader;
        all_shaders[7] = &effect_sphere_shader;
        all_shaders[8] = &digital_grid_shader;
    }

    void setViewProjection(const glm::mat4& view, const glm::mat4& projection)
    {
        for (int i=0; i<NUM_SHADERS; i++)
        {
            all_shaders[i]->use();
            all_shaders[i]->setMat4("view", view);
            all_shaders[i]->setMat4("projection", projection);
        }
    }  
};

constexpr std::size_t sizeOfShaderManager = sizeof(ShaderManager);

// COMPONENTs

#define ENTITY_TYPE_LIST(V)  \
    V(Object)                \
    V(Projectile)            \
    V(IdleProjectile)        \
    V(Camera)                \
    V(Enemy)                 \
    V(NPC)                   \
    V(Door)                  \
    V(StatusChangeArea)      \
    V(HouseLayerToggle)      \
    V(House)                 \
    V(PoolElementDestructor) \
    V(Other)                 \

enum class EntityType 
{
#define AS_ENUM(name) name,
    ENTITY_TYPE_LIST(AS_ENUM)
#undef AS_ENUM
};

inline const char* toString(EntityType type) 
{
    switch (type) {
#define AS_STRING(name) case EntityType::name: return #name;
        ENTITY_TYPE_LIST(AS_STRING)
#undef AS_STRING
        default: ASSERT(false, "invalid entity type");
    }
}

// =======================================================
// PhysicsBody
// =======================================================

// =======================================================
// Physics Types
// =======================================================

enum class CollisionObjectType 
{
    staticObject,
    staticStair,
    dynamic,
    projectile,
    kynetic
};

enum class PhysicsBodyType 
{
    noPhysics,
    dynamic,
    particle,
    rigidBody
};

struct PhysicsBody 
{
    Real3 position;
    Real3 prev_position;
    Real3 velocity;
    Real3 angular_velocity;
    Real3 orientation;      // euler

    Real  move_speed;
    Real3 move_intent;

    CollisionShape      rest_shape;
    CollisionShape      runtime_shape;
    CollisionObjectType objectType;
    PhysicsBodyType     physicsType;

    Index aabb_tree_idx = nullAABBTreeIndex;

    Real3 accumulated_correction = Real3(0.0f);
    bool has_collided            = false;
};

struct StaticPhysicsBody
{ 
    CollisionObjectType objectType;
    CollisionShape      runtime_shape;
    Index aabb_tree_idx = nullAABBTreeIndex;
    bool visible        = true;
};

template <typename T>
concept ValidStaticBody = requires(T obj) 
{
    { obj.runtime_shape } -> std::convertible_to<const CollisionShape&>;
    { obj.objectType }    -> std::convertible_to<CollisionObjectType>;
};

// =======================================================
// END PhysicsBody
// =======================================================

// =======================================================
// Constraint
// =======================================================

enum class ConstraintType : uint8_t
{
    Distance,    // Maintains a specific length
    MaxDistance, // Allows Slack (like a rope)
    MinDistance  // Keeps things away
};

struct Constraint 
{
    // static constexpr Real arrivalThreshold = 0.2;

    ConstraintType type; 
    Real rest_length;   
    Real stiffness;  

    Real3 anchor; 
    Real  life_time;
    bool  is_static_point;

    // bool   destroy_on_closeness;
    Entity entity1, entity2; 
};

constexpr std::size_t sizeOfConstraint = sizeof(Constraint);


// =======================================================
// END Constraint
// =======================================================

// =======================================================
// Rendering Components
// =======================================================

struct RenderInfo 
{ 
    Mesh* mesh; 
    Real3 color; 
    Real  alpha; 

    uint8_t fps; // TODO: types: make fps type
    uint8_t currframe_played_steps;

    bool visible = true;

    void reset() { alpha=1.0f; }
};

// TODO: particle renderer: should have one particle renderer object here only values to inject to it in draw call
struct ParticleRendererInfo
{
    ParticleSystem particle_system;
};

inline void destroyParticleRendererInfo(ParticleRendererInfo &particle_renderer_info)
{
    particle_renderer_info.particle_system.destroy();
}

struct AnimationState
{
    AnimState              current_state;
    AnimState              previous_state;
    AnimationStateMeshMap* state_mesh_map;
    uint8_t                currframe_played_steps;
};

// =======================================================
// END Rendering Components
// =======================================================

struct LifeTime 
{ 
    float life; 
};

#define CROSSHAIR_LINE_SEGMENTS 8

struct CrosshairInfo 
{ 
    Real3  position;
    Real3  normal;
    Entity target; 
    float  z_offset; 
    Mesh*  mesh; 
    Real3  color;
    bool   valid_target;
};

struct TrajectoryInfo
{
    bool  valid;
    Real3 x0;
    Real3 direction;
    Real  v0;
    Real  te;
    Real  g;
};

struct CameraInfo 
{ 
    glm::vec3 position;
    glm::vec3 offset; 
    Entity target; 
    glm::mat4 view; 
    glm::mat4 projection; 
    float max_distance; 
    float vel; 
};

struct Attachment  
{ 
    Entity target; 
    Real3  position_attach; 
    Real3  starting_target_orientation;
    Real3  starting_attach_orientation;
};

struct PathFollower 
{
    std::vector<Real3> points; 
    size_t current_index = 0;      
    float  speed         = 1.0f;            
    float  threshold     = 0.1f;        
    bool   loop          = true;
};

struct RandomDirectionMover 
{
    Real  change_interval;
    Real  timer;
    Real3 direction;
};

struct SimplePosition 
{
    Real3 position;
    Real3 orientation;
};

// =======================================================
// Projectile Component
// =======================================================

enum class ProjectileEjectionMode : uint8_t
{
    Shooting, 
    Throwing
};

enum class ProjectileTrajectoryType : uint8_t 
{
    Parabolic,   
    Straight3D,
    Straight2D   
};

enum class ProjectileOrientationMode : uint8_t 
{
    AlignWithVelocity,
    Spinning
    // Billboard	    always faces the player/camera, 
    // LookAtTarget     always points toward a specific entity or coordinate
};

struct ProjectileTypeTrait
{
    ProjectileEjectionMode    ejection_mode; 
    ProjectileOrientationMode orientation_mode;
    ProjectileTrajectoryType  trajectory_type;
    const char* mesh_name;
};

enum class ProjectileType : uint8_t
{
    Arrow, 
    Potion, 
    Hook, 
    COUNT
};

constexpr ProjectileTypeTrait get_trait(ProjectileType type) 
{
    switch (type) 
    {
        case ProjectileType::Arrow: return { 
            .ejection_mode    = ProjectileEjectionMode::Shooting, 
            .orientation_mode = ProjectileOrientationMode::AlignWithVelocity,
            .trajectory_type  = ProjectileTrajectoryType::Parabolic,
            .mesh_name        = "projectile" 
        };
        case ProjectileType::Potion: return { 
            .ejection_mode    = ProjectileEjectionMode::Throwing,
            .orientation_mode = ProjectileOrientationMode::Spinning, 
            .trajectory_type  = ProjectileTrajectoryType::Parabolic,
            .mesh_name        = "potion"
        };
        case ProjectileType::Hook: return { 
            .ejection_mode    = ProjectileEjectionMode::Shooting, 
            .orientation_mode = ProjectileOrientationMode::AlignWithVelocity,
            .trajectory_type  = ProjectileTrajectoryType::Straight3D,
            .mesh_name        = "projectile" 
        };
        default: return { 
            .ejection_mode    = ProjectileEjectionMode::Shooting, 
            .orientation_mode = ProjectileOrientationMode::AlignWithVelocity,
            .trajectory_type  = ProjectileTrajectoryType::Parabolic,
            .mesh_name        = "projectile" 
        };
    }
}

constexpr ProjectileTypeTrait projectile_type_traits[] = {
    get_trait(ProjectileType::Arrow),
    get_trait(ProjectileType::Potion),
    get_trait(ProjectileType::Hook)
};

inline const ProjectileTypeTrait& getTrait(ProjectileType type) {
    return projectile_type_traits[toIndex(type)];
}

struct ProjectileInfo 
{
    ProjectileType type;
    Entity shooter;
    Real3  init_velocity;
    Real3  init_position;
    Real   time_passed = 0.0f;

    Real3  position;
    Real3  prev_position;
    Real3  orientation;
    Real3  velocity;
    CollisionShape runtime_shape;
};

static_assert(std::size(projectile_type_traits) == toIndex(ProjectileType::COUNT), "Missing projectile trait definition in the lookup table.");
constexpr std::size_t sizeOfProjectileInfo = sizeof(ProjectileInfo);
constexpr std::size_t sizeOfprojectile_type_traits = sizeof(projectile_type_traits);

// =======================================================
// END Projectile Component
// =======================================================

struct EnemyInfo
{
    bool aim_locked;

    void reset() { aim_locked = false; }
};

// =======================================================
// Status AND Effect
// =======================================================

enum class EffectType : uint8_t 
{
    Burnt, Poisoned, Healing, Freezed, Slowed, COUNT
};

static constexpr size_t EffectTypeCount = static_cast<size_t>(EffectType::COUNT);

struct Effect
{
    EffectType type;
    bool active;
    bool on_tick;
    Real strength;
    Real tick_length;
    Real tick_timer;
    Real duration;
    Real timer;
};

struct Status
{
    Real life;
    Real max_life;

    Effect effects[EffectTypeCount];

    Status() = default;

    Status(Real max_life) : 
        life(max_life),
        max_life(max_life)
    {
        for (int i=0; i<EffectTypeCount; i++)
        {
            Effect effect;
            effect.type        = (EffectType) i; 
            effect.active      = false;
            effect.on_tick     = false;
            effect.strength    = 0.0;
            effect.tick_length = 0.0;
            effect.tick_timer  = 0.0;
            effect.duration    = 0.0;
            effect.timer       = 0.0;

            effects[i] = effect;
        }
    }

    bool isEffectActive(EffectType type) const
    {
        return effects[toIndex(type)].active;
    }

    void activateEffect(const Effect& effect_template)
    {
        ASSERT(toIndex(effect_template.type) < EffectTypeCount, "EffectType out of bounds");
        ASSERT(!effects[toIndex(effect_template.type)].active, "cant activate an effect already active");

        Effect& effect = effects[toIndex(effect_template.type)];

        effect.active      = true;
        effect.type        = effect_template.type;
        effect.on_tick     = effect_template.on_tick;
        effect.strength    = effect_template.strength;
        effect.tick_length = effect_template.tick_length;
        effect.duration    = effect_template.duration;
        effect.timer       = 0.0f;
        effect.tick_timer  = 0.0f;
    }
};

using EffectApplyFn = void(*)(Registry&, Entity, Effect&);

void applyBurn(Registry&,   Entity, Effect&);
void applyPoison(Registry&, Entity, Effect&);
void applyHeal(Registry&,   Entity, Effect&);
void applyFreeze(Registry&, Entity, Effect&);
void applySlow(Registry&,   Entity, Effect&);

struct EffectDef
{
    EffectApplyFn apply;
    bool on_tick;
};

constexpr EffectDef EFFECT_DEFS[EffectTypeCount] = {
    { applyBurn,   true  }, /* Burnt    */ 
    { applyPoison, true  }, /* Poisoned */ 
    { applyHeal,   true  }, /* Healing  */ 
    { applyFreeze, false }, /* Freezed  */ 
    { applySlow,   false }  /* Slowed   */ 
};

// =======================================================
// END Status AND Effect
// =======================================================

// =======================================================
// Interactable
// =======================================================

enum class TriggerMode : uint8_t
{
    OnAreaFirstEnter, // First ever frame of overlap
    OnAreaEnter,      // First frame of overlap, can be exited and re-entered
    OnAreaExit,       // Frame overlap ceases
    OnAreaStay,       // Every frame of overlap (Continuous)
    OnAreaEnterAndExit, 
    // OnAreaTick,    // Periodically while overlapping (Per-Entity Timer)
    OnGlobalTick,     // Periodically while overlapping (Global timer)
    OnClick,          // Clicked while hovered/pointed
    OnHover           // While mouse is over (UI feedback)
};

enum class TriggerAction : uint8_t
{
    Enter, Exit, Stay, None
};

enum class InteractionType 
{
    Dialogue,
    Loot,
    StatusChange,
    HouseLayerToggle, 
    PoolElementDestruction, 
    Door
};

// POOL ELEMENT DESTROYER

struct PoolDestructionInfo
{
    ComponentPoolID type_id;
    PoolIndex idx;
};

// HOUSE

#define HOUSE_LAYER_MAX_ENTITIES 16
#define HOUSE_MAX_LAYERS 4

struct HouseLayer
{
    Entity house_entity;
    uint8_t floor_level;
};

struct HouseLayerInfo
{
    SmallSet<uint16_t, HOUSE_LAYER_MAX_ENTITIES> static_phys_bodies;
    SmallSet<Entity,   HOUSE_LAYER_MAX_ENTITIES> static_entities;
    SmallSet<Entity,   HOUSE_LAYER_MAX_ENTITIES> dynamic_entities;
    bool hidden = false;
};

struct House
{
    uint8_t size = 0;
    std::array<HouseLayerInfo, HOUSE_MAX_LAYERS> layers;

    uint8_t trigger_layer_size = 0;
    std::array<Entity, HOUSE_MAX_LAYERS> layer_trigger_entities;

    // DEBUG
    TriggerAction this_frame_trigger_action = TriggerAction::None;

    void reset()
    {
        this_frame_trigger_action = TriggerAction::None;
    }
};

// DOOR

struct DoorInfo
{ 
    bool locked;
    bool open;
    bool was_open;
    CollisionShape open_shape;
    CollisionShape closed_shape;
    Mesh* open_mesh;
    Mesh* closed_mesh;
};

// STATUS INFECTOR

struct StatusChangeInfo
{
    Effect effect;
};

// TODO: interactable info size reduction: pack all boolean fields in a bitmask

struct InteractableInfo 
{
    TriggerMode     trigger_mode;
    InteractionType interaction_type;

    // Filters
    bool only_player;
    bool clickable;
    bool affects_actor;

    // State for ONLY_PLAYER
    bool player_is_inside   = false;
    bool player_was_inside  = false;
    bool player_has_entered = false;

    // State for anonymous interactables
    uint8_t overlap_count          = 0;
    uint8_t previous_overlap_count = 0;    

    // UI State
    bool hovered = false;

    // TODO: runtime/static collision shape: maybe make a separate struct that has them both an rewrite logic on that (cleaner)
    // BVH/Shape Data
    CollisionShape rest_shape;
    CollisionShape runtime_shape;
    // Index aabb_tree_idx = nullAABBTreeIndex;

    InteractableInfo() = default;

    InteractableInfo(
        TriggerMode     trigger_mode, 
        InteractionType interaction_type, 
        bool only_player,
        bool clickable,
        bool affects_actor,
        CollisionShape rest_shape, 
        CollisionShape runtime_shape) 
    : 
        trigger_mode(trigger_mode),
        interaction_type(interaction_type),
        only_player(only_player),
        clickable(clickable),
        affects_actor(affects_actor), 
        rest_shape(rest_shape),
        runtime_shape(runtime_shape)
    {}

    void reset() 
    { 
        player_has_entered = player_has_entered || player_is_inside;
        player_was_inside  = player_is_inside;
        player_is_inside   = false;
        
        previous_overlap_count = overlap_count;
        overlap_count          = 0;
        
        hovered = false; 
    }
};

constexpr auto sizeOfCollisionShape   = sizeof(CollisionShape);
constexpr auto sizeOfInteractableInfo = sizeof(InteractableInfo);

struct InteractionHistory 
{
    SmallSet<Entity, 16> current;
    SmallSet<Entity, 16> previous;
    SmallSet<Entity, 16> historical;

    void reset() 
    {
        historical.add(current);
        previous = current;
        current.clear();
    }

    void addOverlap(Entity entity)
    {
        ASSERT(!current.contains(entity), "For debug: in my logic should not enter entity twice in current interaction history");

        current.add(entity);
    }
};

// =======================================================
// END Interactable
// =======================================================

struct Type 
{ 
    EntityType type; 
};

struct WorldTransform 
{
    Real3 position;
    Real3 orientation;
};

template<typename T>
inline void doNothing(const T& t) {} 

// =======================================================
// ECS Registry
// =======================================================

#define COMPONENT_LIST                                        \
    X(RenderInfo,           128, doNothing)                   \
    X(Type,                  64, doNothing)                   \
    X(PhysicsBody,           64, doNothing)                   \
    X(Attachment,            32, doNothing)                   \
    X(PathFollower,          32, doNothing)                   \
    X(SimplePosition,        64, doNothing)                   \
    X(RandomDirectionMover,  32, doNothing)                   \
    X(ProjectileInfo,        32, doNothing)                   \
    X(EnemyInfo,             32, doNothing)                   \
    X(Status,                32, doNothing)                   \
    X(InteractableInfo,      32, doNothing)                   \
    X(DoorInfo,              32, doNothing)                   \
    X(AnimationState,        32, doNothing)                   \
    X(StatusChangeInfo,      32, doNothing)                   \
    X(LifeTime,              32, doNothing)                   \
    X(ParticleRendererInfo,  32, destroyParticleRendererInfo) \
    X(InteractionHistory,    32, doNothing)                   \
    X(HouseLayer,             8, doNothing)                   \
    X(House,                  2, doNothing)                   \
    X(PoolDestructionInfo,   16, doNothing)                   \

// TODO: mechanism to destroy when entity owner dies: owner predicate in component struct checked with type traits / concepts like resettable 
#define COMPONENT_POOL_LIST \
    X(Constraint, 16)       \

#define DEF_POOL_COMPONENT(type, size) ComponentPool<type, size> p_##type

#define DEF_MAP_COMPONENT(type, size) ComponentMap<type, size> m_##type

#define DEF_SET_COMPONENT(type)                                    \
    inline void set##type##Component(Entity e, const type& comp) { \
        m_##type.setComponent(e, comp);                            \
    }

#define DEF_HAS_COMPONENT(type)                  \
    inline bool has##type##Component(Entity e) { \
        return m_##type.hasComponent(e);         \
    }

#define DEF_GET_COMPONENT(type)                   \
    inline type& get##type##Component(Entity e) { \
        return m_##type.getComponent(e);          \
    }

#define DEF_ERASE_COMPONENT(type, destructor)      \
    inline void erase##type##Component(Entity e) { \
        if (m_##type.hasComponent(e))              \
            destructor(m_##type.getComponent(e));  \
        m_##type.eraseComponent(e);                \
    }

#define DEF_COMPONENT(type, size, destructor) \
    DEF_MAP_COMPONENT(type, size);            \
    DEF_SET_COMPONENT(type);                  \
    DEF_GET_COMPONENT(type);                  \
    DEF_HAS_COMPONENT(type);                  \
    DEF_ERASE_COMPONENT(type, destructor)     \

#define HAS_COMPONENT(e, type)      registry.has##type##Component(e)
#define GET_COMPONENT(e, type)      registry.get##type##Component(e)
#define SET_COMPONENT(e, type, val) registry.set##type##Component(e, val)
#define COMPONENT(e, type)          registry.m_##type[e]

#define MAP(type)        registry.m_##type
#define CONST_MAP(type)  std::as_const(MAP(type))

#define POOL(type)       registry.p_##type
#define CONST_POOL(type) std::as_const(POOL(type))

// Overloads
#define EXPAND(x) x
#define GET_MACRO(_1, _2, _3, NAME, ...) NAME

#define MAPS_1(a)           MAP(a)
#define MAPS_2(a, b)        MAP(a), MAP(b)
#define MAPS_3(a, b, c)     MAP(a), MAP(b), MAP(c)

#define MAPS(...) EXPAND(GET_MACRO(__VA_ARGS__, MAPS_3, MAPS_2, MAPS_1)(__VA_ARGS__))

#define HAS_ASSERT(e, type) ASSERT(HAS_COMPONENT(e, type), #e " must have " #type " component")

#define DELETE_COMPONENT(e, type) erase##type##Component(e)

template <std::size_t N>
struct EntityCreator 
{
    bool  used[N]       = { false };
    Entity last_checked = 0;

    Entity create() 
    {
        for (Entity i = 0; i < N; ++i) 
        {
            Entity idx = (last_checked + i) % N;
            if (!used[idx]) 
            {
                used[idx]    = true;
                last_checked = (idx + 1) % N;
                return idx;
            }
        }

        ASSERT(false, "FreeList out of space");
        return -1;
    }

    void remove(Entity e) 
    {
        ASSERT(e >= 0 && e < static_cast<SparseIndex>(N), "Invalid FreeList index");
        ASSERT(used[e], "Double free detected in FreeList");

        used[e] = false;
    }
};

#define INITIAL_SHOOTING_SPEED 12.0f
#define INITIAL_THROWING_SPEED  7.0f

#define MIN_EJECT_SPEED 6.0f
#define MAX_EJECT_SPEED 15.0f

#define MIN_SHOOTING_DISTANCE  1.0f
#define MAX_SHOOTING_DISTANCE  4.0f
#define GRAVITY_ACCELERATION  10.0f

#define WIDTH  800
#define HEIGHT 600

enum class ComponentPoolID : uint8_t
{
    #define X(type, size) type,
        COMPONENT_POOL_LIST
    #undef X 
};

enum class ComponentMapID : uint8_t
{
    #define X(type, size, destructor) type,
        COMPONENT_LIST
    #undef X
};

struct Registry 
{
    #define X(type, size, destructor) DEF_COMPONENT(type, size, destructor);
        COMPONENT_LIST
    #undef X

    #define X(type, size) DEF_POOL_COMPONENT(type, size);
        COMPONENT_POOL_LIST
    #undef X

#ifdef SPARSE_MAP_IMPLEMENTATION

    EntityCreator<MAX_NUMBER_ENTITIES> entity_creator;

    Entity createEntity()
    {
        return entity_creator.create();
    }

    void eraseEntity(Entity e) 
    {
        #define X(type, size, destructor) DELETE_COMPONENT(e, type);
            COMPONENT_LIST
        #undef X

        entity_creator.remove(e);
    }

#else

    Entity nextId = 0;
    Entity createEntity() 
    { 
        return nextId++; 
    }
    
    void eraseEntity(Entity e) 
    {
        #define X(type, size) DELETE_COMPONENT(e, type)
            COMPONENT_LIST
        #undef X
    }

#endif

    std::vector<StaticPhysicsBody> static_world;
    AABBTree        static_world_bvh;
    DynamicAABBTree dynamic_world_bvh;

    void loadLevel(std::string file_path, const std::vector<std::string>& house_paths);
    void loadHouse(const std::string& file_path, std::vector<AABB>& bvh_boxes, std::vector<StaticAABBTreeNodeData>& bvh_data);

    std::vector<Entity> entities_to_remove;

    inline void setEntityToRemove(Entity e) 
    { 
        ASSERT(!isEntityMarkedForRemoval(e), "Must not set entity to remove twice");
        entities_to_remove.push_back(e); 
    }

    inline bool isEntityMarkedForRemoval(Entity e) const 
    {
        return std::find(entities_to_remove.begin(),
                        entities_to_remove.end(),
                        e) != entities_to_remove.end();
    }

    inline void removeDeathEntities() 
    {
        for (Entity e : entities_to_remove)
            eraseEntity(e);
        entities_to_remove.clear();
    }

    inline void flushPools()
    {
        #define X(type, size) p_##type.flushRemovals();
            COMPONENT_POOL_LIST
        #undef X
    }

    CameraInfo    camera;
    CrosshairInfo crosshair;
    TrajectoryInfo trajectory;

    Entity player_entity;

    ProjectileType equipped_projectile = ProjectileType::Arrow;
    Real shooting_speed                = INITIAL_SHOOTING_SPEED;
    Real throwing_speed                = INITIAL_THROWING_SPEED;
    bool valid_eject_target            = false;

    MeshManager   mesh_manager;
    InputManager  input_manager;
    ShaderManager shader_manager;

    Mesh circle_mesh;
    Mesh crosshair_line_mesh;

    SnowRenderer          snow_renderer;
    FogRenderer           fog_renderer;
    TrajectoryRenderer    trajectory_renderer;
    ShootingRangeRenderer shooting_range_renderer;
    EffectSphereRenderer  effect_sphere_renderer;
    DebugDepthRenderer    debug_depth_renderer;
    DigitalGridRenderer   digital_grid_renderer;

    SceneBuffer scene_buffer;

    std::vector<float> alternative_crosshair_line_vertices; 

    Ray pointer_ray;
    bool valid_mouse_pointer;

    std::vector<Entity> transparent_entities;
    std::set<Entity>    pointed_entities;

    inline bool isTransparent(const Entity& entity) const 
    {
        return std::find(
                    transparent_entities.begin(),
                    transparent_entities.end(),
                    entity) != transparent_entities.end();
    }

    inline bool isPointed(const Entity& entity) const 
    {
        return pointed_entities.find(entity) != pointed_entities.end();
    }

    inline bool isPlayer(const Entity& e) const 
    {
        return e == player_entity;
    }

    Registry(GLFWwindow* window) : 
        circle_mesh(createUnitCircleMesh(32)),
        crosshair_line_mesh(createSegmentedDynamicLineMesh(CROSSHAIR_LINE_SEGMENTS)), 
        scene_buffer(WIDTH, HEIGHT)
    {
        dynamic_world_bvh = createDynamicAABBTree();
        transparent_entities.reserve(16);

        input_manager.window = window;

        pointer_ray         = {Real3(0.0f), Real3(0.0f)};
        valid_mouse_pointer = false;
    }

private:

    std::optional<WorldTransform> internalGetTransform(Entity entity) 
    {
        if (m_ProjectileInfo.hasComponent(entity))
            return WorldTransform{ m_ProjectileInfo[entity].position, m_ProjectileInfo[entity].orientation };
        if (m_PhysicsBody.hasComponent(entity))
            return WorldTransform{ m_PhysicsBody[entity].position, m_PhysicsBody[entity].orientation };
        if (m_SimplePosition.hasComponent(entity))
            return WorldTransform{ m_SimplePosition[entity].position, m_SimplePosition[entity].orientation };

        return std::nullopt;
    }

public:

    WorldTransform getWorldTransform(Entity entity, std::source_location loc = std::source_location::current()) 
    {
        auto transform = internalGetTransform(entity);

        if (!transform.has_value())
        {
            reportMissingComponentError(entity, "WorldTransform", loc);
            return {};
        }

        return transform.value();
    }

    Real3 getWorldPosition(Entity entity, std::source_location loc = std::source_location::current()) 
    {
        return getWorldTransform(entity, loc).position;
    }

    Real3 getWorldOrientation(Entity entity, std::source_location loc = std::source_location::current()) 
    {
        return getWorldTransform(entity, loc).orientation;
    }

    void setComponentToRemove(ComponentPoolID pool_id, PoolIndex idx)
    {
        switch(pool_id)
        {
            #define X(type, size) case ComponentPoolID::type: p_##type.setComponentToRemove(idx); return;
                COMPONENT_POOL_LIST
            #undef X 

            default: ASSERT(false, "impossible component pool id");
        }
    }

};

// ASSEMBLEs

// void updateCollisionShape(const CollisionShape& rest_shape, CollisionShape& runtime_shape, const Real3& pos, const Real3& prev_pos);
// void updateCollisionShape(const CollisionShape& rest_shape, CollisionShape& runtime_shape, const Real3& pos);

Entity assemblePlayer(Registry& registry, Mesh* mesh);
Entity assembleRandomDynamic(Registry& registry, Mesh* mesh, Real3 position);
Entity assembleStaticNPC(Registry& registry, Mesh* mesh, Real3 position);
Entity assembleStatusChangeArea(Registry& registry, Real3 position, Real radius, Real life_time);
Entity assembleDoor(
    Registry& registry, 
    Real3 position,
    Mesh* open_mesh, Mesh* closed_mesh, Real3 color,
    AABB open_coll_aabb, AABB closed_coll_abb,
    bool locked);
Entity assembleHouseLayerToggle(Registry& registry, Real3 position);

Entity assembleStaticAACapsuleObject(Registry& registry, Mesh* mesh, Real3 position, Real radius, Real height, Axis axis, Real3 color = Real3(1.0f));
Entity assembleStaticCapsuleObject(Registry& registry, Mesh* mesh, Real3 start, Real3 end, Real radius, Real3 color = Real3(1.0f));
Entity assembleStaticCylinderObject(Registry& registry, Mesh* mesh, Real3 position, Real radius, Real height, Axis axis, Real3 color);
Entity assembleStaticSphereObject(Registry& registry, Mesh* mesh, Real3 position, Real radius, Real3 color = Real3(1.0f));
Entity assembleStaticAARectangleObject(Registry& registry, Mesh* mesh, Real3 position, Real3 extent, Axis axis, Real3 color = Real3(1.0f));
Entity assembleStaticAxialOBBObject(Registry& registry, Real3 position, Real3 extent, Real angle, Axis axis);
Entity assembleStaticAA4SidedPyramidObject(Registry& registry, Real3 position, Real2 half_extents, Real height, Axis axis, CollisionObjectType obj_type);
Entity assembleStaticAAEllipsoidObject(Registry& registry, Real3 position, Real3 half_extents);
Entity assembleStaticAAConeObject(Registry& registry, Real3 position, Real radius, Real height, Axis axis);
Entity assembleStaticAAHollowCylinderObject(Registry& registry, Real3 position, Real inner_radius, Real outer_radius, Real height, Axis axis);
Entity assembleStaticAAHemisphereObject(Registry& registry, Real3 position, Real radius, AxisDir axis);
Entity assembleStaticAATrapezoidObject(Registry& registry, Real3 position, Real2 half_extents, Real2 top_half_extents, Real height, Axis axis);
Entity assembleStaticAATetrahedronObject(Registry& registry, Real3 position, Real3 extents);
Entity assembleStaticAARampObject(Registry& registry, Real3 position, Real3 pos_extents, AARampDirection dir);


// Entity assemblePathObject(
//         Registry& registry, 
//         Mesh* mesh, 
//         glm::vec3& color, 
//         glm::vec3& position, 
//         AABB& aabb, 
//         std::vector<glm::vec3> pathPoints, 
//         float speed);

Entity assembleSceneObject(Registry& registry, Mesh* mesh, glm::vec3 color);

void initCamera(Registry& registry, Entity target);
void initCrosshair(Registry& registry, Mesh* mesh);

// Entity assembleProjectile(Registry& registry, Real3 position, Real3 direction, float speed, Entity shooter);
// Entity assemblePotionProjectile(Registry& registry, Real3 position, Real3 direction, float speed, Entity shooter);

Entity assembleProjectile(Registry& registry, ProjectileType proj_type, Real3 position, Real3 direction, float speed, Entity shooter);
Entity assembleStaticIdleProjectile(Registry& registry, Mesh* mesh, Real3 position, Real3 orientation);
Entity assembleAttachedIdleProjectile(
    Registry& registry, 
    Mesh* mesh, 
    Real3  position_attach, 
    Real3  starting_projectile_orientation,
    Entity target);

// SYSTEMs

void assertSystem(Registry &registry);

void resetSystem(Registry &registry);

void updateSystem(Registry &registry, Real delta_time);

void playerInputSystem(Registry& registry, InputManager& input_manager);
void shootingSystem(Registry& registry, InputManager& input_manager, Entity shooter);

void movementSystem(Registry& registry, Real delta_time);

void effectSystem(Registry& registry, Real dt);

void physicsSystem(Registry& registry, Real delta_time);

void solveConstraintsSystem(Registry& registry, Real dt);

void projectileOrientationSystem(Registry& registry);

void pathFollowingSystem(Registry& registry);

void attachmentSystem(Registry& registry);

void transparentOccludingSystem(Registry& registry);

void collisionShapeUpdateSystem(Registry& registry);
void collisionSystem(Registry& registry, InputManager& input_manager);

void interactionSystem(Registry& registry);

void updateAnimationStateSystem(Registry& registry);

void houseHidingSystem(Registry& registry);
void cameraFollowSystem(Registry& registry, float delta_time);
void cameraSetupSystem(Registry& registry);
void renderSystem(Registry& registry);
void renderCrosshairSystem(Registry& registry);
void renderCollisionShapesSystem(Registry& registry);

void lifeTimeSystem(Registry& registry, float delta_time);
void deathSystem(Registry& registry);
void clearSystem(Registry& registry);

void aimSystem(Registry& registry, InputManager& input_manager);
