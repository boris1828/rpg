#pragma once

#include <stdint.h>
#include <unordered_map>
#include <iostream>
#include <algorithm>
#include <set>
#include <vector>
#include <filesystem>
#include <map>
#include <array>
#include <random>
#include <cstdint>
#include <chrono>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Macro.h"
#include "Types.h"


// COMPONENTS

using Entity = uint16_t;


struct Position
{
    Real x, y, z;
};

struct Velocity
{
    Real x, y, z;
};

struct Transform
{
    Real3 position;
    Real3 rotation;
    Real3 scale;
};

struct RigidBody
{
    Real mass;
    Real invMass;
    Real3 force;
    bool isStatic;
};

struct Collider
{
    Real3 halfExtents;
    uint32_t layer;
    bool isTrigger;
};

struct RenderMesh
{
    uint32_t meshId;
    uint32_t materialId;
    bool visible;
};

struct AIState
{
    uint32_t state;
    Real timer;
    Entity target;
};

struct Animation
{
    uint32_t currentClip;
    Real time;
    Real speed;
    bool looping;
};

struct NetworkSync
{
    uint32_t netId;
    Real lastUpdateTime;
    bool dirty;
};

struct Inventory
{
    Entity items[16];
    uint32_t count;
};

struct Health
{
    Real current;
    Real max;
    Real regenRate;
};

struct StatusEffect
{
    bool active;
    Real damagePerSecond;
};

struct LifeTime
{
    Real time;   // seconds remaining
};

#define COMPONENT_LIST      \
    X(Position,     MAX_NUMBER_ENTITIES);   \
    X(Velocity,     MAX_NUMBER_ENTITIES);   \
    X(Transform,    MAX_NUMBER_ENTITIES);   \
    X(RigidBody,    MAX_NUMBER_ENTITIES);   \
    X(Collider,     MAX_NUMBER_ENTITIES);   \
    X(RenderMesh,   MAX_NUMBER_ENTITIES);   \
    X(AIState,      MAX_NUMBER_ENTITIES);   \
    X(Animation,    MAX_NUMBER_ENTITIES);   \
    X(NetworkSync,  MAX_NUMBER_ENTITIES);   \
    X(Inventory,    MAX_NUMBER_ENTITIES);   \
    X(Health,       MAX_NUMBER_ENTITIES);   \
    X(StatusEffect, MAX_NUMBER_ENTITIES);   \
    X(LifeTime,     MAX_NUMBER_ENTITIES);

// SPARSE SET ECS

// #define SPARSE_ECS

#ifdef SPARSE_ECS

#define MAX_NUMBER_ENTITIES 256

using SparseIndex = uint16_t;

constexpr SparseIndex NULL_INDEX = UINT16_MAX;

template<typename T, std::size_t N>
struct ComponentMap
{
    T      dense_components[N];
    Entity dense_entities[N];

    std::array<SparseIndex, MAX_NUMBER_ENTITIES> sparse_map;
    SparseIndex size = 0;

    ComponentMap()
    {
        sparse_map.fill(NULL_INDEX);
    }

    bool hasComponent(Entity e) const
    {
        ASSERT(e < MAX_NUMBER_ENTITIES, "Entity must be in valid range");
        return sparse_map[e] != NULL_INDEX;
    }

    T& getComponent(Entity e)
    {
        ASSERT(e < MAX_NUMBER_ENTITIES, "Entity must be in valid range");
        ASSERT(hasComponent(e), "Entity does not have this component");

        return dense_components[sparse_map[e]];
    }

    void setComponent(Entity entity, const T& component)
    {
        ASSERT(entity < MAX_NUMBER_ENTITIES, "Entity must be in valid range");

        SparseIndex idx = sparse_map[entity];

        if (idx == NULL_INDEX)
        {
            ASSERT(size < N, "Component pool overflow");

            idx = size++;
            sparse_map[entity]   = idx;
            dense_entities[idx]  = entity;
            dense_components[idx] = component;
        }
        else
        {
            dense_components[idx] = component;
        }
    }

    SparseIndex createComponent(Entity entity)
    {
        ASSERT(entity < MAX_NUMBER_ENTITIES, "Entity must be in valid range");
        ASSERT(!hasComponent(entity), "Entity already has this component");
        ASSERT(size < N, "Component pool overflow");

        SparseIndex idx = size++;

        sparse_map[entity]   = idx;
        dense_entities[idx]  = entity;

        return idx;
    }

    void eraseComponent(Entity entity)
    {
        ASSERT(entity < MAX_NUMBER_ENTITIES, "Entity must be in valid range");

        SparseIndex idx = sparse_map[entity];
        if (idx == NULL_INDEX)
            return;

        SparseIndex last = --size;

        Entity last_entity = dense_entities[last];

        // Move last element into the removed slot
        dense_components[idx] = dense_components[last];
        dense_entities[idx]   = last_entity;

        sparse_map[last_entity] = idx;
        sparse_map[entity]      = NULL_INDEX;
    }

    T& operator[](Entity entity)
    {
        ASSERT(entity < MAX_NUMBER_ENTITIES, "Entity must be in valid range");

        SparseIndex idx = sparse_map[entity];
        if (idx == NULL_INDEX)
            idx = createComponent(entity);

        return dense_components[idx];
    }

    struct iterator
    {
        Entity* e;
        T*      c;

        iterator(Entity* e, T* c) : e(e), c(c) {}

        iterator& operator++() { ++e; ++c; return *this; }
        bool operator!=(const iterator& other) const { return e != other.e; }

        auto operator*() const
        {
            return std::pair<Entity&, T&>(*e, *c);
        }
    };

    iterator begin() { return { dense_entities, dense_components }; }
    iterator end()   { return { dense_entities + size, dense_components + size }; }
};


#define DEF_MAP_COMPONENT(type, size) ComponentMap<type, size> m_##type

#define DEF_SET_COMPONENT(type)                                    \
    inline void set##type##Component(Entity e, const type& comp) { \
        m_##type.setComponent(e, comp);                            \
    }

#define DEF_HAS_COMPONENT(type)                    \
    inline bool has##type##Component(Entity e) {   \
        return m_##type.hasComponent(e);           \
    }

#define DEF_GET_COMPONENT(type)                    \
    inline type& get##type##Component(Entity e) {  \
        return m_##type.getComponent(e);           \
    }

#define DEF_COMPONENT(type, size)  \
    DEF_MAP_COMPONENT(type, size); \
    DEF_SET_COMPONENT(type);       \
    DEF_GET_COMPONENT(type);       \
    DEF_HAS_COMPONENT(type);       \

#define HAS_COMPONENT(e, type)      registry.has##type##Component(e)
#define GET_COMPONENT(e, type)      registry.get##type##Component(e)
#define SET_COMPONENT(e, type, val) registry.set##type##Component(e, val)
#define COMPONENT(e, type)          registry.m_##type[e]

#define HAS_ASSERT(e, type) ASSERT(HAS_COMPONENT(e, type), #e " must have " #type " component")

#define DELETE_COMPONENT(e, type) m_##type.eraseComponent(e)

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

struct TestRegistry 
{
    #define X(type, size) DEF_COMPONENT(type, size)
        COMPONENT_LIST
    #undef X

    EntityCreator<MAX_NUMBER_ENTITIES> entity_creator;

    std::set<Entity> valid_entities;

    Entity createEntity()
    {
        Entity e = entity_creator.create();
        valid_entities.insert(e);
        return e;
    }
    
    void eraseEntity(Entity e) 
    {
        #define X(type, size) DELETE_COMPONENT(e, type)
            COMPONENT_LIST
        #undef X

        entity_creator.remove(e);
        valid_entities.erase(e);
    }
};

#else 

#define DEF_MAP_COMPONENT(type) std::unordered_map<Entity, type> m_##type

#define DEF_SET_COMPONENT(type)                                    \
    inline void set##type##Component(Entity e, const type& comp) { \
        m_##type[e] = comp;                                        \
    }

#define DEF_HAS_COMPONENT(type)                    \
    inline bool has##type##Component(Entity e) {   \
        return m_##type.find(e) != m_##type.end(); \
    }

#define DEF_GET_COMPONENT(type)                                             \
    inline  type& get##type##Component(Entity e) {                          \
        ASSERT(                                                             \
            has##type##Component(e),                                        \
            "Entity must have " #type " component before it is retrieved"); \
        return m_##type[e];                                                 \
    }

#define DEF_COMPONENT(type)   \
    DEF_MAP_COMPONENT(type);  \
    DEF_SET_COMPONENT(type);  \
    DEF_GET_COMPONENT(type);  \
    DEF_HAS_COMPONENT(type);  \

#define HAS_COMPONENT(e, type)      registry.has##type##Component(e)
#define GET_COMPONENT(e, type)      registry.get##type##Component(e)
#define SET_COMPONENT(e, type, val) registry.set##type##Component(e, val)
#define COMPONENT(e, type)          registry.m_##type[e]

#define HAS_ASSERT(e, type) ASSERT(HAS_COMPONENT(e, type), #e " must have " #type " component")

#define DELETE_COMPONENT(e, type) m_##type.erase(e)

struct TestRegistry 
{
    #define X(type, size) DEF_COMPONENT(type)
        COMPONENT_LIST
    #undef X

    std::set<Entity> valid_entities;

    Entity nextId = 0;
    Entity createEntity() 
    { 
        valid_entities.insert(nextId);
        return nextId++; 
    }
    
    void eraseEntity(Entity e) {
        #define X(type, size) DELETE_COMPONENT(e, type)
            COMPONENT_LIST
        #undef X
        valid_entities.erase(e);
    }
};

#endif

void movementSystem(TestRegistry& registry, Real dt)
{
    for (auto& [e, vel] : registry.m_Velocity)
    {
        if (!HAS_COMPONENT(e, Position)) continue;

        Position& pos = GET_COMPONENT(e, Position);
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;
        pos.z += vel.z * dt;
    }
}

void gravitySystem(TestRegistry& registry, Real dt)
{
    const Real g = -9.81f;

    for (auto& [e, rb] : registry.m_RigidBody)
    {
        if (rb.isStatic) continue;

        rb.force.z += g * rb.mass;
    }
}

void physicsIntegrationSystem(TestRegistry& registry, Real dt)
{
    for (auto& [e, rb] : registry.m_RigidBody)
    {
        if (!HAS_COMPONENT(e, Velocity)) continue;

        Velocity& vel = GET_COMPONENT(e, Velocity);

        Real3 acc = rb.force * rb.invMass;
        vel.x += acc.x * dt;
        vel.y += acc.y * dt;
        vel.z += acc.z * dt;

        rb.force = Real3(0,0,0);
    }
}

void transformSyncSystem(TestRegistry& registry)
{
    for (auto& [e, tr] : registry.m_Transform)
    {
        if (!HAS_COMPONENT(e, Position)) continue;

        Position& pos = GET_COMPONENT(e, Position);
        tr.position = Real3(pos.x, pos.y, pos.z);
    }
}

void aiStateSystem(TestRegistry& registry, Real dt)
{
    for (auto& [e, ai] : registry.m_AIState)
    {
        ai.timer += dt;

        if (ai.timer > 2.0f)
        {
            ai.state = (ai.state + 1) % 3;
            ai.timer = 0.0f;
        }
    }
}

void steeringSystem(TestRegistry& registry)
{
    for (auto& [e, ai] : registry.m_AIState)
    {
        if (!HAS_COMPONENT(e, Velocity)) continue;

        Velocity& vel = GET_COMPONENT(e, Velocity);

        switch (ai.state)
        {
            case 0: vel.x =  1.0f; vel.y =  0.0f; break;
            case 1: vel.x = -1.0f; vel.y =  0.0f; break;
            case 2: vel.x =  0.0f; vel.y =  1.0f; break;
        }
    }
}

void healthRegenSystem(TestRegistry& registry, Real dt)
{
    for (auto& [e, health] : registry.m_Health)
    {
        if (health.current <= 0.0f) continue;

        health.current += health.regenRate * dt;
        health.current = glm::min(health.current, health.max);
    }
}

void damageOverTimeSystem(TestRegistry& registry, Real dt)
{
    for (auto& [e, status] : registry.m_StatusEffect)
    {
        if (!HAS_COMPONENT(e, Health)) continue;

        Health& hp = GET_COMPONENT(e, Health);

        if (status.active)
        {
            hp.current -= status.damagePerSecond * dt;

            if (hp.current <= 0.0f)
                status.active = false;
        }
    }
}

void lifetimeSystem(TestRegistry& registry, Real dt)
{
    for (auto& [e, life] : registry.m_LifeTime)
    {
        life.time -= dt;

        if (life.time <= 0.0f)
        {
            registry.eraseEntity(e);
        }
    }
}

void visibilitySystem(TestRegistry& registry, const Real3& cameraPos)
{
    for (auto& [e, mesh] : registry.m_RenderMesh)
    {
        if (!HAS_COMPONENT(e, Position)) continue;

        Position& pos = GET_COMPONENT(e, Position);

        Real dx = pos.x - cameraPos.x;
        Real dy = pos.y - cameraPos.y;
        Real dz = pos.z - cameraPos.z;

        Real dist2 = dx*dx + dy*dy + dz*dz;
        mesh.visible = (dist2 < 1000.0f);
    }
}

static std::mt19937 rng{ 12345 }; 

static uint32_t randU32(uint32_t min, uint32_t max)
{
    std::uniform_int_distribution<uint32_t> dist(min, max);
    return dist(rng);
}

static Real randReal(Real min, Real max)
{
    std::uniform_real_distribution<Real> dist(min, max);
    return dist(rng);
}

static uint32_t decideSpawnCount()
{
    uint32_t r = randU32(0, 99);

    if (r < 70) return 0;  // 70%
    if (r < 90) return 1;  // 20%
    return 2;              // 10%
}

uint32_t randomSpawnSystem(TestRegistry& registry)
{
    uint32_t spawnCount = decideSpawnCount();
    if (spawnCount == 0) return 0;

    for (uint32_t i = 0; i < spawnCount; ++i)
    {
        Entity e = registry.createEntity();

        uint32_t archetype = randU32(0, 3);

        Transform transform{};
        transform.position = { randReal(-50, 50), randReal(-50, 50), randReal(-5, 5) };
        transform.rotation = { 0, 0, 0 };
        transform.scale    = { 1, 1, 1 };

        SET_COMPONENT(e, Transform, transform);

        switch (archetype)
        {
            // =====================================================
            // 1) Dynamic Actor
            // =====================================================
            case 0:
            {
                Velocity vel{ randReal(-1, 1), randReal(-1, 1), 0 };
                SET_COMPONENT(e, Velocity, vel);

                RigidBody rb{};
                rb.mass     = 10.0f;
                rb.invMass  = 0.1f;
                rb.force    = { 0, 0, 0 };
                rb.isStatic = false;
                SET_COMPONENT(e, RigidBody, rb);

                Collider col{};
                col.halfExtents = { 0.5f, 0.5f, 1.0f };
                col.layer       = 1;
                col.isTrigger   = false;
                SET_COMPONENT(e, Collider, col);

                RenderMesh mesh{};
                mesh.meshId     = 1;
                mesh.materialId = 1;
                mesh.visible    = true;
                SET_COMPONENT(e, RenderMesh, mesh);

                AIState ai{};
                ai.state  = randU32(0, 3);
                ai.timer  = randReal(0, 5);
                ai.target = UINT32_MAX;
                SET_COMPONENT(e, AIState, ai);

                Animation anim{};
                anim.currentClip = 0;
                anim.time        = 0;
                anim.speed       = 1.0f;
                anim.looping     = true;
                SET_COMPONENT(e, Animation, anim);
                break;
            }

            // =====================================================
            // 2) Static World Object
            // =====================================================
            case 1:
            {
                RigidBody rb{};
                rb.mass     = 0.0f;
                rb.invMass  = 0.0f;
                rb.force    = { 0, 0, 0 };
                rb.isStatic = true;
                SET_COMPONENT(e, RigidBody, rb);

                Collider col{};
                col.halfExtents = { 1.0f, 1.0f, 1.0f };
                col.layer       = 2;
                col.isTrigger   = false;
                SET_COMPONENT(e, Collider, col);

                RenderMesh mesh{};
                mesh.meshId     = 2;
                mesh.materialId = 2;
                mesh.visible    = true;
                SET_COMPONENT(e, RenderMesh, mesh);
                break;
            }

            // =====================================================
            // 3) Networked Moving Entity
            // =====================================================
            case 2:
            {
                Velocity vel{ randReal(-2, 2), randReal(-2, 2), 0 };
                SET_COMPONENT(e, Velocity, vel);

                NetworkSync net{};
                net.netId          = randU32(1000, 10000);
                net.lastUpdateTime = 0;
                net.dirty          = true;
                SET_COMPONENT(e, NetworkSync, net);

                RenderMesh mesh{};
                mesh.meshId     = 3;
                mesh.materialId = 1;
                mesh.visible    = true;
                SET_COMPONENT(e, RenderMesh, mesh);
                break;
            }

            // =====================================================
            // 4) AI Agent with Inventory
            // =====================================================
            case 3:
            {
                Velocity vel{ randReal(-0.5f, 0.5f), randReal(-0.5f, 0.5f), 0 };
                SET_COMPONENT(e, Velocity, vel);

                AIState ai{};
                ai.state  = randU32(0, 2);
                ai.timer  = randReal(1, 10);
                ai.target = UINT32_MAX;
                SET_COMPONENT(e, AIState, ai);

                Inventory inv{};
                inv.count = randU32(0, 4);
                for (uint32_t j = 0; j < inv.count; ++j)
                    inv.items[j] = UINT32_MAX;
                SET_COMPONENT(e, Inventory, inv);

                RenderMesh mesh{};
                mesh.meshId     = 4;
                mesh.materialId = 3;
                mesh.visible    = true;
                SET_COMPONENT(e, RenderMesh, mesh);
                break;
            }
        }
    }

    return spawnCount;
}

uint32_t randomDespawnSystem(TestRegistry& registry)
{
    uint32_t toRemove = decideSpawnCount();

    if (toRemove == 0 || registry.valid_entities.empty())
        return 0;

    std::vector<Entity> entities(registry.valid_entities.begin(), registry.valid_entities.end());

    uint32_t removed = 0;
    for (uint32_t i = 0; i < toRemove && !entities.empty(); ++i)
    {
        uint32_t idx = randU32(0, static_cast<uint32_t>(entities.size() - 1));
        Entity e = entities[idx];

        registry.eraseEntity(e);

        entities.erase(entities.begin() + idx);
        removed++;
    }

    return removed;
}


#define NUM_ITERATIONS 1000000
#define NUM_INIT_SPAWN 100

void loop()
{
    TestRegistry registry;
    std::cout << "Size of TestRegistry: " << sizeof(TestRegistry) << " bytes\n";

    Real dt          = 1.0/60.0;
    Real3 camera_pos = Real3(1.0, 2.0, 3.0);

    for (int i=0; i<NUM_INIT_SPAWN; i++)
    {
        randomSpawnSystem(registry);
    }

    std::cout << "Number of valid entites at the beginning " << registry.valid_entities.size() << "\n";

    auto start = std::chrono::high_resolution_clock::now();

    uint32_t added = 0;
    uint32_t removed = 0;

    for (int i=0; i<NUM_ITERATIONS; i++)
    {
        added += randomSpawnSystem(registry);

        movementSystem(registry, dt);
        gravitySystem(registry, dt);
        physicsIntegrationSystem(registry, dt);
        aiStateSystem(registry, dt);
        healthRegenSystem(registry, dt);
        damageOverTimeSystem(registry, dt);
        lifetimeSystem(registry, dt);

        transformSyncSystem(registry);
        steeringSystem(registry);

        visibilitySystem(registry, camera_pos);

        removed += randomDespawnSystem(registry);

        // if (i%100==0) std::cout << "valid entites at" << i << ": " << registry.valid_entities.size() << "\n";
    }

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed = end - start;

    std::cout << "Number of valid entites at the end " << registry.valid_entities.size() << "\n";
    std::cout << "Loop of " << NUM_ITERATIONS << " iterations took " << elapsed.count() << " ms.\n";

    std::cout << "Added: " << added << "\n" "Removed: " << removed << " \n";

    std::cout << "Press Enter to continue...";
    std::cin.get();
}