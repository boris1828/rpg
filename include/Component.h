#pragma once

#include <stdint.h>
#include <source_location>
#include <algorithm>

#include "Macro.h"
#include "Types.h"
#include "Utility.h"

static void reportMissingComponentError(Entity entity, const char* componentName, const std::source_location& loc) 
{
    static char msgBuffer[512]; 
    snprintf(msgBuffer, sizeof(msgBuffer), 
        "\n[ECS Error] File: %s (%u:%u)\n"
        "Function: %s\n"
        "Entity %u is missing a %s component!", 
        loc.file_name(), loc.line(), loc.column(),
        loc.function_name(), (uint32_t)entity, componentName);

    ASSERT(false, msgBuffer);
}

// =======================================================
// Component Map
// =======================================================

#define SPARSE_MAP_IMPLEMENTATION 

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

    constexpr static std::size_t maxNumbersComponentsToRemove = 16;
    SmallSet<Entity, maxNumbersComponentsToRemove> components_to_remove;

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
            ASSERT(size < N, (std::string("Component pool overflow in ") + typeid(T).name()).c_str());

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

    void setComponentToRemove(Entity entity)
    {
        ASSERT(!components_to_remove.contains(entity), "Can not set components to remove twice");

        components_to_remove.add(entity);
    }

    void flushRemovals()
    {
        for (auto entity: components_to_remove)
            eraseComponent(entity);

        components_to_remove.clear();
    }


    SparseIndex createComponent(Entity entity)
    {
        ASSERT(entity < MAX_NUMBER_ENTITIES, "Entity must be in valid range");
        ASSERT(!hasComponent(entity), "Entity already has this component");
        ASSERT(size < N, (std::string("Component pool overflow in ") + typeid(T).name()).c_str());

        SparseIndex idx = size;
        
        size++;

        sparse_map[entity]  = idx;
        dense_entities[idx] = entity;

        return idx;
    }

    void eraseComponent(Entity entity)
    {
        ASSERT(entity < MAX_NUMBER_ENTITIES, "Entity must be in valid range");

        SparseIndex idx = sparse_map[entity];
        if (idx == NULL_INDEX) return;

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

    struct const_iterator
    {
        const Entity* e;
        const T*      c;

        const_iterator(const Entity* e, const T* c) : e(e), c(c) {}

        const_iterator& operator++()
        {
            ++e;
            ++c;
            return *this;
        }

        bool operator!=(const const_iterator& other) const
        {
            return e != other.e;
        }

        auto operator*() const
        {
            return std::pair<const Entity&, const T&>(*e, *c);
        }
    };

    iterator begin() { return { dense_entities, dense_components }; }
    iterator end()   { return { dense_entities + size, dense_components + size }; }

    const_iterator begin() const { return { dense_entities, dense_components }; }
    const_iterator end()   const { return { dense_entities + size, dense_components + size }; }

    template<typename Func>
    void forEach(Func&& f) 
    {
        for (SparseIndex i = 0; i < size; ++i) 
        {
            f(dense_entities[i], dense_components[i]);
        }
    }

    template<typename Func>
    void forEach(Func&& f) const 
    {
        for (SparseIndex i = 0; i < size; ++i) 
        {
            f(dense_entities[i], static_cast<const T&>(dense_components[i]));
        }
    }
};

// =======================================================
// END Component Map
// =======================================================



// =======================================================
// View
// =======================================================

template<typename... Maps>
struct View 
{
    std::tuple<Maps*...> maps;

    View(Maps&... args) : maps(&args...) {}

    template<typename Func>
    void forEach(Func&& f) 
    {    
        auto leader = std::get<0>(maps);
        
        for (size_t i = 0; i < leader->size; ++i) 
        {
            Entity ent = leader->dense_entities[i];

            if (has_all<Maps...>(ent)) 
            {
                f(ent, (std::get<Maps*>(maps)->getComponent(ent))...);
            }
        }
    }

    template<typename Func>
    void forEachStrict(Func&& f, std::source_location loc = std::source_location::current()) 
    {    
        auto leader = std::get<0>(maps);
        for (size_t i = 0; i < leader->size; ++i) 
        {
            Entity ent = leader->dense_entities[i];

            const char* missingName = find_missing(ent);
            if (missingName != nullptr) 
            {
                reportMissingComponentError(ent, missingName, loc);
            }

            f(ent, (std::get<Maps*>(maps)->getComponent(ent))...);
        }
    }

private:

    template<typename... M>
    bool has_all(Entity ent) 
    {
        return (std::get<M*>(maps)->hasComponent(ent) && ...);
    }

    template<typename T>
    const char* get_comp_name() 
    {
        return typeid(T).name();
    }

    const char* find_missing(Entity ent) 
    {
        const char* name = nullptr;
        
        ((std::get<Maps*>(maps)->hasComponent(ent) ? true : (name = get_comp_name<Maps>(), false)) && ...);
        
        return name;
    }
};

// =======================================================
// END View
// =======================================================



// =======================================================
// Component Pool
// =======================================================

/*
    MaxNumberComponents = N

    T dense_components[N];
    SparseIndex pool_index_to_component_map[N];  // PoolIndex -> DenseComponentIndex
*/

using PoolIndex = uint16_t;

template<typename T, std::size_t N>
struct ComponentPool
{
    T dense_components[N];
    PoolIndex   dense_to_stable[N]; 
    SparseIndex stable_to_dense[N]; 

    size_t size = 0;
    PoolIndex next_stable_id = 0; 

    constexpr static std::size_t maxRemovals = 16;
    SmallSet<uint16_t, maxRemovals> indices_to_remove;

    ComponentPool() 
    {
        for (size_t i = 0; i < N; ++i) stable_to_dense[i] = NULL_INDEX;
    }   

    PoolIndex add(const T& component) 
    {
        ASSERT(size < N, "Component pool overflow");

        while (stable_to_dense[next_stable_id] != NULL_INDEX) 
        {
            next_stable_id = (next_stable_id + 1) % N;
        }

        PoolIndex stable_id   = next_stable_id;
        SparseIndex dense_idx = (SparseIndex)size;

        stable_to_dense[stable_id] = dense_idx;
        dense_to_stable[dense_idx] = stable_id;

        dense_components[dense_idx] = component;

        size++;
        next_stable_id = (next_stable_id + 1) % N;
        return stable_id;
    }

    void erase(PoolIndex stable_id) 
    {
        SparseIndex idx_to_remove = stable_to_dense[stable_id];
        ASSERT(idx_to_remove != NULL_INDEX, "Attempting to erase non-existent stable ID");

        size_t last_dense_idx = size - 1;
        PoolIndex last_stable_id = dense_to_stable[last_dense_idx];

        if (idx_to_remove != (SparseIndex)last_dense_idx) 
        {
            dense_components[idx_to_remove] = dense_components[last_dense_idx];
            dense_to_stable[idx_to_remove]  = last_stable_id;

            stable_to_dense[last_stable_id] = idx_to_remove;
        }

        stable_to_dense[stable_id] = NULL_INDEX;
        size--;
    }

    void flushRemovals()
    {
        if (indices_to_remove.empty()) return;

        for (auto idx : indices_to_remove) erase(idx);
        indices_to_remove.clear();
    }

    void setComponentToRemove(PoolIndex index) 
    {
        ASSERT(!indices_to_remove.contains(index), "Can not set components to remove twice");

        indices_to_remove.add(index); 
    }

    template<typename Func>
    void forEach(Func&& f) 
    {
        for (PoolIndex i = 0; i < size; ++i) 
        {
            f(i, dense_components[i]);
        }
    }

    template<typename Func>
    void forEach(Func&& f) const
    {
        for (PoolIndex i = 0; i < size; ++i) 
        {
            f(i, dense_components[i]);
        }
    }
};

// =======================================================
// END Component Pool
// =======================================================