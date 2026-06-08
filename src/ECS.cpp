#include <tiny_obj_loader.h>

#include <stdint.h>
#include <unordered_map>
#include <iostream>
#include <random>
#include <cmath>
#include <functional>
#include <set>
#include <concepts>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "ECS.h"
#include "Mesh.h"
#include "InputManager.h"
#include "VertexArray.h"
#include "VertexBuffer.h"
#include "Collision.h"
#include "Macro.h"

// REGISTRY

Real3 randColor()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(0.4f, 0.7f);

    // return glm::vec3(0.3 + dist(gen), 0.3 + dist(gen), 0.3 + dist(gen));
    return Real3(dist(gen));
};

AABB extractMeshGeometry(
    const tinyobj::attrib_t& attrib, 
    const tinyobj::mesh_t& mesh, 
    std::vector<float>& out_v,
    std::vector<uint32_t>& out_i)
{
    AABB aabb;
    bool first = true;

    for (const auto& idx : mesh.indices) 
    {
        float vx = attrib.vertices[3 * idx.vertex_index + 0];
        float vy = attrib.vertices[3 * idx.vertex_index + 1];
        float vz = attrib.vertices[3 * idx.vertex_index + 2];
        glm::vec3 v(vx, vy, vz);
        
        if (first) { aabb.minv = aabb.maxv = v; first = false; }
        else { aabb.minv = glm::min(aabb.minv, v); aabb.maxv = glm::max(aabb.maxv, v); }

        out_v.insert(out_v.end(), {vx, vy, vz});
        if (idx.normal_index >= 0) 
        {
            out_v.insert(out_v.end(), {attrib.normals[3 * idx.normal_index + 0], 
                                       attrib.normals[3 * idx.normal_index + 1], 
                                       attrib.normals[3 * idx.normal_index + 2]});
        } 
        else 
        {
            out_v.insert(out_v.end(), {0, 1, 0});
        }

        out_i.push_back(static_cast<uint32_t>(out_i.size()));
    }

    return aabb;
}

CollisionShape parseCollisionShape(const std::string& name, const AABB& aabb) 
{
    CollisionShape coll_shape;

    if (name.find("Cube") != std::string::npos) 
    {
        coll_shape.type = CollisionShapeType::AABB;
        coll_shape.aabb = aabb;
    } 
    else if (name.find("Stair") != std::string::npos) 
    {
        AARamp aaramp;

        if      (name.find("posx") != std::string::npos) aaramp.dir = AARampDirection::PosX;
        else if (name.find("negx") != std::string::npos) aaramp.dir = AARampDirection::NegX;
        else if (name.find("posy") != std::string::npos) aaramp.dir = AARampDirection::PosY;
        else if (name.find("negy") != std::string::npos) aaramp.dir = AARampDirection::NegY;
        else 
            ASSERT(false, "Unknown stair direction in OBJ file.");

        aaramp.minv       = aabb.minv;
        aaramp.maxv       = aabb.maxv;
        coll_shape.type   = CollisionShapeType::AARamp;
        coll_shape.aaramp = aaramp;
    } 
    else 
    {
        ASSERT(false, "Unknown ground object type in OBJ file.");
    }

    return coll_shape;
}

CollisionObjectType determineObjectType(const std::string& name)
{
    if (name.find("Cube") != std::string::npos) 
    {
        return CollisionObjectType::staticObject;
    } 
    else if (name.find("Stair") != std::string::npos) 
    {
        return CollisionObjectType::staticStair;
    } 
    else 
    {
        ASSERT(false, "Unknown ground object type in OBJ file.");
    }  
}

StaticPhysicsBody createStaticPhysicsBody(const CollisionShape& coll_shape, CollisionObjectType obj_type)
{
    StaticPhysicsBody phys_body;

    // phys_body.position      = Real3{0.0f, 0.0f, 0.0f};
    // phys_body.velocity      = Real3{0.0f, 0.0f, 0.0f};
    // phys_body.orientation   = Real3{0.0f, 0.0f, 0.0f};
    // phys_body.rest_shape    = coll_shape;
    phys_body.runtime_shape = coll_shape;
    phys_body.objectType    = obj_type;
    // phys_body.physicsType   = PhysicsBodyType::noPhysics;
    phys_body.aabb_tree_idx = nullAABBTreeIndex;

    return phys_body;
}

struct HouseMetadata 
{
    int house_idx = -1;
    int layer_idx = -1;
    bool is_valid = false;
};

HouseMetadata parseHouseName(const std::string& name) 
{
    HouseMetadata meta;

    int h = -1, l = -1;
    int matches = sscanf(name.c_str(), "H%d.L%d", &h, &l);

    if (matches == 2) {
        meta.house_idx = h;
        meta.layer_idx = l;
        meta.is_valid = true;
    }

    return meta;
}

CollisionObjectType houseDetermineObjectType(const std::string& name)
{
    if (name.find("Wall") != std::string::npos || name.find("Floor") != std::string::npos) 
    {
        return CollisionObjectType::staticObject;
    } 
    else if (name.find("Stair") != std::string::npos) 
    {
        return CollisionObjectType::staticStair;
    } 
    else 
    {
        ASSERT(false, "Unknown ground object type in OBJ file.");
    }  
}

CollisionShape houseParseCollisionShape(const std::string& name, const AABB& aabb) 
{
    CollisionShape coll_shape;

    if (name.find("Wall") != std::string::npos || name.find("Floor") != std::string::npos) 
    {
        coll_shape.type = CollisionShapeType::AABB;
        coll_shape.aabb = aabb;
    } 
    else if (name.find("Stair") != std::string::npos) 
    {
        AARamp aaramp;

        if      (name.find("posx") != std::string::npos) aaramp.dir = AARampDirection::PosX;
        else if (name.find("negx") != std::string::npos) aaramp.dir = AARampDirection::NegX;
        else if (name.find("posy") != std::string::npos) aaramp.dir = AARampDirection::PosY;
        else if (name.find("negy") != std::string::npos) aaramp.dir = AARampDirection::NegY;
        else 
            ASSERT(false, "Unknown stair direction in OBJ file.");

        aaramp.minv       = aabb.minv;
        aaramp.maxv       = aabb.maxv;
        coll_shape.type   = CollisionShapeType::AARamp;
        coll_shape.aaramp = aaramp;
    } 
    else 
    {
        ASSERT(false, "Unknown ground object type in OBJ file.");
    }

    return coll_shape;
}

void Registry::loadHouse(const std::string& file_path, std::vector<AABB>& bvh_boxes, std::vector<StaticAABBTreeNodeData>& bvh_data)
{
    Registry& registry = *this;

    static auto isHitbox = [](const tinyobj::shape_t &shape)
    {
        return (shape.name.find("Hitbox") != std::string::npos);
    };

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(std::string(ASSET_PATH) + file_path, tinyobj::ObjReaderConfig())) 
    {
        std::abort();
    }

    const auto& attrib = reader.GetAttrib();
    const auto& shapes = reader.GetShapes();

    int num_layers = -1;
    int num_trigger_layers = 0;
    int house_idx  = -1;

    Entity house_entity = registry.createEntity();

    std::vector<Entity>   static_layers_entities[HOUSE_MAX_LAYERS];
    std::vector<uint16_t> static_layers_phys_bodies[HOUSE_MAX_LAYERS];
    std::array<Entity, HOUSE_MAX_LAYERS> layer_trigger_entities;
    Entity layer_trigger_entity[HOUSE_MAX_LAYERS];
    bool layer_trigger_entity_found[HOUSE_MAX_LAYERS] = {false};
    
    for (const auto& shape : shapes) 
    {
        std::vector<float>    vertices;
        std::vector<uint32_t> indices;

        AABB aabb = extractMeshGeometry(attrib, shape.mesh, vertices, indices);

        HouseMetadata house_data = parseHouseName(shape.name);

        ASSERT(house_data.is_valid, "");
        ASSERT(house_data.layer_idx < HOUSE_MAX_LAYERS, "");

        if (house_idx == -1)
            house_idx = house_data.house_idx;
        else
            ASSERT(house_data.house_idx == house_idx, "");
    
        if (house_data.layer_idx > num_layers)
            num_layers =  house_data.layer_idx;

        if (!isHitbox(shape))
        {
            Mesh* mesh           = new Mesh(vertices.data(), vertices.size(), indices.data(), indices.size(), GL_STATIC_DRAW);
            size_t mesh_idx      = mesh_manager.addMesh(mesh);
            Entity visual_entity = assembleSceneObject(*this, mesh_manager.meshes[mesh_idx], randColor());

            CollisionShape coll_shape    = houseParseCollisionShape(shape.name, aabb);
            CollisionObjectType obj_type = houseDetermineObjectType(shape.name);

            StaticPhysicsBody phys_body = createStaticPhysicsBody(coll_shape, obj_type);
            static_world.push_back(phys_body);

            StaticAABBTreeNodeData data;
            data.entity     = visual_entity;
            data.object_idx = static_world.size() - 1;

            bvh_boxes.push_back(aabb);
            bvh_data.push_back(data);

            static_layers_entities[house_data.layer_idx].push_back(data.entity);
            static_layers_phys_bodies[house_data.layer_idx].push_back(data.object_idx);
        }
        else
        {
            ASSERT(layer_trigger_entity_found[house_data.layer_idx] == false, "");

            layer_trigger_entity_found[house_data.layer_idx] = true;

            Entity layer_entity = registry.createEntity();

            CollisionShape rest_coll_shape;
            rest_coll_shape.type = CollisionShapeType::AABB;
            rest_coll_shape.aabb = aabb;

            InteractableInfo inter_info(
                TriggerMode::OnAreaEnterAndExit, 
                InteractionType::HouseLayerToggle, 
                true, false, false, 
                rest_coll_shape, rest_coll_shape);

            HouseLayer house_layer;
            house_layer.house_entity = house_entity;
            house_layer.floor_level  = house_data.layer_idx;

            COMPONENT(layer_entity, InteractableInfo)   = inter_info;
            COMPONENT(layer_entity, HouseLayer)         = house_layer;
            COMPONENT(layer_entity, SimplePosition)     = { Real3(0.0f), Real3(0.0f) };
            COMPONENT(layer_entity, Type)               = {EntityType::HouseLayerToggle};

            layer_trigger_entities[house_data.layer_idx] = layer_entity;
            num_trigger_layers++;
        }
    }

    ASSERT(house_idx != -1, "");
    ASSERT(num_layers > -1, "");

    House house;

    house.size = num_layers+1;

    house.trigger_layer_size = num_trigger_layers;
    house.layer_trigger_entities = layer_trigger_entities;

    for (int i=0; i<house.size; i++)
    {
        ASSERT(static_layers_entities[i].size() ==  static_layers_phys_bodies[i].size(), "");
        ASSERT(static_layers_entities[i].size() != 0, "");

        for (auto entity : static_layers_entities[i])
        {
            house.layers[i].static_entities.add(entity);
        }

        for (auto object_idx : static_layers_phys_bodies[i])
        {
            house.layers[i].static_phys_bodies.add(object_idx);
        }
    }

    COMPONENT(house_entity, House) = house;
    COMPONENT(house_entity, Type)  = {EntityType::House};
}

void Registry::loadLevel(std::string file_path, const std::vector<std::string>& house_paths) 
{
    std::vector<AABB>                   bvh_boxes;
    std::vector<StaticAABBTreeNodeData> bvh_data;

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(std::string(ASSET_PATH) + file_path, tinyobj::ObjReaderConfig())) 
    {
        std::abort();
    }

    const auto& attrib = reader.GetAttrib();
    const auto& shapes = reader.GetShapes();

    for (const auto& shape : shapes) 
    {
        std::vector<float>    vertices;
        std::vector<uint32_t> indices;

        AABB aabb = extractMeshGeometry(attrib, shape.mesh, vertices, indices);

        Mesh* mesh = new Mesh(vertices.data(), vertices.size(), indices.data(), indices.size(), GL_STATIC_DRAW);
        size_t mesh_idx = mesh_manager.addMesh(mesh);
        Entity visual_entity = assembleSceneObject(*this, mesh_manager.meshes[mesh_idx], randColor());

        CollisionShape coll_shape    = parseCollisionShape(shape.name, aabb);
        CollisionObjectType obj_type = determineObjectType(shape.name);

        StaticPhysicsBody phys_body = createStaticPhysicsBody(coll_shape, obj_type);
        static_world.push_back(phys_body);

        StaticAABBTreeNodeData data;
        data.entity     = visual_entity;
        data.object_idx = static_world.size() - 1;

        bvh_boxes.push_back(aabb);
        bvh_data.push_back(data);
    }

    for (const auto& house_file_name : house_paths)
    {
        loadHouse(house_file_name, bvh_boxes, bvh_data);
    }

    std::cout << "loaded " << bvh_boxes.size() << " static world collision objects\n";

    static_world_bvh = constuctStaticAABBTree(bvh_boxes, bvh_data);

    std::cout << "number of nodes in static world AABB tree: " << static_world_bvh.node_count << "\n";
}

// DEBUG EFFECT FACTORY

static EffectType selected_effect = EffectType::Poisoned;

static const char* get_effect_name(EffectType type)
{
    switch (type)
    {
        case EffectType::Burnt:    return "Burnt";
        case EffectType::Poisoned: return "Poisoned";
        case EffectType::Healing:  return "Healing";
        case EffectType::Freezed:  return "Freezed";
        case EffectType::Slowed:   return "Slowed";
        default:                   return "Unknown";
    }
}

static void increment_selected_effect()
{
    size_t next_idx = toIndex(selected_effect) + 1;

    if (next_idx >= EffectTypeCount) 
    {
        next_idx = 0;
    }

    selected_effect = static_cast<EffectType>(next_idx);
    
    std::cout << "[Debug] Effect Switched: " << get_effect_name(selected_effect) 
              << " (Index: " << toIndex(selected_effect) << ")" << std::endl;
}

static void decrement_selected_effect()
{
    size_t current_idx = toIndex(selected_effect);

    if (current_idx == 0) 
    {
        current_idx = EffectTypeCount - 1;
    }
    else 
    {
        current_idx--;
    }

    selected_effect = static_cast<EffectType>(current_idx);

    std::cout << "[Debug] Effect Switched: " << get_effect_name(selected_effect) 
              << " (Index: " << toIndex(selected_effect) << ")" << std::endl;
}

static Effect get_selected_effect()
{
    switch(selected_effect)
    {
        case EffectType::Poisoned:
        {
            return {
                .type        = EffectType::Poisoned,
                .active      = true,
                .on_tick     = true,
                .strength    = 5.0f,
                .tick_length = 1.0f,
                .tick_timer  = 0.0f,
                .duration    = 5.0f,
                .timer       = 0.0f
            };
        }
        case EffectType::Burnt:
        {
            return {
                .type        = EffectType::Burnt,
                .active      = true,
                .on_tick     = true,
                .strength    = 10.0f, // Burnt might do more damage
                .tick_length = 0.5f,  // but faster ticks
                .tick_timer  = 0.0f,
                .duration    = 3.0f,
                .timer       = 0.0f
            };
        }
        case EffectType::Healing:
        {
            return {
                .type        = EffectType::Healing,
                .active      = true,
                .on_tick     = true,
                .strength    = 2.0f,
                .tick_length = 1.0f,
                .tick_timer  = 0.0f,
                .duration    = 10.0f,
                .timer       = 0.0f
            };
        }
        case EffectType::Freezed:
        {
            return {
                .type        = EffectType::Freezed,
                .active      = true,
                .on_tick     = false, // Continuous effect
                .strength    = 0.0f,  // 0% speed
                .tick_length = 0.0f,
                .tick_timer  = 0.0f,
                .duration    = 2.0f,
                .timer       = 0.0f
            };
        }
        case EffectType::Slowed:
        {
            return {
                .type        = EffectType::Slowed,
                .active      = true,
                .on_tick     = false, // Continuous effect
                .strength    = 0.5f,  // 50% speed
                .tick_length = 0.0f,
                .tick_timer  = 0.0f,
                .duration    = 4.0f,
                .timer       = 0.0f
            };
        }
        default:
        {
            ASSERT(false, "should never be here");
            return {};
        }
    }
}

// END DEBUG EFFECT FACTORY


// ASSEMBLEs

PhysicsBody createDynamicPhysicsBody(Registry& registry, Entity entity, Real3 pos, AABB localAABB) 
{
    PhysicsBody pb;
    pb.position    = pos;
    pb.velocity    = Real3(0.0f);
    pb.orientation = Real3(0.0f);
    
    pb.rest_shape.type = CollisionShapeType::AABB;
    pb.rest_shape.aabb = localAABB;

    updateCollisionShape(pb.runtime_shape, pb.rest_shape, pos);

    pb.objectType  = CollisionObjectType::dynamic;
    pb.physicsType = PhysicsBodyType::dynamic;

    pb.aabb_tree_idx = dynamicAABBTreeInsertLeaf(
        registry.dynamic_world_bvh, 
        pb.runtime_shape.aabb, 
        entity
    );

    return pb;
}

PhysicsBody createDynamicPhysicsBody(Registry& registry, Entity entity, Real3 pos, AACapsule localAACapsule) 
{
    PhysicsBody pb;
    pb.position    = pos;
    pb.velocity    = Real3(0.0f);
    pb.orientation = Real3(0.0f);
    
    pb.rest_shape.type      = CollisionShapeType::AACapsule;
    pb.rest_shape.aacapsule = localAACapsule;

    updateCollisionShape(pb.runtime_shape, pb.rest_shape, pos);

    pb.objectType  = CollisionObjectType::dynamic;
    pb.physicsType = PhysicsBodyType::dynamic;

    pb.aabb_tree_idx = dynamicAABBTreeInsertLeaf(
        registry.dynamic_world_bvh, 
        pb.runtime_shape.aabb, 
        entity
    );

    return pb;
}

constexpr AABB characterAABB = { Real3(-0.1, -0.1, 0.0), Real3(0.1, 0.1, 0.2) };

constexpr AACapsule characterAACapsule = { Axis::Z, Real3(0.0f, 0.0f, 0.1f), 0.2f, 0.1f};

Entity assemblePlayer(Registry& registry, Mesh* mesh) 
{
    Entity entity = registry.createEntity();

    PhysicsBody phys_body = createDynamicPhysicsBody(registry, entity, Real3(0.0f), characterAACapsule);
    COMPONENT(entity, PhysicsBody) = phys_body;

    ASSERT(mesh->m_type == MeshType::animated, "Player mesh must be of animated type");

    AnimationState anim_state;
    anim_state.state_mesh_map         = &registry.mesh_manager.player_state_mesh_map;
    anim_state.current_state          = anim_state.state_mesh_map->default_state;
    anim_state.previous_state         = anim_state.state_mesh_map->default_state;
    anim_state.currframe_played_steps = 0;

    COMPONENT(entity, AnimationState) = anim_state;

    RenderInfo render_info;
    const AnimStateInfo& anim_state_info = registry.mesh_manager.player_state_mesh_map.get(anim_state.current_state);
    
    render_info.color = Real3(1.0f, 0.5f, 0.2f);
    render_info.alpha = 1.0f;

    render_info.mesh                   = anim_state_info.mesh;
    render_info.fps                    = anim_state_info.fps;
    render_info.currframe_played_steps = 0;

    COMPONENT(entity, RenderInfo) = render_info;

    COMPONENT(entity, Type)   = Type{EntityType::Other}; 
    COMPONENT(entity, Status) = Status(100.0f);

    COMPONENT(entity, ParticleRendererInfo).particle_system.init(phys_body.position);

    registry.player_entity = entity;
    return entity;
}

Entity assembleRandomDynamic(Registry& registry, Mesh* mesh, Real3 position)
{
    Entity entity = registry.createEntity();

    PhysicsBody phys_body = createDynamicPhysicsBody(registry, entity, position, characterAACapsule);
    phys_body.move_speed = 1.0f;

    COMPONENT(entity, PhysicsBody) = phys_body;

    RandomDirectionMover rdm;
    rdm.change_interval = 2.0f;
    rdm.timer           = 0.0f;
    rdm.direction       = Real3(1.0f, 0.0f, 0.0f);

    COMPONENT(entity, RandomDirectionMover) = rdm;

    COMPONENT(entity, RenderInfo) = RenderInfo{mesh, Real3(0.0f, 0.8f, 0.2f), 1.0f};

    COMPONENT(entity, Type)       = Type{EntityType::Enemy}; 
    
    COMPONENT(entity, EnemyInfo)  = EnemyInfo{false};
    
    COMPONENT(entity, Status)     = Status(100.0f);

    COMPONENT(entity, ParticleRendererInfo).particle_system.init(position);

    return entity;
}

Entity assembleStaticNPC(Registry& registry, Mesh* mesh, Real3 position)
{
    Entity entity = registry.createEntity();

    PhysicsBody phys_body = createDynamicPhysicsBody(registry, entity, position, characterAABB);
    phys_body.move_speed = 0.0f;

    COMPONENT(entity, PhysicsBody) = phys_body;
    
    InteractableInfo inter_info;
    
    inter_info.interaction_type = InteractionType::Dialogue;

    AABB aabb = { Real3(-0.2, -0.2, 0.0), Real3(0.2, 0.2, 0.2) };

    inter_info.rest_shape.type = CollisionShapeType::AABB;
    inter_info.rest_shape.aabb = aabb;

    aabb.minv += position;
    aabb.maxv += position;
    inter_info.runtime_shape.type = CollisionShapeType::AABB;
    inter_info.runtime_shape.aabb = aabb;

    // inter_info.aabb_tree_idx = dynamicAABBTreeInsertLeaf(registry.dynamic_world_bvh, aabb, entity, true);

    inter_info.only_player = true;
    inter_info.clickable   = true;
    inter_info.hovered     = false;

    COMPONENT(entity, InteractableInfo) = inter_info;

    COMPONENT(entity, RenderInfo)  = RenderInfo{mesh, Real3(0.9f, 0.8f, 0.3f), 1.0f};
    COMPONENT(entity, Type)        = Type{EntityType::NPC}; 

    return entity;
}

Entity assembleStatusChangeArea(Registry& registry, Real3 position, Real radius, Real life_time)
{
    Entity entity = registry.createEntity();

    COMPONENT(entity, SimplePosition) = SimplePosition{position, Real3(0.0f)};

    // Collision/BVH 
    Sphere inter_sphere = {Real3(0.0f), radius};

    CollisionShape rest_shape;
    rest_shape.type   = CollisionShapeType::Sphere;
    rest_shape.sphere = inter_sphere;

    CollisionShape runtime_shape;
    inter_sphere.center += position;
    runtime_shape.type   = CollisionShapeType::Sphere;
    runtime_shape.sphere = inter_sphere;

    // Index aabb_tree_idx = dynamicAABBTreeInsertLeaf(registry.dynamic_world_bvh, computeSphereAABB(inter_sphere), entity, true);

    InteractableInfo inter_info(
        TriggerMode::OnAreaEnter,
        InteractionType::StatusChange,
        false,
        false,
        true,
        rest_shape,
        runtime_shape
    );

    COMPONENT(entity, InteractableInfo) = inter_info;

    InteractionHistory inter_history;

    COMPONENT(entity, InteractionHistory) = inter_history;

    StatusChangeInfo status_change_info;
    status_change_info.effect = get_selected_effect();;

    COMPONENT(entity, StatusChangeInfo) = status_change_info;

    COMPONENT(entity, LifeTime) = LifeTime{ life_time };

    COMPONENT(entity, Type) = Type{ EntityType::StatusChangeArea }; 

    return entity;
}

Entity assembleDoor(
    Registry& registry, Real3 position,
    Mesh* open_mesh, Mesh* closed_mesh, Real3 color,
    AABB open_coll_aabb, AABB closed_coll_aabb, bool locked)
{
    Entity entity = registry.createEntity();

    PhysicsBody phys_body;
    phys_body.position     = position;
    phys_body.velocity     = Real3{0.0f, 0.0f, 0.0f};
    phys_body.orientation  = Real3{0.0f, 0.0f, 0.0f};
    phys_body.move_speed = 0.0f;

    phys_body.rest_shape.type = CollisionShapeType::AABB;
    phys_body.rest_shape.aabb = closed_coll_aabb;

    AABB init_runtime_shape  = closed_coll_aabb;
    init_runtime_shape.minv += position;
    init_runtime_shape.maxv += position;
    phys_body.runtime_shape.type = CollisionShapeType::AABB;
    phys_body.runtime_shape.aabb = init_runtime_shape;

    phys_body.objectType  = CollisionObjectType::staticObject;
    phys_body.physicsType = PhysicsBodyType::noPhysics;

    phys_body.aabb_tree_idx = dynamicAABBTreeInsertLeaf(registry.dynamic_world_bvh, closed_coll_aabb, entity);
    
    COMPONENT(entity, PhysicsBody) = phys_body;

    RenderInfo rend_info;
    rend_info.mesh  = closed_mesh;
    rend_info.color = color;
    rend_info.alpha = 1.0f;

    COMPONENT(entity, RenderInfo) = rend_info;

    DoorInfo door_info;
    door_info.open     = false;
    door_info.was_open = false;
    door_info.locked   = locked;

    door_info.closed_mesh = closed_mesh;
    door_info.open_mesh   = open_mesh;

    CollisionShape closed_shape;
    closed_shape.type = CollisionShapeType::AABB;
    closed_shape.aabb = closed_coll_aabb;

    CollisionShape open_shape;
    open_shape.type = CollisionShapeType::AABB;
    open_shape.aabb = open_coll_aabb;

    door_info.open_shape   = open_shape;
    door_info.closed_shape = closed_shape;

    COMPONENT(entity, DoorInfo) = door_info;

    AACylinder aacyl = { Real3(0.0f), 0.3f, 0.2f, Axis::Z};

    CollisionShape rest_shape;
    rest_shape.type       = CollisionShapeType::AACylinder;
    rest_shape.aacylinder = aacyl;

    CollisionShape runtime_shape;
    aacyl.base_center += position;
    runtime_shape.type       = CollisionShapeType::AACylinder;
    runtime_shape.aacylinder = aacyl;

    // Index aabb_tree_idx = dynamicAABBTreeInsertLeaf(registry.dynamic_world_bvh, computeAACylinderAABB(aacyl), entity, true);

    InteractableInfo inter_info(
        TriggerMode::OnAreaStay, 
        InteractionType::Door, 
        false,
        false,
        false,
        rest_shape, runtime_shape
    );

    COMPONENT(entity, InteractableInfo) = inter_info; 

    COMPONENT(entity, Type) = Type{EntityType::Door}; 

    return entity;
}


Entity assembleHouseLayerToggle(Registry& registry, Real3 position)
{
    Entity entity = registry.createEntity();

    SimplePosition simp_position = {position, Real3(0.0f)};

    COMPONENT(entity, SimplePosition) = simp_position;

    CollisionShape rest_shape;
    rest_shape.type = CollisionShapeType::AABB;
    rest_shape.aabb = {Real3(-0.5f, -0.5f, 0.0f), Real3(0.5f, 0.5f, 1.0f)};

    CollisionShape runtime_shape;
    updateCollisionShape(runtime_shape, rest_shape, position);

    InteractableInfo inter_info(
                        TriggerMode::OnAreaEnterAndExit, 
                        InteractionType::HouseLayerToggle, 
                        true, false, false, 
                        rest_shape, 
                        runtime_shape);

    COMPONENT(entity, InteractableInfo) = inter_info;

    InteractionHistory inter_history;

    COMPONENT(entity, InteractionHistory) = inter_history;

    COMPONENT(entity, Type) = Type{EntityType::HouseLayerToggle}; 

    return entity;
}

Entity assemblePoolElementDestructor(Registry& registry, Real3 position, ComponentPoolID pool_id, PoolIndex index)
{
    Entity entity = registry.createEntity();

    COMPONENT(entity, SimplePosition) = SimplePosition{ position, Real3(0.0f) };

    const Real radius = 0.3f;
    Sphere trigger_sphere = { Real3(0.0f), radius };

    CollisionShape rest_shape;
    rest_shape.type   = CollisionShapeType::Sphere;
    rest_shape.sphere = trigger_sphere;

    CollisionShape runtime_shape;
    updateCollisionShape(runtime_shape, rest_shape, position);

    InteractableInfo inter_info(
        TriggerMode::OnAreaEnter, 
        InteractionType::PoolElementDestruction, 
        true,  // only_player:   TRUE
        false, // clickable:     FALSE
        false, // affects_actor: FALSE
        rest_shape, 
        runtime_shape
    );

    COMPONENT(entity, InteractableInfo) = inter_info;

    PoolDestructionInfo dest_info;
    dest_info.type_id = pool_id;
    dest_info.idx     = index;

    COMPONENT(entity, PoolDestructionInfo) = dest_info;

    COMPONENT(entity, Type) = Type{ EntityType::PoolElementDestructor }; 

    return entity;
}



template<typename ShapeAssignFunc, typename AABBFunc>
Entity assembleStaticObjectBase(
    Registry&           registry, 
    Mesh*               mesh, 
    Real3               position, 
    Real3               color, 
    CollisionShapeType  type, 
    ShapeAssignFunc     assignShape, 
    AABBFunc            getAABB, 
    CollisionObjectType object_type = CollisionObjectType::staticObject)
{
    Entity entity = registry.createEntity();

    PhysicsBody phys_body;
    phys_body.position    = position;
    phys_body.velocity    = Real3(0.0f);
    phys_body.orientation = Real3(0.0f);
    phys_body.move_speed  = 0.0f;
    phys_body.objectType  = object_type;
    phys_body.physicsType = PhysicsBodyType::noPhysics;

    phys_body.rest_shape.type    = type;
    phys_body.runtime_shape.type = type;
    
    assignShape(phys_body.rest_shape);

    updateCollisionShape(phys_body.runtime_shape, phys_body.rest_shape, position);

    AABB runtime_aabb = getAABB(phys_body.runtime_shape);
    phys_body.aabb_tree_idx = dynamicAABBTreeInsertLeaf(registry.dynamic_world_bvh, runtime_aabb, entity);

    COMPONENT(entity, PhysicsBody) = phys_body;

    RenderInfo rend_info = RenderInfo{mesh, color, 1.0f};
    rend_info.visible    = false;
    COMPONENT(entity, RenderInfo) = rend_info;

    COMPONENT(entity, Type) = Type{EntityType::Object};

    return entity;
}

Entity assembleStaticAACapsuleObject(Registry& registry, Mesh* mesh, Real3 position, Real radius, Real height, Axis axis, Real3 color) 
{
    return assembleStaticObjectBase(registry, mesh, position, color, CollisionShapeType::AACapsule,
        [&](CollisionShape& s) {
            s.aacapsule = { .axis = axis, .start = Real3(0.0f), .height = height, .radius = radius, };
        },
        [](const CollisionShape& s) { return computeAACapsuleAABB(s.aacapsule); }
    );
}

Entity assembleStaticCapsuleObject(Registry& registry, Mesh* mesh, Real3 position, Real3 direction, Real radius, Real3 color)
{
    return assembleStaticObjectBase(registry, mesh, position, color, CollisionShapeType::Capsule,
        [&](CollisionShape& s) 
        {
            s.capsule = { .start = Real3(0.0f), .end = direction, .radius = radius,  };
        },
        [](const CollisionShape& s) 
        { 
            return computeCapsuleAABB(s.capsule); 
        }
    );
}

Entity assembleStaticCylinderObject(Registry& registry, Mesh* mesh, Real3 position, Real radius, Real height, Axis axis, Real3 color) 
{
    return assembleStaticObjectBase(registry, mesh, position, color, CollisionShapeType::AACylinder,
        [&](CollisionShape& s) 
        {
            s.aacylinder = { .base_center = Real3(0.0f), .radius = radius, .height = height, .axis = axis };
        },
        [](const CollisionShape& s) 
        { 
            return computeAACylinderAABB(s.aacylinder); 
        }
    );
}

Entity assembleStaticSphereObject(Registry& registry, Mesh* mesh, Real3 position, Real radius, Real3 color) 
{
    return assembleStaticObjectBase(registry, mesh, position, color, CollisionShapeType::Sphere,
        [&](CollisionShape& s) 
        {
            s.sphere = { .center = Real3(0.0f), .radius = radius };
        },
        [](const CollisionShape& s) 
        { 
            return computeSphereAABB(s.sphere); 
        }
    );
}

Entity assembleStaticAARectangleObject(Registry& registry, Mesh* mesh, Real3 position, Real3 extent, Axis axis, Real3 color) 
{
    return assembleStaticObjectBase(registry, mesh, position, color, CollisionShapeType::AARectangle,
        [&](CollisionShape& s) 
        {
            s.aarect = { .axis = axis, .p1 = Real3(0.0f), .p2 = extent };
        },
        [](const CollisionShape& s) 
        { 
            return computeAARectangleAABB(s.aarect); 
        }
    );
}

Entity assembleStaticAxialOBBObject(Registry& registry, Real3 position, Real3 extents, Real angle, Axis axis)
{
    return assembleStaticObjectBase(registry, nullptr, position, Real3(0.0f), CollisionShapeType::AxialOBB,
        [&](CollisionShape& s) 
        {
            s.axialobb = { .center = Real3(0.0f), .extents = extents, .angle = angle, .rotation_axis = axis };
        },
        [](const CollisionShape& s) 
        { 
            return computeAxialOBBAABB(s.axialobb); 
        }
    );
}

Entity assembleStaticAA4SidedPyramidObject(Registry& registry, Real3 position, Real2 half_extents, Real height, Axis axis, CollisionObjectType obj_type)
{
    return assembleStaticObjectBase(registry, nullptr, position, Real3(0.0f), CollisionShapeType::AA4SidedPyramid,
        [&](CollisionShape& s) 
        {
            s.aa4pyramid = { .base_center = Real3(0.0f), .half_extents = half_extents, .height = height, .axis = axis };
        },
        [](const CollisionShape& s) 
        { 
            return computeAA4SidedPyramidAABB(s.aa4pyramid); 
        }, 
        obj_type
    );
}

Entity assembleStaticAAEllipsoidObject(Registry& registry, Real3 position, Real3 half_extents)
{
    return assembleStaticObjectBase(registry, nullptr, position, Real3(0.0f), CollisionShapeType::AAEllipsoid,
        [&](CollisionShape& s) 
        {
            s.aaellipsoid= { .center = Real3(0.0f), .half_extents = half_extents };
        },
        [](const CollisionShape& s) 
        { 
            return computeAAEllipsoidAABB(s.aaellipsoid); 
        }
    );
}

Entity assembleStaticAAConeObject(Registry& registry, Real3 position, Real radius, Real height, Axis axis)
{
    return assembleStaticObjectBase(registry, nullptr, position, Real3(0.0f), CollisionShapeType::AACone,
        [&](CollisionShape& s) 
        {
            s.aacone = { .base_center = Real3(0.0f), .radius = radius, .height = height, .axis = axis };
        },
        [](const CollisionShape& s) 
        { 
            return computeAAConeAABB(s.aacone); 
        }
    );
}

Entity assembleStaticAAHollowCylinderObject(Registry& registry, Real3 position, Real inner_radius, Real outer_radius, Real height, Axis axis)
{
    return assembleStaticObjectBase(registry, nullptr, position, Real3(0.0f), CollisionShapeType::AAHollowCylinder,
        [&](CollisionShape& s) 
        {
            s.aahollcyl = { .base_center = Real3(0.0f), .inner_radius = inner_radius, .outer_radius = outer_radius, .height = height, .axis = axis };
        },
        [](const CollisionShape& s) 
        { 
            return computeAAHollowCylinderAABB(s.aahollcyl); 
        }
    );
}

Entity assembleStaticAAHemisphereObject(Registry& registry, Real3 position, Real radius, AxisDir axis)
{
    return assembleStaticObjectBase(registry, nullptr, position, Real3(0.0f), CollisionShapeType::AAHemisphere,
        [&](CollisionShape& s) 
        {
            s.aahemisphere = { .center = Real3(0.0f), .radius = radius, .axis = axis };
        },
        [](const CollisionShape& s) 
        { 
            return computeAAHemisphereAABB(s.aahemisphere); 
        }
    );
}

Entity assembleStaticAATrapezoidObject(Registry& registry, Real3 position, Real2 half_extents, Real2 top_half_extents, Real height, Axis axis)
{
    return assembleStaticObjectBase(registry, nullptr, position, Real3(0.0f), CollisionShapeType::AATrapezoid,
        [&](CollisionShape& s) 
        {
            s.aatrapezoid = { .base_center = Real3(0.0f), .half_extents = half_extents, .top_half_extents = top_half_extents, .height = height, .axis = axis };
        },
        [](const CollisionShape& s) 
        { 
            return computeAATrapezoidAABB(s.aatrapezoid); 
        }
    );
}

Entity assembleStaticAATetrahedronObject(Registry& registry, Real3 position, Real3 extents)
{
    return assembleStaticObjectBase(registry, nullptr, position, Real3(0.0f), CollisionShapeType::AATetrahedron,
        [&](CollisionShape& s) 
        {
            s.aatetra = { .corner = Real3(0.0f), .extents = extents };
        },
        [](const CollisionShape& s) 
        { 
            return computeAATetrahedronAABB(s.aatetra); 
        }
    );  
}

Entity assembleStaticAARampObject(Registry& registry, Real3 position, Real3 pos_extents, AARampDirection dir)
{
    ASSERT(pos_extents.x > 0.0f, "");
    ASSERT(pos_extents.y > 0.0f, "");
    ASSERT(pos_extents.z > 0.0f, "");

    return assembleStaticObjectBase(registry, nullptr, position, Real3(0.0f), CollisionShapeType::AARamp,
        [&](CollisionShape& s) 
        {
            s.aaramp = { .minv= Real3(0.0f), .maxv = pos_extents, .dir = dir };
        },
        [](const CollisionShape& s) 
        { 
            return computeAARampAABB(s.aaramp); 
        }
    );  
}


/*
// Entity assemblePathObject(
//         Registry& registry, 
//         Mesh* mesh, 
//         glm::vec3& color, 
//         glm::vec3& position, 
//         AABBShape& aabb, 
//         std::vector<glm::vec3> pathPoints, 
//         float speed) {

//     Entity entity = registry.createEntity();
//     COMPONENT(entity, Position)           = Position{position, glm::vec3(0.0), glm::vec3(0.0)};
//     COMPONENT(entity, Velocity)           = Velocity{glm::vec3(0.0), 1.0f};
//     COMPONENT(entity, RenderInfo)         = RenderInfo{mesh, color};
//     COMPONENT(entity, PhysicsBody)        = PhysicsBody{false, false};
//     COMPONENT(entity, Type)               = Type{EntityType::Object}; 
//     COMPONENT(entity, CollisionComponent) = CollisionComponent{CollisionResponseType::Static, CollisionShapeType::AABB, {aabb}};
//     COMPONENT(entity, PathFollower)       = PathFollower{pathPoints, 1, speed, 0.05, true};

//     return entity;
// }
*/

Entity assembleSceneObject(Registry& registry, Mesh* mesh, glm::vec3 color) 
{
    Entity entity = registry.createEntity();
    COMPONENT(entity, RenderInfo)     = RenderInfo{mesh, color, 1.0f};
    COMPONENT(entity, SimplePosition) = SimplePosition{glm::vec3(0.0), glm::vec3(0.0)};
    return entity;
}

Entity assembleProjectile(Registry& registry, ProjectileType proj_type, Real3 position, Real3 direction, float speed, Entity shooter)
{
    Entity entity = registry.createEntity();

    Segment seg = {Real3(0.0f), Real3(0.0f)};

    CollisionShape segment_shape;
    segment_shape.type    = CollisionShapeType::Segment;
    segment_shape.segment = seg;

    Mesh* mesh = registry.mesh_manager[projectile_type_traits[toIndex(proj_type)].mesh_name];
    COMPONENT(entity, RenderInfo) = RenderInfo{mesh, Real3(0.2, 0.3, 1.0), 1.0f};

    ProjectileInfo proj_info;
    proj_info.type          = proj_type;
    proj_info.shooter       = shooter;
    proj_info.init_position = position;
    proj_info.init_velocity = speed * direction;

    proj_info.position      = Real3{position.x, position.y, position.z};
    proj_info.velocity      = speed * direction;
    proj_info.orientation   = Real3{0.0f, 0.0f, 0.0f};
    proj_info.runtime_shape = segment_shape;

    COMPONENT(entity, ProjectileInfo) = proj_info;

    COMPONENT(entity, Type) = Type{ EntityType::Projectile }; 

    return entity;
}


Entity assembleStaticIdleProjectile(Registry& registry, Mesh* mesh, Real3 position, Real3 orientation) 
{
    Entity entity = registry.createEntity();

    COMPONENT(entity, SimplePosition) = SimplePosition{position, orientation};
    COMPONENT(entity, RenderInfo)     = RenderInfo{mesh, Real3(0.2, 0.3, 0.1), 1.0f};
    COMPONENT(entity, Type)           = Type{EntityType::IdleProjectile}; 
    COMPONENT(entity, LifeTime)       = LifeTime{2.0f};

    return entity;
}

Entity assembleAttachedIdleProjectile(
    Registry& registry, 
    Mesh*  mesh, 
    Real3  position_attach, 
    Real3  starting_projectile_orientation,
    Entity target) 
{
    HAS_ASSERT(target, PhysicsBody);
    const PhysicsBody& target_phys_body = GET_COMPONENT(target, PhysicsBody);

    Real3 starting_position = target_phys_body.position + position_attach;
    Real3 starting_target_orientation = target_phys_body.orientation;

    Entity entity = 
        assembleStaticIdleProjectile(
            registry, 
            mesh, 
            starting_position, 
            starting_projectile_orientation);

    COMPONENT(entity, Attachment) = Attachment{ 
        target, 
        position_attach, 
        starting_target_orientation,
        starting_projectile_orientation
    };

    return entity;
}


void initCamera(Registry& registry, Entity target) 
{
    HAS_ASSERT(target, PhysicsBody);
    const PhysicsBody& target_phys_body = GET_COMPONENT(target, PhysicsBody);

    glm::vec3 offset(0.0f, -3.0f, 7.0f);
    glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);

    CameraInfo cam_info;

    cam_info.position     = target_phys_body.position + offset;
    cam_info.offset       = offset;
    cam_info.target       = target;
    cam_info.view         = glm::lookAt(cam_info.position, target_phys_body.position, cameraUp);
    cam_info.projection   = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
    cam_info.max_distance = 0.2f;
    cam_info.vel          = 1.0f;

    registry.camera = cam_info;

    return;
}

void initCrosshair(Registry& registry, Mesh* mesh) 
{
    HAS_ASSERT(registry.player_entity, PhysicsBody);
    const PhysicsBody& target_phys_body = GET_COMPONENT(registry.player_entity, PhysicsBody);

    registry.crosshair.position = target_phys_body.position;
    registry.crosshair.target   = registry.player_entity;
    registry.crosshair.z_offset = 0.2f;
    registry.crosshair.mesh     = mesh;
    registry.crosshair.color    = Real3(1.0f, 1.0f, 1.0f);

    return;
}

// SYSTEMs

namespace ShapeMask 
{
    enum : uint32_t 
    {
        #define X(type_name, union_name) type_name = 1 << (int)CollisionShapeType::type_name, 
        COLLISION_TYPES
        #undef X

        Static =    AABB             |
                    AACapsule        | 
                    AACylinder       | 
                    Capsule          | 
                    Sphere           | 
                    AARectangle      | 
                    AxialOBB         | 
                    AA4SidedPyramid  | 
                    AAEllipsoid      |
                    AACone           |
                    AAHollowCylinder |
                    AAHemisphere     |
                    AATrapezoid      |
                    AATetrahedron    |
                    AARamp,
        Stair        = AARamp | AA4SidedPyramid,
        Dynamic      = AABB | AACapsule, 
        Interactable = AABB | AACylinder | Sphere
    };
}

void assertSystem(Registry &registry)
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    CONST_MAP(PhysicsBody).forEach([](Entity entity, const PhysicsBody& phys_body) 
    {
        uint32_t shapeBit = 1 << (int)phys_body.runtime_shape.type;

        switch (phys_body.objectType) 
        {
            case CollisionObjectType::staticObject:
                ASSERT(shapeBit & ShapeMask::Static, "Invalid shape for static object"); break;
            case CollisionObjectType::staticStair:
                ASSERT(shapeBit & ShapeMask::Stair, "Invalid shape for stair object"); break;
            case CollisionObjectType::dynamic:
                ASSERT(shapeBit & ShapeMask::Dynamic, "Invalid shape for dynamic object"); break;
            default:
                ASSERT(false, "invalid physics object collision type");
        }
    });

    for (const auto& phys_body : registry.static_world)
    {
        uint32_t shapeBit = 1 << (int)phys_body.runtime_shape.type;

        switch (phys_body.objectType) 
        {
            case CollisionObjectType::staticObject:
                ASSERT(shapeBit & ShapeMask::Static, "Invalid shape for static object"); break;
            case CollisionObjectType::staticStair:
                ASSERT(shapeBit & ShapeMask::Stair, "Invalid shape for stair object"); break;
            default:
                ASSERT(false, "invalid static physics object collision type");
        }
    }

    CONST_MAP(InteractableInfo).forEach([&](auto entity, auto& inter_info)
    {
        uint32_t shapeBit = 1 << (int)inter_info.runtime_shape.type;

        ASSERT(shapeBit & ShapeMask::Interactable, "Invalid shape for interactable shape");
    }
    );

    CONST_MAP(ProjectileInfo).forEach([&](auto entity, auto& proj_info)
    {
        ASSERT(proj_info.runtime_shape.type == CollisionShapeType::Segment, "projectile must have segment collision shape");
    }
    );

    PROFILE_SYSTEM_END();
}



// =======================================================
//  UPDATE SYSTEM
// =======================================================

void updateDoors(Registry &registry) 
{
    TRACE_SYSTEM();

    View(MAPS(DoorInfo, PhysicsBody, RenderInfo)).forEachStrict(
    [&](Entity entity, DoorInfo& door_info, PhysicsBody& phys_body, RenderInfo& render_info)
    {
        ASSERT(door_info.open_shape.type   == CollisionShapeType::AABB,          "Only AABB door shapes are supported.");
        ASSERT(door_info.closed_shape.type == CollisionShapeType::AABB,          "Only AABB door shapes are supported.");
        ASSERT((door_info.locked && !door_info.open) || (!door_info.locked),     "Door can not be locked and open at the same time.");
        ASSERT(registry.dynamic_world_bvh.indexIsValid(phys_body.aabb_tree_idx), "Door entity has invalid AABB tree index.");
        ASSERT(phys_body.objectType == CollisionObjectType::staticObject,        "Door entity must be of static object type.");

        if (door_info.open != door_info.was_open)
        {
            phys_body.rest_shape = door_info.open ? door_info.open_shape : door_info.closed_shape;
            render_info.mesh     = door_info.open ? door_info.open_mesh  : door_info.closed_mesh;
        }

        // TODO: door switching: handle attached entities (remove them??, reinsert them??)

        door_info.was_open = door_info.open;
        door_info.open     = false;
    }
    );
}

void updateParticleRenderers(Registry &registry, Real delta_time)
{
    // TODO: this should be done for the shader just once
    for (const auto& [entity, particle_renderer] : registry.m_ParticleRendererInfo)
    {
        particle_renderer.particle_system.update(registry.shader_manager.particle_shader, delta_time);
    }
}


void updateSystem(Registry &registry, Real delta_time) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    updateDoors(registry);
    // updateParticleRenderers(registry, delta_time);

    PROFILE_SYSTEM_END();
}

// =======================================================
//  END UPDATE SYSTEM
// =======================================================



// =======================================================
//  RESET SYSTEM
// =======================================================

template<typename T>
concept Resettable = requires(T t) 
{
    { t.reset() } -> std::same_as<void>; 
};

template<typename Component, std::size_t N>
void resetComponents(ComponentMap<Component, N>& map) 
{
    if constexpr (Resettable<Component>) 
    {
        map.forEach([](Entity e, Component& c) 
        {
            c.reset();
        });
    }
}

#define RESET_COMPONENT(type) resetComponents(MAP(type))

void resetSystem(Registry &registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    #define X(type, size, destructor) RESET_COMPONENT(type);
        COMPONENT_LIST
    #undef X

    PROFILE_SYSTEM_END();
}

// =======================================================
//  END RESET SYSTEM
// =======================================================



void playerInputSystem(Registry& registry, InputManager& input_manager) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    Entity player = registry.player_entity;

    HAS_ASSERT(player, PhysicsBody);
    HAS_ASSERT(player, AnimationState);

    PhysicsBody &phys_body     = GET_COMPONENT(player, PhysicsBody); 
    AnimationState &anim_state = GET_COMPONENT(player, AnimationState);

    glm::vec2 v(0.0);

    phys_body.move_intent.x = 0.0;
    phys_body.move_intent.y = 0.0;
    phys_body.move_intent.z = 0.0;

    phys_body.move_speed = 2.0f;

    if (input_manager.isPressed(KEY(W)))  v.y =  1.0;
    if (input_manager.isPressed(KEY(S)))  v.y = -1.0;
    if (input_manager.isPressed(KEY(A)))  v.x = -1.0;
    if (input_manager.isPressed(KEY(D)))  v.x =  1.0;
    if (input_manager.isPressed(KEY(M)))  phys_body.move_speed =  4.0f; 

    if (input_manager.isClicked(KEY(SPACE))) phys_body.velocity.z += 5.0f;

    if (input_manager.isClicked(KEY(LEFT)))  decrement_selected_effect();
    if (input_manager.isClicked(KEY(RIGHT))) increment_selected_effect();

    if (glm::length(v) > 1e-3)
    {
        v = glm::normalize(v);

        phys_body.move_intent.x = v.x;
        phys_body.move_intent.y = v.y;

        float angle = std::atan2(v.y, v.x) - glm::radians(90.0); 

        phys_body.orientation    = Real3(0.0) ;
        phys_body.orientation.y  = angle;
        anim_state.current_state = AnimState::moving;
    }
    else
    {
        phys_body.orientation    = Real3(0.0);
        anim_state.current_state = AnimState::idle;
    }

    // PROJECTILE

    if (registry.input_manager.isMouseClicked(MOUSE(RIGHT))) 
    {
        uint8_t next_idx = static_cast<uint8_t>(registry.equipped_projectile) + 1;
        next_idx %= static_cast<uint8_t>(ProjectileType::COUNT);
        registry.equipped_projectile = static_cast<ProjectileType>(next_idx);
    }

    ProjectileEjectionMode current_eject_mode = getTrait(registry.equipped_projectile).ejection_mode;

    static auto update_ejection_velocity = [&](Real update)
    {
        switch(current_eject_mode)
        {
            case ProjectileEjectionMode::Shooting:
            {
                registry.shooting_speed = glm::clamp(registry.shooting_speed + update, MIN_EJECT_SPEED, MAX_EJECT_SPEED);
                return registry.shooting_speed;
            }
            case ProjectileEjectionMode::Throwing:
            {
                registry.throwing_speed = glm::clamp(registry.throwing_speed + update, MIN_EJECT_SPEED, MAX_EJECT_SPEED);
                return registry.throwing_speed;
            }
            default:
            {
                ASSERT(false, "Cant update projectile speed, invalid projectile type");
            }
        }
    };

    Real eject_speed_update = registry.input_manager.scroll() * 0.3f;
    update_ejection_velocity(eject_speed_update);

    // INPUT SHOOTING LOGIC

    if (registry.trajectory.valid == true && registry.input_manager.isMouseClicked(MOUSE(LEFT))) 
    {
        assembleProjectile(
            registry, 
            registry.equipped_projectile, 
            registry.trajectory.x0, 
            registry.trajectory.direction, 
            registry.trajectory.v0, 
            registry.player_entity);
    }

    // DEBUG INPUT SYSTEM

    if (registry.input_manager.isClicked(KEY(P)))
    {
        const auto& p = phys_body.position;
        std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")\n";
    }

    PROFILE_SYSTEM_END();

    return;
}

void movementSystem(Registry& registry, Real delta_time)
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    static auto randDirection = []() 
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<float> dist(0.0f, 360.0f);

        float angleDeg = dist(gen);
        float angleRad = glm::radians(angleDeg);

        return Real3{
            std::cos(angleRad),
            std::sin(angleRad),
            0.0f
        };
    };

    View(MAPS(RandomDirectionMover, PhysicsBody)).forEachStrict(
    [&](Entity entity, RandomDirectionMover& rdm, PhysicsBody& phys_body)
    {
        phys_body.move_intent.x = 0.0;
        phys_body.move_intent.y = 0.0;
        phys_body.move_intent.z = 0.0;

        rdm.timer += delta_time;

        if (rdm.timer > rdm.change_interval)
        {
            rdm.timer = 0.0f;
            rdm.direction = randDirection();
        }

        Real3 v = glm::normalize(rdm.direction);

        phys_body.move_intent.x = v.x;
        phys_body.move_intent.y = v.y;
    }
    );

    PROFILE_SYSTEM_END();
}



struct TrajectoryResult 
{
    Real3 direction;
    Real  te;
    bool  valid;
};

TrajectoryResult solveTrajectory(const ProjectileTypeTrait& trait, Real3 x0, Real3 target, Real v0, Real g)
{   
    Real3 diff      = target - x0;

    switch(trait.trajectory_type)
    {
        case ProjectileTrajectoryType::Parabolic:
        {
            const Real xe = glm::length(Real3(diff.x, diff.y, 0.0f));;
            const Real h0 = x0.z - target.z;

            Real discriminant = v0 * v0 * v0 * v0 - g * (g * xe * xe - 2.0f * h0 * v0 * v0);

            if (discriminant < 0.0f)
            {
                return { .valid = false };
            }

            Real tan_theta1 = (v0 * v0 + sqrt(discriminant)) / (g * xe);
            Real tan_theta2 = (v0 * v0 - sqrt(discriminant)) / (g * xe);

            Real angle1 = std::atanf(tan_theta1);
            Real angle2 = std::atanf(tan_theta2);

            Real elevation = glm::min(angle1, angle2);

            Real3 horizontal_dir = glm::normalize(Real3(diff.x, diff.y, 0.0f));
            
            Real3 direction = Real3(
                horizontal_dir.x * cosf(elevation),
                horizontal_dir.y * cosf(elevation),
                sinf(elevation)
            );

            Real  te = xe / (v0 * cos(elevation));

            return {
                .direction = direction,
                .te        = te,
                .valid     = true
            };
        }
        case ProjectileTrajectoryType::Straight3D:
        {
            return {
                .direction = glm::normalize(diff),
                .te        = glm::length(diff) / v0,
                .valid     = true
            };
        }
        case ProjectileTrajectoryType::Straight2D:
        {
            return {
                .direction = glm::normalize(Real3(diff.x, diff.y, 0.0f)),
                .te        = glm::length(Real2(diff.x, diff.y)) / v0,
                .valid     = true
            };
        }
        default:
        {
            ASSERT(false, "unreachable");
        }
    }
}

void updateAimLocking(Registry& registry, const ProjectileTypeTrait& trait, const TrajectoryResult& traj_result, const Real3& x0, Real v0, Real g)
{
    AABB trajectory_aabb;
    
    Segment     segments[CROSSHAIR_LINE_SEGMENTS];
    FastSegment fast_segments[CROSSHAIR_LINE_SEGMENTS];
    int num_segments = 0;

    if (trait.trajectory_type == ProjectileTrajectoryType::Parabolic)
    {
        Real3 current_p = x0;
        trajectory_aabb = { x0, x0 };
        num_segments    = CROSSHAIR_LINE_SEGMENTS - 1;

        for (int i = 1; i < CROSSHAIR_LINE_SEGMENTS; ++i)
        {
            Real t = traj_result.te * (static_cast<Real>(i) / static_cast<Real>(CROSSHAIR_LINE_SEGMENTS - 1));
            Real3 next_p = x0 + (traj_result.direction * v0) * t + Real3(0.0f, 0.0f, -0.5f * g * t * t);

            segments[i - 1]      = { current_p, next_p };
            fast_segments[i - 1] = { current_p, 1.0f / (next_p - current_p)};
            trajectory_aabb.expand(next_p);
            current_p = next_p;
        }
    }
    else 
    {
        ASSERT_ONE_OF(trait.trajectory_type, "here must be one of straight types", ProjectileTrajectoryType::Straight2D, ProjectileTrajectoryType::Straight3D);

        Real3 end_p      = x0 + (traj_result.direction * v0) * traj_result.te;
        segments[0]      = { x0, end_p };
        fast_segments[0] = { x0, 1.0f / (end_p - x0)};
        num_segments     = 1;
        trajectory_aabb  = computeSegmentAABB(segments[0]);
    }

    trajectory_aabb.enlarge(0.05f);

    QueryOverlapResult<DynamicAABBTreeNodeData> possible_collisions;
    queryOverlap(registry.dynamic_world_bvh, trajectory_aabb, possible_collisions);

    // PROFILE_SYSTEM_BEGIN();

    for (const auto& collision_data : possible_collisions)
    {
        Entity candidate = collision_data.entity;

        if (candidate == registry.player_entity) continue;
        if (COMPONENT(candidate, Type).type != EntityType::Enemy) continue;

        EnemyInfo& enemy_info  = COMPONENT(candidate, EnemyInfo);
        const AABB& enemy_aabb = COMPONENT(candidate, PhysicsBody).runtime_shape.aabb;

        if (computeFastSegmentsAABBCollision(fast_segments, num_segments, enemy_aabb))
        {
            enemy_info.aim_locked = true;
        }
    }

    // PROFILE_SYSTEM_END();
}



void shootingSystem(Registry& registry, InputManager& input_manager, Entity shooter) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    // VALIDATE AIMING DISTANCE

    HAS_ASSERT(shooter, PhysicsBody);
    const PhysicsBody& shooter_phys_body = GET_COMPONENT(shooter, PhysicsBody);

    const Real3 shooter_position = shooter_phys_body.position;
    const CrosshairInfo& aim     = registry.crosshair;

    Real3 x0     = Real3(shooter_position.x, shooter_position.y, shooter_position.z + aim.z_offset);
    Real3 target = aim.position;

    Real3 diff     = target - x0;
    Real  distance = glm::length(diff);

    if (distance < MIN_SHOOTING_DISTANCE) 
    {
        registry.trajectory.valid = false;
        PROFILE_SYSTEM_END();
        return;
    }

    if (distance > MAX_SHOOTING_DISTANCE)
    {
        target = x0 + glm::normalize(diff) * MAX_SHOOTING_DISTANCE;
    }

    // CALCULATE TRAJECTORY

    static auto get_ejection_velocity = [&](ProjectileEjectionMode eject_mode) -> Real 
    {
        switch(eject_mode)
        {
            case ProjectileEjectionMode::Shooting: return registry.shooting_speed;
            case ProjectileEjectionMode::Throwing: return registry.throwing_speed;
            default: ASSERT(false, "Cant find projectile speed, invalid projectile type");
        }
    };

    const auto& trait = getTrait(registry.equipped_projectile);

    const Real v0 = get_ejection_velocity(trait.ejection_mode);
    const Real g  = GRAVITY_ACCELERATION;

    TrajectoryResult traj_result = solveTrajectory(trait, x0, target, v0, g);

    if (traj_result.valid == false)
    {
        registry.trajectory.valid = false;
        PROFILE_SYSTEM_END();
        return;
    }

    registry.trajectory = {
            .valid     = true,
            .x0        = x0,
            .direction = traj_result.direction,
            .v0        = v0,
            .te        = traj_result.te,
            .g         = g
        };

    updateAimLocking(registry, trait, traj_result, x0, v0, g);

    PROFILE_SYSTEM_END();
}


void projectileOrientationSystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();
    
    MAP(ProjectileInfo).forEach([&](Entity entity, ProjectileInfo& proj_info)
    {
        HAS_ASSERT(entity, Type);
        ASSERT(COMPONENT(entity, Type).type == EntityType::Projectile, "entity with projectile physics body must be of type projectile");

        ProjectileOrientationMode orientation_mode = projectile_type_traits[toIndex(proj_info.type)].orientation_mode;

        switch(orientation_mode)
        {
            case ProjectileOrientationMode::AlignWithVelocity:
            {
                if (glm::length(proj_info.velocity) < 1e-3f) return;

                Real3 dir = glm::normalize(proj_info.velocity);
                Real3 xy_dir(dir.x, dir.y, 0.0);

                dir    = glm::normalize(dir);
                xy_dir = glm::normalize(xy_dir);

                float angle = acos(glm::dot(dir, xy_dir));
                if (dir.z < 0.0) angle = -angle;

                float pitch = angle;
                float yaw   = atan2(dir.y, dir.x) - glm::radians(90.0);
                float roll  = 0.0f;

                proj_info.orientation = Real3(pitch, yaw, roll);
                break;
            }
            case ProjectileOrientationMode::Spinning:
            {
                // TODO: potion orientation system: implement angular velocity integration
                break;
            }
            default:
            {
                ASSERT(false, "Orientation system for this projectile type not implemented");
            }
        }
    }
    );

    PROFILE_SYSTEM_END();
}

void pathFollowingSystem(Registry& registry) {
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    // TODO: implement path following system

    // for (auto [entity, path] : registry.m_PathFollower) {

    //     HAS_ASSERT(entity, Position);
    //     HAS_ASSERT(entity, Velocity);
    //     ASSERT(!path.points.empty(), "PathFollower component cant have empty points list");

    //     Position& pos = GET_COMPONENT(entity, Position); 
    //     Velocity& vel = GET_COMPONENT(entity, Velocity); 

    //     glm::vec3 target = path.points[path.current_index];
    //     glm::vec3 dir = target - pos.pos;
    //     float dist = glm::length(dir);

    //     if (dist < path.threshold) {
    //         path.current_index++;
    //         if (path.current_index >= path.points.size()) {
    //             if (path.loop)
    //                 path.current_index = 0;
    //             else
    //                 path.current_index = path.points.size() - 1;
    //         }
    //     }

    //     if (dist > 1e-3f) {
    //         glm::vec3 moveDir = glm::normalize(dir);
    //         vel.vel = moveDir * path.speed;
    //     } 
    //     else {
    //         vel.vel = glm::vec3(0.0);
    //     }
    // }  

    PROFILE_SYSTEM_END();
}


void attachmentSystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    for (auto [entity, attach] : registry.m_Attachment) 
    {    
        HAS_ASSERT(entity, SimplePosition);
        HAS_ASSERT(attach.target, PhysicsBody);

        SimplePosition& position            = GET_COMPONENT(entity, SimplePosition);
        const PhysicsBody& target_phys_body = GET_COMPONENT(attach.target, PhysicsBody);

        position.position    = target_phys_body.position + attach.position_attach;
        // TODO: handle orientation attachment
        // position.orientation = target_phys_body.orientation + attach.attach_euler;
    }

    PROFILE_SYSTEM_END();
}



// =======================================================
//  EFFECT SYSTEM
// =======================================================

// TODO: effects application: Damage effects processed after physics, Movement effects processed before physics

// NOTE-IMPORTANT: effectSystem must run AFTER ai choose velocity and player input system 

void damage(Registry& registry, Entity entity, Real quantity) 
{
    ASSERT(quantity>=0.0, "damage quantity must be positive value");

    HAS_ASSERT(entity, Status);
    Status& status = COMPONENT(entity, Status);

    status.life -= quantity;
}

void heal(Registry& registry, Entity entity, Real quantity)   
{
    ASSERT(quantity>=0.0, "heal quantity must be positive value");

    HAS_ASSERT(entity, Status);
    Status& status = COMPONENT(entity, Status);

    status.life += quantity;
    if (status.life >= status.max_life)
        status.life = status.max_life;
}

void scaleMovementIntent(Registry& registry, Entity entity, Real ratio)
{
    ASSERT(ratio >= 0.0, "speed ratio must be positive value");

    HAS_ASSERT(entity, PhysicsBody);
    PhysicsBody& phys_body = COMPONENT(entity, PhysicsBody);

    phys_body.move_intent.x *= ratio;
    phys_body.move_intent.y *= ratio;
}

void applyBurn(Registry&   registry, Entity entity, Effect& effect) { damage(registry, entity, effect.strength); }
void applyPoison(Registry& registry, Entity entity, Effect& effect) { damage(registry, entity, effect.strength); }
void applyHeal(Registry&   registry, Entity entity, Effect& effect) { heal(registry, entity, effect.strength);   }
void applyFreeze(Registry& registry, Entity entity, Effect& effect) { scaleMovementIntent(registry, entity, effect.strength); }
void applySlow(Registry&   registry, Entity entity, Effect& effect) { scaleMovementIntent(registry, entity, effect.strength); }

// TODO: valid entity: function that says if entity is currently valid in registry, useful for assertions
// TODO: confused state: where you go in the opposite direction, perhaps even for shooting

void effectSystem(Registry& registry, Real dt)
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    ASSERT(dt > 0.0f, "dt must be positive");

    MAP(Status).forEach([&](Entity entity, Status& status)
    {
        for (size_t i = 0; i < EffectTypeCount; i++)
        {
            Effect& effect = status.effects[i];

            if (!effect.active) continue;

            ASSERT(EFFECT_DEFS[i].apply   != nullptr,        "EffectDef.apply must not be null");
            ASSERT(EFFECT_DEFS[i].on_tick == effect.on_tick, "Effect.on_tick mismatch with EffectDef");
            ASSERT(effect.duration > 0.0f,                   "Active effect must have positive duration");
            ASSERT(effect.timer <= effect.duration + dt,     "Effect timer overflow (effect not deactivated on time)");

            effect.timer += dt;

            if (effect.on_tick)
            {
                ASSERT(effect.tick_length > 0.0f, "Tick effect must have positive tick_length");
                ASSERT(effect.tick_length > dt,   "DEBUG ONLY: assumes fixed timestep");
                ASSERT(effect.tick_timer >= 0.0f, "Tick timer must not be negative");

                effect.tick_timer += dt;
                if (effect.tick_timer >= effect.tick_length)
                {
                    EFFECT_DEFS[i].apply(registry, entity, effect);
                    effect.tick_timer -= effect.tick_length;
                }
            }
            else
            {
                EFFECT_DEFS[i].apply(registry, entity, effect);
            }

            if (effect.timer >= effect.duration) effect.active = false;
        }
    }
    );

    PROFILE_SYSTEM_END();
}

// =======================================================
//  END EFFECT SYSTEM
// =======================================================


// =======================================================
//  PHYSICS SYSTEM
// =======================================================

#define MAX_VELOCITY     20.0f
#define VELOCITY_DAMPING  0.9f

void physicsSystem(Registry& registry, Real delta_time) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    Real3 gravity(0.0f, 0.0f, -GRAVITY_ACCELERATION);

    MAP(ProjectileInfo).forEach([&](auto entity, auto& proj_info)
    {
        proj_info.time_passed += delta_time;

        Real t   = proj_info.time_passed;
        Real3 x0 = proj_info.init_position;
        Real3 v0 = proj_info.init_velocity;
        Real3 a  = Real3(0.0f, 0.0f, -GRAVITY_ACCELERATION);

        ProjectileTrajectoryType traj_type = getTrait(proj_info.type).trajectory_type;

        if (traj_type == ProjectileTrajectoryType::Straight2D ||
            traj_type == ProjectileTrajectoryType::Straight3D)
        {
            a = Real3(0.0f);
        }

        Real3 xt = x0 + v0 * t + 0.5f * a * (t * t);
        Real3 vt = v0 + a * t;

        proj_info.prev_position = proj_info.position;
        proj_info.position      = xt;
        proj_info.velocity      = vt;
    }
    );

    for (auto [entity, phys_body] : registry.m_PhysicsBody) 
    {
        if (phys_body.physicsType == PhysicsBodyType::noPhysics) continue;

        if (phys_body.objectType == CollisionObjectType::dynamic)
        {
            phys_body.prev_position = phys_body.position;

            phys_body.velocity += gravity * delta_time;

            Real3 movement_velocity = phys_body.move_intent * phys_body.move_speed;

            Real3 tot_velocity = (movement_velocity + phys_body.velocity);

            if (glm::length(tot_velocity) > MAX_VELOCITY)
            {
                WARNING(true, "max velocity shouldn't be reached");

                HAS_ASSERT(entity, Type);
                std::cout << "type:              " << toString(COMPONENT(entity, Type).type) << "\n";
                std::cout << "movement_velocity: " << movement_velocity.x << ", " << movement_velocity.y << ", " << movement_velocity.z << "\n";
                std::cout << "tot_velocity:      " << tot_velocity.x << ", " << tot_velocity.y << ", " << tot_velocity.z << "\n";
                std::cout << "velocity length:   " << glm::length(tot_velocity) << "\n\n";

                tot_velocity = glm::normalize(tot_velocity) * MAX_VELOCITY;
            }

            phys_body.position += tot_velocity * delta_time; 

            // velocity damping
            phys_body.velocity.x *= VELOCITY_DAMPING; 
            phys_body.velocity.y *= VELOCITY_DAMPING;
        }
        else
        {
            ASSERT(false, "Should never end up here");
        }
    }

    PROFILE_SYSTEM_END();
}


void solveConstraintsSystem(Registry& registry, Real dt)
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    POOL(Constraint).forEach(
    [&](PoolIndex idx, Constraint& constraint)
    {
        ASSERT(constraint.is_static_point, "only static constraint implemented");

        Entity entity = constraint.entity1;

        HAS_ASSERT(entity, PhysicsBody);
        PhysicsBody& phys_body = COMPONENT(entity, PhysicsBody);

        constraint.life_time -= dt;
        if (constraint.life_time < 0.0f) 
        { 
            POOL(Constraint).setComponentToRemove(idx);
            return;
        }

        Real3 dir = phys_body.position - constraint.anchor;
        Real dist = glm::length(dir);

        if (dist < 0.0001f) return;

        bool violated = (constraint.type == ConstraintType::Distance) ||
                        (constraint.type == ConstraintType::MaxDistance && dist > constraint.rest_length) ||
                        (constraint.type == ConstraintType::MinDistance && dist < constraint.rest_length);

        if (violated) 
        {
            Real3 normal = dir / dist;
            Real error   = dist - constraint.rest_length;

            phys_body.position -= normal * error * constraint.stiffness;

            phys_body.velocity -= glm::dot(phys_body.velocity, normal) * normal;
        }

        // if (constraint.destroy_on_closeness &&  glm::length(phys_body.position - constraint.anchor) < Constraint::arrivalThreshold)
        // {
        //     POOL(Constraint).setComponentToRemove(idx);
        //     return;
        // }
    }
    );

    POOL(Constraint).flushRemovals();

    PROFILE_SYSTEM_END();
}

// =======================================================
//  END PHYSICS SYSTEM
// =======================================================

// TODO: dynamic aabb tree leaf update i might want to be passed the index as reference so i can change it in the function directly
void collisionShapeUpdateSystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    for (auto [entity, phys_body] : registry.m_PhysicsBody) 
    {
        Real3 pos      = phys_body.position;
        Real3 prev_pos = phys_body.prev_position;

        updateCollisionShape(phys_body.runtime_shape, phys_body.rest_shape, pos);
        
        if (phys_body.objectType != CollisionObjectType::dynamic &&
            phys_body.objectType != CollisionObjectType::staticObject &&
            phys_body.objectType != CollisionObjectType::staticStair)
        {
            ASSERT(false, "Object in dynamic aabb tree with unhandled collision object type");
        }

        ASSERT(registry.dynamic_world_bvh.indexIsValid(phys_body.aabb_tree_idx), "Static or dynamic object in dynamic world must have valid tree index");
        
        AABB runtime_aabb = computeCollisionShapeAABB(phys_body.runtime_shape);

        phys_body.aabb_tree_idx = dynamicAABBTreeUpdateLeaf(registry.dynamic_world_bvh, phys_body.aabb_tree_idx, runtime_aabb);
    }

    MAP(ProjectileInfo).forEach([&](auto entity, auto& proj_info)
    {
        updateSegment(proj_info.runtime_shape.segment, proj_info.prev_position, proj_info.position);
    }
    );

    MAP(InteractableInfo).forEach([&](auto entity, auto& inter_info)
    {
        updateCollisionShape(inter_info.runtime_shape, inter_info.rest_shape, registry.getWorldPosition(entity));
    }
    );

    PROFILE_SYSTEM_END();
}


// =======================================================
//  INTERACTION SYSTEM
// =======================================================

void updatePointedEntities(Registry& registry, QueryOverlapResult<DynamicAABBTreeNodeData>& dyn_overlaps)
{
    // TODO: mouse pointed entities: bring this calculation somewhere else (before the collision system)
    registry.pointed_entities.clear();

    if (registry.valid_mouse_pointer)
    {
        queryOverlap(
            registry.dynamic_world_bvh, 
            registry.pointer_ray, 
            dyn_overlaps
        );

        for (const auto& data : dyn_overlaps) 
        {
            // TODO: mouse pointed entities: further check intersection with real collision shape
            registry.pointed_entities.insert(data.entity);
        }
    }
}


void handlePoolElementDestruction(Registry& registry, Entity inter_entity)
{
    ASSERT(!registry.isEntityMarkedForRemoval(inter_entity), "interacting entity can not alredy set for removal");
    HAS_ASSERT(inter_entity, PoolDestructionInfo);
    PoolDestructionInfo& dest_info = COMPONENT(inter_entity, PoolDestructionInfo);

    registry.setComponentToRemove(dest_info.type_id, dest_info.idx);
    registry.setEntityToRemove(inter_entity);
}

void handleDoorLogic(Registry& registry, Entity inter_entity)
{
    HAS_ASSERT(inter_entity, DoorInfo);
    DoorInfo& door_info = COMPONENT(inter_entity, DoorInfo);

    if (!door_info.locked) door_info.open = true;
}

void handleStatusChangeArea(Registry& registry, Entity entity, Entity inter_entity)
{
    if (!HAS_COMPONENT(entity, Status)) return;

    Status& status = COMPONENT(entity, Status);

    HAS_ASSERT(inter_entity, StatusChangeInfo);
    StatusChangeInfo& status_change_info = COMPONENT(inter_entity, StatusChangeInfo);

    Effect& effect_template = status_change_info.effect;

    if (status.isEffectActive(effect_template.type)) return;
    
    status.activateEffect(effect_template);
}

void handleHouseLayerToggle(Registry& registry, Entity entity, Entity inter_entity, TriggerAction action)
{
    ASSERT_ONE_OF(action, "only enter and exit action should trigger house layer event", TriggerAction::Enter, TriggerAction::Exit);

    HAS_ASSERT(inter_entity, HouseLayer);
    const HouseLayer& house_layer = COMPONENT(inter_entity, HouseLayer);

    HAS_ASSERT(house_layer.house_entity, House);
    House& house = COMPONENT(house_layer.house_entity, House); 

    if (registry.isPlayer(entity))
    {
        ASSERT(house.this_frame_trigger_action == TriggerAction::None, "two trigger action in one frame should not be possible");
        house.this_frame_trigger_action = action;

        HAS_ASSERT(entity, PhysicsBody);
        const PhysicsBody& phys_body = COMPONENT(entity, PhysicsBody);

        ASSERT(phys_body.runtime_shape.type == CollisionShapeType::AABB, "dynamic player entity must have aabb collision type");

        bool is_at_least_one_colliding    = false;
        uint8_t max_collision_floor_level = 0;

        for (int i=0; i<house.trigger_layer_size; i++)
        {
            Entity trigger_layer_entity = house.layer_trigger_entities[i];

            HAS_ASSERT(trigger_layer_entity, HouseLayer);
            HAS_ASSERT(trigger_layer_entity, InteractableInfo);

            const InteractableInfo& layer_inter_info = COMPONENT(trigger_layer_entity, InteractableInfo);
            const HouseLayer& current_layer = COMPONENT(trigger_layer_entity, HouseLayer);

            ASSERT(layer_inter_info.runtime_shape.type == CollisionShapeType::AABB, "only aabb collision shapes are possible for layer triggers");

            // if (max_collision_floor_level > current_layer.floor_level) continue;

            if (AABBOverlap(phys_body.runtime_shape.aabb, layer_inter_info.runtime_shape.aabb))
            {
                is_at_least_one_colliding = true;

                if (current_layer.floor_level > max_collision_floor_level)
                {
                    max_collision_floor_level = current_layer.floor_level;
                }
            }
        }

        for (uint8_t i = 0; i < house.size; i++) 
        {
            if (!is_at_least_one_colliding) 
            {
                house.layers[i].hidden = false;
            } 
            else 
            {
                house.layers[i].hidden = (i > max_collision_floor_level);
            }
        }
    }
    else
    {
        if (action == TriggerAction::Enter)
        {

        }
        else if (action == TriggerAction::Exit)
        {

        }
    }
}

// NOTE: a thing to consider: removal of an interactable during its own application to multiple entities (ex: loot that can be collected by multiple entities)

void interactionHandlingCollectOverlaps(Registry& registry, QueryOverlapResult<DynamicAABBTreeNodeData>& dyn_overlaps)
{
    MAP(InteractableInfo).forEach([&](Entity inter_entity, InteractableInfo& inter_info) 
    {
        ASSERT(!registry.isEntityMarkedForRemoval(inter_entity), "Interacting with entity marked for removal");

        AABB query_aabb = computeCollisionShapeAABB(inter_info.runtime_shape);

        queryOverlap(registry.dynamic_world_bvh, query_aabb, dyn_overlaps);

        for (const auto& data : dyn_overlaps) 
        {
            if (inter_entity == data.entity) continue;

            if (inter_info.only_player && !registry.isPlayer(data.entity)) continue;

            Entity entity = data.entity;

            HAS_ASSERT(entity, PhysicsBody);
            PhysicsBody& phys_body = COMPONENT(entity, PhysicsBody);
            
            if (phys_body.objectType != CollisionObjectType::dynamic) continue;

            AABB dyn_aabb = computeCollisionShapeAABB(phys_body.runtime_shape);

            bool is_overlapping = testAABBShapeCollision(dyn_aabb, inter_info.runtime_shape);

            if (!is_overlapping) continue;

            inter_info.overlap_count++;
            
            if (inter_info.only_player && registry.isPlayer(entity))
            {
                inter_info.player_is_inside = true;
            }
            else if (!inter_info.affects_actor)
            {
                ASSERT(inter_info.trigger_mode == TriggerMode::OnAreaStay, "Interactable that doesnt affect actors can only have continously triggered events");
            }
            else
            {
                HAS_ASSERT(inter_entity, InteractionHistory);
                InteractionHistory& inter_history = COMPONENT(inter_entity, InteractionHistory);

                inter_history.addOverlap(entity);
            }
        }
    }
    );
}

void interactionHandlingCollectHovered(Registry& registry)
{
    MAP(InteractableInfo).forEach([&](Entity inter_entity, InteractableInfo& inter_info)
    {
        if (registry.isPointed(inter_entity)) inter_info.hovered = true;
    });
}

void interactionHandlingResolve(Registry& registry)
{
    MAP(InteractableInfo).forEach([&](Entity inter_entity, InteractableInfo& inter_info)
    {
        ASSERT(!(inter_info.affects_actor && inter_info.only_player), "if interactable acts only on player shouldnt have affects actor flag set");
        
        // GENERAL AFFECTOR INTERACTABLES
        if (inter_info.affects_actor)
        {
            HAS_ASSERT(inter_entity, InteractionHistory);
            InteractionHistory& inter_history = COMPONENT(inter_entity, InteractionHistory);

            static auto get_affected_entities = [](TriggerMode trigger_mode, const InteractionHistory& inter_history) -> auto 
            {
                switch(trigger_mode)
                {
                    case TriggerMode::OnAreaEnter:        return Difference(inter_history.current,  inter_history.previous);
                    case TriggerMode::OnAreaExit:         return Difference(inter_history.previous, inter_history.current);
                    case TriggerMode::OnAreaFirstEnter:   return Difference(inter_history.current,  inter_history.historical);
                    case TriggerMode::OnAreaStay:         return inter_history.current;
                    case TriggerMode::OnAreaEnterAndExit: return SymmetricDifference(inter_history.current, inter_history.previous);
                    default:
                    {
                        ASSERT(false, "Trigger mode not yet implemented");
                    }
                }
            };
        
            static auto get_immediate_action = [](const InteractionHistory& inter_history, Entity entity) -> TriggerAction
            {
                bool is_current   = inter_history.current.contains(entity);
                bool was_previous = inter_history.previous.contains(entity);

                if (is_current  && !was_previous) return TriggerAction::Enter;
                if (!is_current && was_previous)  return TriggerAction::Exit;
                if (is_current  && was_previous)  return TriggerAction::Stay;
                
                return TriggerAction::None;
            };

            auto affected_entities = get_affected_entities(inter_info.trigger_mode, inter_history);

            for (Entity entity : affected_entities)
            {
                ASSERT(!registry.isEntityMarkedForRemoval(entity), "Should not affect an entity marked for removal");
                ASSERT(!registry.isEntityMarkedForRemoval(inter_entity), "Interactable entity marked for removal should not applying it effect");

                switch(inter_info.interaction_type) 
                {
                    case InteractionType::StatusChange:
                    {
                        handleStatusChangeArea(registry, entity, inter_entity);
                        break;
                    }
                    case InteractionType::HouseLayerToggle:
                    {
                        handleHouseLayerToggle(registry, entity, inter_entity, get_immediate_action(inter_history, entity));
                        break;
                    }
                    default:
                    {
                        ASSERT(false, "Impossible interaction type");
                    }
                } 

                // TODO: interaction handling: here should check in interactable has been destroyed
            }

            return;
        } 
        
        // ONLY PLAYER INTERACTABLES
        if (inter_info.only_player)
        {

            auto is_only_player_interactable_triggered = [](const InteractableInfo& inter_info) -> auto 
            {
                switch(inter_info.trigger_mode)
                {
                    case TriggerMode::OnAreaEnter:        return inter_info.player_is_inside  && !inter_info.player_was_inside;
                    case TriggerMode::OnAreaExit:         return !inter_info.player_is_inside && inter_info.player_was_inside;
                    case TriggerMode::OnAreaFirstEnter:   return inter_info.player_is_inside && !inter_info.player_has_entered;
                    case TriggerMode::OnAreaStay:         return inter_info.player_is_inside;
                    case TriggerMode::OnAreaEnterAndExit: return inter_info.player_is_inside != inter_info.player_was_inside;
                    default:
                    {
                        ASSERT(false, "Trigger mode not yet implemented");
                        return false;
                    }
                }
            };

            static auto get_player_immediate_action = [](const InteractableInfo& inter_info) -> TriggerAction
            {
                if (inter_info.player_is_inside  && !inter_info.player_was_inside) return TriggerAction::Enter;
                if (!inter_info.player_is_inside &&  inter_info.player_was_inside) return TriggerAction::Exit;
                if (inter_info.player_is_inside  &&  inter_info.player_was_inside) return TriggerAction::Stay;
                
                return TriggerAction::None;
            };

            if (!is_only_player_interactable_triggered(inter_info)) return;

            ASSERT(!registry.isEntityMarkedForRemoval(inter_entity), "Interactable entity marked for removal should not applying it effect");

            Entity player_entity = registry.player_entity;

            switch(inter_info.interaction_type) 
            {
                case InteractionType::Door:
                {
                    handleDoorLogic(registry, inter_entity);
                    break;
                }
                case InteractionType::StatusChange:
                {
                    handleStatusChangeArea(registry, player_entity, inter_entity);
                    break;
                }
                case InteractionType::HouseLayerToggle:
                {
                    handleHouseLayerToggle(registry, player_entity, inter_entity, get_player_immediate_action(inter_info));
                    break;
                }
                case InteractionType::PoolElementDestruction:
                {
                    handlePoolElementDestruction(registry, inter_entity);
                    break;
                }
                default:
                {
                    ASSERT(false, "Impossible interaction type");
                }
            } 

            return;
        }
        
        // ANONYMOUSE INTERACTABLES
        if (!inter_info.affects_actor && !inter_info.only_player)
        {
            ASSERT(inter_info.trigger_mode == TriggerMode::OnAreaStay, "Interactable that doesnt affect actors can only have continously triggered events");

            bool is_triggered = inter_info.overlap_count > 0;

            if (!is_triggered) return;

            ASSERT(!registry.isEntityMarkedForRemoval(inter_entity), "Interactable entity marked for removal should not applying it effect");

            switch(inter_info.interaction_type) 
            {
                case InteractionType::Door:
                {
                    handleDoorLogic(registry, inter_entity);
                    break;
                }
                default:
                {
                    ASSERT(false, "Interaction handling for type not supported");
                }
            } 

            return;
        }

        ASSERT(false, "Should never end up here");
    });
}

void interactionSystem(Registry& registry)
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    QueryOverlapResult<DynamicAABBTreeNodeData> dyn_overlaps;

    interactionHandlingCollectOverlaps(registry, dyn_overlaps);

    updatePointedEntities(registry, dyn_overlaps);
    interactionHandlingCollectHovered(registry);

    interactionHandlingResolve(registry);

    PROFILE_SYSTEM_END();
}

// =======================================================
//  END INTERACTION SYSTEM
// =======================================================

// DYNAMIC COLLISION HANDLING FUNCTIONs

Real AARampGetElevationAt(const AARamp& aaramp, const Real2& xy)
{
    Real3 diff = aaramp.maxv - aaramp.minv;

    ASSERT(diff.x > 0.0, "Stair x length must be positive");
    ASSERT(diff.y > 0.0, "Stair y length must be positive");

    Real2 tv = (xy - Real2(aaramp.minv)) / Real2(diff);

    float t;
    switch(aaramp.dir) 
    {
        case AARampDirection::PosX: t = tv.x;       break;
        case AARampDirection::PosY: t = tv.y;       break;
        case AARampDirection::NegX: t = 1.0 - tv.x; break;
        case AARampDirection::NegY: t = 1.0 - tv.y; break;
        default: ASSERT(false, "Stair increase type must be set correctly");
    }

    return aaramp.minv.z + diff.z * glm::clamp(t, 0.0f, 1.0f);
};

Real AA4PyramidGetElevationAt(const AA4SidedPyramid& aa4pyr, const Real2& xy)
{
    ASSERT(aa4pyr.axis == Axis::Z, "Elevation static object must be Z aligned");
    ASSERT(aa4pyr.height >= 0.0f,  "Elevation static object must have positive height");

    Real2 tv = 1.0f - glm::abs(xy - Real2(aa4pyr.base_center) / aa4pyr.half_extents);

    Real t = std::min(tv.x, tv.y);

    return aa4pyr.base_center.z + aa4pyr.height * glm::clamp(t, 0.0f, 1.0f);
};

template <ValidStaticBody T>
void handleDynamicStaticStairCollision(const T& stair_body, PhysicsBody &dyn_body) 
{
    TRACE_SYSTEM();

    // ASSERT(stair_body.runtime_shape.type == CollisionShapeType::AARamp, "Stair must have AARamp collision shape");

    AABB dyn_aabb = computeCollisionShapeAABB(dyn_body.runtime_shape);

    AABB stair_aabb  = computeCollisionShapeAABB(stair_body.runtime_shape);
    auto coll_result = computeAABBAABBCollision(dyn_aabb, stair_aabb);

    if (!coll_result.is_colliding) return;

    Real3 center = AABBCenter(dyn_aabb);
    Real2 proj(center.x, center.y);

    Real height = stair_aabb.maxv.z - stair_aabb.minv.z;

    ASSERT(height > 0.0, "Stair z elevation must be positive");

    Real stair_height;
    
    if (stair_body.runtime_shape.type == CollisionShapeType::AARamp)
        stair_height = AARampGetElevationAt(stair_body.runtime_shape.aaramp, proj) + 0.1f;
    else if (stair_body.runtime_shape.type == CollisionShapeType::AA4SidedPyramid)
        stair_height = AA4PyramidGetElevationAt(stair_body.runtime_shape.aa4pyramid, proj) + 0.1f;
    else
        ASSERT(false, "unknown elevation function");

    Real dyn_feet_height = dyn_body.position.z;

    Real feet_elevation_distance = stair_height - dyn_feet_height;
    Real upward_displacement_threshold = height*0.2;

    if (stair_height >= dyn_feet_height)
    {
        if (coll_result.mtv.z < 0.0f)
        {
            dyn_body.position               += coll_result.mtv;
            dyn_body.accumulated_correction += coll_result.mtv;

            dyn_body.has_collided = true;
        }
        else if (feet_elevation_distance < upward_displacement_threshold || (feet_elevation_distance > upward_displacement_threshold && coll_result.mtv.z > 1e-3)) 
        {    
            Real dyn_feet_new_height = glm::min(stair_height, stair_aabb.maxv.z);

            Real3 mtv = Real3(0.0f);
            mtv.z     = dyn_feet_new_height - dyn_feet_height;

            dyn_body.position               += mtv;
            dyn_body.accumulated_correction += mtv;

            dyn_body.has_collided = true;
        }
        else
        {
            ASSERT(abs(coll_result.mtv.z) < 1e-3, "Z coorection here must always be zero");

            dyn_body.position               += coll_result.mtv;
            dyn_body.accumulated_correction += coll_result.mtv;
            
            dyn_body.has_collided = true;
        }
    }
}

template <ValidStaticBody T>
void handleDynamicStaticObjectCollision(const T& static_body, PhysicsBody &dyn_body)
{
    CollisionResult coll_result;

    if (dyn_body.runtime_shape.type == CollisionShapeType::AACapsule)
    {
        coll_result = computeAACapsuleShapeCollision(dyn_body.runtime_shape.aacapsule, static_body.runtime_shape);
    }
    else if (dyn_body.runtime_shape.type == CollisionShapeType::AABB)
    {
        coll_result = computeAABBShapeCollision(dyn_body.runtime_shape.aabb, static_body.runtime_shape);
    }

    if (coll_result.is_colliding) 
    {
        Real3 mtv = coll_result.mtv;

        Real mtv_len = glm::length(mtv);
        
        if (mtv_len > 1e-6f) 
        {
            Real3 mtv_dir = mtv / mtv_len;
            Real dot_up   = mtv_dir.z;

            static constexpr Real vertical_threshold = 0.707f; //0.866f; 

            if (dot_up > vertical_threshold) 
            {
                Real vertical_push = mtv_len / dot_up;
                mtv = Real3(0.0f, 0.0f, mtv.z);
            }
        }

        dyn_body.position               += mtv;
        dyn_body.accumulated_correction += mtv;
        dyn_body.has_collided = true;
    }
}

void handleDynamicDynamicCollision(PhysicsBody& dyn1_body, PhysicsBody& dyn2_body)
{
    ASSERT(dyn1_body.runtime_shape.type == CollisionShapeType::AABB, "Dynamic object must have AABB collision shape");
    ASSERT(dyn2_body.runtime_shape.type == CollisionShapeType::AABB, "Dynamic object must have AABB collision shape");

    CollisionResult coll_result = 
        computeAABBAABBCollision(
            dyn1_body.runtime_shape.aabb, 
            dyn2_body.runtime_shape.aabb);

    if (coll_result.is_colliding) 
    {
        dyn1_body.position += coll_result.mtv * 0.5f;
        dyn2_body.position -= coll_result.mtv * 0.5f;

        dyn1_body.accumulated_correction += coll_result.mtv * 0.5f;
        dyn2_body.accumulated_correction -= coll_result.mtv * 0.5f;

        dyn1_body.has_collided = true;
        dyn2_body.has_collided = true;
    }
}

void dynamicCollisionHandling(Registry &registry, QueryOverlapResult<StaticAABBTreeNodeData>& stat_overlaps, QueryOverlapResult<DynamicAABBTreeNodeData>& dyn_overlaps)
{
    // =======================================================
    //  DYNAMIC OBJECTS COLLISION RESOLUTION
    // =======================================================

    // TODO: resetting physics body has_collided: i dont like it here, it is a RESET functionality
    MAP(PhysicsBody).forEach(
    [](Entity entity, PhysicsBody& phys_body) 
    { 
        phys_body.accumulated_correction = Real3(0.0f); 
        phys_body.has_collided           = false; 
    });

    MAP(PhysicsBody).forEach([&](Entity entity, PhysicsBody& phys_body) 
    {
        if (phys_body.objectType == CollisionObjectType::projectile || 
            phys_body.objectType == CollisionObjectType::staticObject ||
            phys_body.objectType == CollisionObjectType::staticStair) 
            return;

        ASSERT(phys_body.objectType == CollisionObjectType::dynamic, "Unknown physics body object type in collision detection within world moving objects");
        // ASSERT(phys_body.runtime_shape.type == CollisionShapeType::AABB, "Dynamic object must have AABB collision shape");

        AABB query_aabb = computeCollisionShapeAABB(phys_body.runtime_shape);

        // STATIC WORLD CHECKING

        queryOverlap(registry.static_world_bvh, query_aabb, stat_overlaps);

        for (const auto& data : stat_overlaps) 
        {
            const StaticPhysicsBody& static_body = registry.static_world[data.object_idx];

            if (static_body.objectType == CollisionObjectType::staticObject)
            {
                handleDynamicStaticObjectCollision(static_body, phys_body);
            }
            else if (static_body.objectType == CollisionObjectType::staticStair)
            {
                handleDynamicStaticStairCollision(static_body, phys_body);
            }
            else
            {
                ASSERT(false, "impossible static object type");
            }   
        }

        // DYNAMIC WORLD CHECKING

        queryOverlap(registry.dynamic_world_bvh, query_aabb, dyn_overlaps);

        for (const auto& data : dyn_overlaps) 
        {
            Entity other_entity = data.entity;

            if (entity == other_entity) continue;

            HAS_ASSERT(other_entity, PhysicsBody);
            PhysicsBody& other_body = GET_COMPONENT(other_entity, PhysicsBody);

            if (other_body.objectType == CollisionObjectType::dynamic) 
            {
                if (other_entity >= entity) continue;

                handleDynamicDynamicCollision(phys_body, other_body);
            }
            else if (other_body.objectType == CollisionObjectType::staticObject) 
            {
                handleDynamicStaticObjectCollision(other_body, phys_body);
            }
            else if (other_body.objectType == CollisionObjectType::staticStair)
            {
                handleDynamicStaticStairCollision(other_body, phys_body);
            }
            else
            {
                ASSERT(false, "impossible static object type");
            }   
        }
    }
    );

    registry.removeDeathEntities();

    // =======================================================
    //  END DYNAMIC OBJECTS COLLISION RESOLUTION
    // =======================================================
}


// PROJECTILE COLLISION HANDLING FUNCTIONs

struct ProjectileImpact 
{
    Entity proj_entity;
    const ProjectileInfo& proj_info;
    Real3 impact_point;
    bool is_static;
    Entity target_entity;
    const PhysicsBody* target_phys_body;
};

void handleProjectileCollision(Registry& registry, const ProjectileImpact& impact_info)
{
    ASSERT(!HAS_COMPONENT(impact_info.proj_entity, InteractableInfo), "Can not handle projectiles with interactable attached");

    registry.setEntityToRemove(impact_info.proj_entity);

    switch(impact_info.proj_info.type)
    {
        case ProjectileType::Arrow:
        {
            HAS_ASSERT(impact_info.proj_entity, RenderInfo);
            RenderInfo& render_info = COMPONENT(impact_info.proj_entity, RenderInfo);

            if (impact_info.is_static)
            {
                assembleStaticIdleProjectile(
                    registry,
                    render_info.mesh,
                    impact_info.impact_point,
                    impact_info.proj_info.orientation);
            }
            else
            {
                assembleAttachedIdleProjectile(
                    registry,
                    render_info.mesh,
                    impact_info.impact_point - impact_info.target_phys_body->position,
                    impact_info.proj_info.orientation, 
                    impact_info.target_entity);
                
                if (HAS_COMPONENT(impact_info.target_entity, Status)) 
                {
                    Status& status = COMPONENT(impact_info.target_entity, Status);
                    status.life -= 10.0f;
                }
            }

            break;
        }
        case ProjectileType::Potion:
        {
            assembleStatusChangeArea( 
                registry,
                impact_info.impact_point,
                0.5f, 
                2.0f);

            break;
        }
        case ProjectileType::Hook:
        {
            if (!impact_info.is_static) break;

            Entity shooter = impact_info.proj_info.shooter;

            Constraint constraint = {
                .type                 = ConstraintType::Distance,
                .rest_length          = 0.0f,
                .stiffness            = 0.1f, 
                .anchor               = impact_info.impact_point,
                .life_time            = 2.0f,
                .is_static_point      = true, 
                // .destroy_on_closeness = false, 
                .entity1              = shooter
            };

            PoolIndex index    = POOL(Constraint).add(constraint);
            ComponentPoolID id = ComponentPoolID::Constraint;

            assemblePoolElementDestructor(registry, impact_info.impact_point, id, index);

            break;
        }
        default:
        {
            ASSERT(false, "Projectile collision handling not supported for projectile type");
        }
    }
}


template <ValidStaticBody T>
bool handleProjectileStaticCollision(Registry& registry, const T& static_body, Entity proj_entity, const ProjectileInfo& proj_info)
{
    SegmentCollisionResult coll_result;

    if (static_body.objectType == CollisionObjectType::staticObject ||
        static_body.objectType == CollisionObjectType::staticStair) 
    {
        coll_result = computeSegmentShapeCollision(proj_info.runtime_shape.segment, static_body.runtime_shape);
    }
    // else if (static_body.objectType == CollisionObjectType::staticStair) 
    // {
    //     coll_result = computeSegmentAARampCollision(
    //                     proj_info.runtime_shape.segment, 
    //                     static_body.runtime_shape.aaramp);
    // }
    else
    {
        ASSERT(false, "Impossible static object type");
    }

    if (coll_result.is_colliding) 
    {
        ProjectileImpact impact_info =  
        {
            .proj_entity      = proj_entity,
            .proj_info        = proj_info,
            .impact_point     = coll_result.entering_point,
            .is_static        = true,
            .target_entity    = 0,
            .target_phys_body = nullptr
        };

        handleProjectileCollision(registry, impact_info);
    }

    return coll_result.is_colliding;
}

bool handleProjectileDynamicCollision(Registry& registry, Entity dyn_entity, const PhysicsBody& dyn_body, Entity proj_entity, const ProjectileInfo& proj_info)
{
    ASSERT(dyn_body.runtime_shape.type == CollisionShapeType::AABB, "Dynamic object must have AABB collision shape");

    SegmentCollisionResult coll_result = 
        computeSegmentAABBCollision(
            proj_info.runtime_shape.segment, 
            dyn_body.runtime_shape.aabb);

    if (coll_result.is_colliding) 
    {
        ProjectileImpact impact_info =  
        {
            .proj_entity      = proj_entity,
            .proj_info        = proj_info,
            .impact_point     = coll_result.entering_point,
            .is_static        = false,
            .target_entity    = dyn_entity,
            .target_phys_body = &dyn_body
        };

        handleProjectileCollision(registry, impact_info);
    }

    return coll_result.is_colliding;
}

void projectileCollisionHandling(Registry &registry, QueryOverlapResult<StaticAABBTreeNodeData>& stat_overlaps, QueryOverlapResult<DynamicAABBTreeNodeData>& dyn_overlaps)
{
    // =======================================================
    //  PROJECTILE COLLISION DETECTION
    // =======================================================

    MAP(ProjectileInfo).forEach(
    [&](Entity proj_entity, ProjectileInfo& proj_info)
    {
        // ASSERT(proj_body.runtime_shape.type == CollisionShapeType::Segment, "Projectile must use Segment shape for static collision");
        // ASSERT(proj_body.objectType == CollisionObjectType::projectile,     "Projectile must be of CollisionObjectType projectile");

        ASSERT(proj_info.runtime_shape.type == CollisionShapeType::Segment, "Projectile must use Segment shape for static collision");

        AABB query_aabb   = computeSegmentAABB(proj_info.runtime_shape.segment); //getQueryAABB(proj_body);
        bool proj_has_hit = false;
        

        // STATIC WORLD CHECKING

        queryOverlap(registry.static_world_bvh, query_aabb, stat_overlaps);

        for (const auto& data : stat_overlaps) 
        {
            const StaticPhysicsBody& static_body = registry.static_world[data.object_idx];

            proj_has_hit = handleProjectileStaticCollision(registry, static_body, proj_entity, proj_info);

            if (proj_has_hit) break;
        }

        if (proj_has_hit) return;


        // DYNAMIC WORLD CHECKING

        queryOverlap(registry.dynamic_world_bvh, query_aabb, dyn_overlaps);

        for (const auto& data : dyn_overlaps) 
        {
            Entity other_entity = data.entity;

            if (other_entity == proj_info.shooter) continue;

            HAS_ASSERT(other_entity, PhysicsBody);
            PhysicsBody& other_body = GET_COMPONENT(other_entity, PhysicsBody);

            if (other_body.objectType == CollisionObjectType::dynamic)
            {
                proj_has_hit = handleProjectileDynamicCollision(registry, other_entity, other_body, proj_entity, proj_info);
            }
            else if (
                other_body.objectType == CollisionObjectType::staticObject || 
                other_body.objectType == CollisionObjectType::staticStair)
            {
                proj_has_hit = handleProjectileStaticCollision(registry, other_body, proj_entity, proj_info);
            }

            if (proj_has_hit) break;
        }
    }
    );

    // EXPLAIN: projectile could have hit in the static world collision pass, so could be not available anymore -> must destroy
    registry.removeDeathEntities();

    // =======================================================
    //  END PROJECTILE COLLISION DETECTION
    // =======================================================
}



void postCollisionVelocityFixup(Registry& registry)
{
    MAP(PhysicsBody).forEach([&](Entity entity, PhysicsBody& phys_body) 
    {
        if (phys_body.objectType == CollisionObjectType::dynamic && phys_body.has_collided) 
        {
            const float epsilon = 1e-3f;
            
            // 1. LANDING: Pushed up by floor while falling
            if (phys_body.accumulated_correction.z > epsilon && phys_body.velocity.z < 0.0f) 
            {
                phys_body.velocity.z = 0.0f;
            }
            
            // 2. HEAD-BUMP: Pushed down by ceiling while jumping
            else if (phys_body.accumulated_correction.z < -epsilon && phys_body.velocity.z > 0.0f) 
            {
                phys_body.velocity.z = 0.0f;
            }
        }
    }
    );
}

void collisionSystem(Registry& registry, InputManager& input_manager) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    // TODO: reorganize QueryOverlapResult data-structure: add query overlap result data structure into the AABB tree so that in stack exists only one
    QueryOverlapResult<StaticAABBTreeNodeData>  stat_overlaps;
    QueryOverlapResult<DynamicAABBTreeNodeData> dyn_overlaps;

    projectileCollisionHandling(registry, stat_overlaps, dyn_overlaps);

    // updatePointedEntities(registry, dyn_overlaps);

    dynamicCollisionHandling(registry, stat_overlaps, dyn_overlaps);

    postCollisionVelocityFixup(registry);

    PROFILE_SYSTEM_END();
}

void transparentOccludingSystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    Ray ray;
    CameraInfo& camera = registry.camera;

    HAS_ASSERT(registry.player_entity, PhysicsBody);
    const PhysicsBody& target_phys_body = GET_COMPONENT(registry.player_entity, PhysicsBody);

    // ASSERT(target_phys_body.runtime_shape.type == CollisionShapeType::AABB, "Player physics body must have AABB collision shape");

    // TODO: chose better origin point
    ray.origin    = target_phys_body.position + Real3(0.0f, 0.0f, 0.1f); // AABBCenter(target_phys_body.runtime_shape.aabb);
    ray.direction = glm::normalize(camera.position - ray.origin);

    QueryOverlapResult<StaticAABBTreeNodeData> occluding_objects;
    queryOverlap(registry.static_world_bvh, ray, occluding_objects);

    struct Occluder 
    {
        Entity entity;
        Real distance;
    };

    std::vector<Occluder> temp_occluding_entities;
    temp_occluding_entities.reserve(occluding_objects.size());

    registry.transparent_entities.clear();
    registry.transparent_entities.reserve(occluding_objects.size());

    for (auto data : occluding_objects)
    {
        const StaticPhysicsBody& stat_body = registry.static_world[data.object_idx];

        auto coll_result = computeRayShapeCollision(ray, stat_body.runtime_shape);

        if (!coll_result.is_colliding) continue;

        Entity occl_ent = data.entity;

        Real distance = glm::length(coll_result.entering_point - ray.origin);

        HAS_ASSERT(occl_ent, RenderInfo);
        RenderInfo &rend_info = GET_COMPONENT(occl_ent, RenderInfo);

        rend_info.alpha = 0.5f;

        temp_occluding_entities.push_back({occl_ent, distance});
    }

    std::sort(
        temp_occluding_entities.begin(), 
        temp_occluding_entities.end(), 
        [](const Occluder& a, const Occluder& b) { return a.distance < b.distance; }
    );

    for (const auto& item : temp_occluding_entities) 
    {
        registry.transparent_entities.push_back(item.entity);
    }

    PROFILE_SYSTEM_END();
}

void cameraFollowSystem(Registry& registry, float delta_time) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    CameraInfo& camera = registry.camera;

    HAS_ASSERT(camera.target, PhysicsBody);
    const PhysicsBody& target_phys_body = GET_COMPONENT(camera.target, PhysicsBody);

    glm::vec3 currentCameraPos(camera.position);

    glm::vec3 wantedCameraPos(target_phys_body.position + camera.offset);

    glm::vec3 cameraDistVec = wantedCameraPos - currentCameraPos;
    glm::vec3 cameraVelDir  = glm::normalize(cameraDistVec);
    float distance          = glm::length(cameraDistVec);

    glm::vec3 cameraPosVec;

    if (distance < 1e-2) 
    {
        cameraPosVec = wantedCameraPos;
    }
    else
    {
        cameraPosVec = currentCameraPos + cameraVelDir * camera.vel * delta_time;
        if (glm::length(wantedCameraPos - cameraPosVec) > camera.max_distance)
            cameraPosVec = wantedCameraPos - camera.max_distance * cameraVelDir;
    }

    camera.position.x = cameraPosVec.x;
    camera.position.y = cameraPosVec.y;
    camera.position.z = wantedCameraPos.z;

    PROFILE_SYSTEM_END();
}

void cameraSetupSystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    CameraInfo& camera = registry.camera;

    glm::vec3 cameraTarget = camera.position - camera.offset;
    glm::vec3 cameraUp(0.0f, 0.0f, 1.0f);

    camera.view       = glm::lookAt(camera.position, cameraTarget, cameraUp);
    camera.projection = glm::perspective(
        glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f
    );

    registry.shader_manager.setViewProjection(camera.view, camera.projection);

    PROFILE_SYSTEM_END();
}

// =======================================================
//  RENDERING SYSTEM
// =======================================================

void updateAnimationStateSystem(Registry& registry)
{
    for (auto [entity, anim_info] : registry.m_AnimationState)
    {
        if (anim_info.current_state != anim_info.previous_state)
        {
            HAS_ASSERT(entity, RenderInfo);

            RenderInfo& render_info = COMPONENT(entity, RenderInfo);

            AnimStateInfo info = anim_info.state_mesh_map->get(anim_info.current_state);

            render_info.mesh                   = info.mesh;
            render_info.fps                    = info.fps;
            render_info.currframe_played_steps = 0;

            info.mesh->setFrame(0); // TODO: if multiple entities share the same animated mesh i should be able to draw anim frame for each one of them
        }   
        else
        {

        }

        anim_info.previous_state = anim_info.current_state;
    } 
}


void houseHidingSystem(Registry& registry)
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    MAP(House).forEach([&](Entity house_entity, const House& house)
    {
        for (int i=0; i<house.size; i++)
        {
            for (auto entity : house.layers[i].static_entities)
            {
                HAS_ASSERT(entity, RenderInfo);
                RenderInfo& rend_info = COMPONENT(entity, RenderInfo);

                rend_info.visible = !house.layers[i].hidden;
            }

            for (auto object_idx : house.layers[i].static_phys_bodies)
            {
                ASSERT(object_idx < registry.static_world.size(), "invalid static world body object index");

                registry.static_world[object_idx].visible = !house.layers[i].hidden;
            }

            for (auto entity : house.layers[i].dynamic_entities)
            {
                HAS_ASSERT(entity, RenderInfo);
                RenderInfo& rend_info = COMPONENT(entity, RenderInfo);

                rend_info.visible = !house.layers[i].hidden;
            }
        }
    });

    PROFILE_SYSTEM_END();
}


void renderLockedEnemies(Registry& registry, Shader& shader) 
{
    TRACE_SYSTEM();

    glm::mat4 model;
    
    for (const auto& [entity, enemy_info] : registry.m_EnemyInfo) 
    {
        if (!enemy_info.aim_locked) continue;

        HAS_ASSERT(entity, PhysicsBody);
        const PhysicsBody& phys_body = GET_COMPONENT(entity, PhysicsBody);

        model = glm::mat4(1.0f);
        model = glm::translate(model, phys_body.position);
        model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.01f));
        model = glm::scale(model, glm::vec3(0.3f, 0.3f, 1.0f));

        shader.setMat4("model", model);
        shader.setFloat("uR", 1.0f);
        shader.setFloat("uG", 1.0f);
        shader.setFloat("uB", 0.0f);
        
        registry.circle_mesh.drawWireframe(shader);
    }
}

void renderInteractables(Registry& registry, Shader& shader)
{
    TRACE_SYSTEM();

    glm::mat4 model;

    Real3 camera_position = registry.camera.position;

    MAP(InteractableInfo).forEach([&](Entity entity, const InteractableInfo& inter_info)
    {
        if (inter_info.interaction_type == InteractionType::StatusChange)
        {
            ASSERT(inter_info.runtime_shape.type == CollisionShapeType::Sphere, "status infector interactable must have sphere shape");

            const Sphere& sphere = inter_info.runtime_shape.sphere;

            registry.effect_sphere_renderer.draw(
                registry.shader_manager.effect_sphere_shader, 
                sphere.center, sphere.radius, Real3(1.0f, 0.2f, 0.9f), 
                camera_position,
                registry.scene_buffer
            );
        }

        if (inter_info.hovered)
        {
            Real3 position = registry.getWorldPosition(entity);

            model = glm::mat4(1.0f);
            model = glm::translate(model, position + Real3(0.0f, 0.0f, 0.01f));
            model = glm::scale(model, Real3(0.3f, 0.3f, 1.0f));

            shader.setMat4("model", model);
            shader.setFloat("uR", 1.0f);
            shader.setFloat("uG", 1.0f);
            shader.setFloat("uB", 0.0f);
            
            registry.circle_mesh.drawWireframe(shader);
        }
    }
    );
}

void renderStatus(Registry &registry) 
{
    TRACE_SYSTEM();

    glm::mat4 model;

    Shader& particle_shader = registry.shader_manager.particle_shader;
    Shader& default_shader = registry.shader_manager.def_shader;

    CONST_MAP(Status).forEach([&](Entity entity, const Status& status) 
    {
        HAS_ASSERT(entity, PhysicsBody);
        const PhysicsBody& phys_body = GET_COMPONENT(entity, PhysicsBody);

        // HEALTH BAR

        Real life_ratio = status.life / status.max_life;
        life_ratio = glm::clamp(life_ratio, 0.0f, 1.0f);

        if (life_ratio > 0.0f)
        {
            default_shader.use();
            model = glm::mat4(1.0f);
            model = glm::translate(model, phys_body.position + Real3(0.0f, 0.0f, 0.6f));
            model = glm::scale(model, Real3(life_ratio, 1.0f, 1.0f));
            default_shader.setMat4("model", model);
            default_shader.setFloat("uR", 1.0f - life_ratio);
            default_shader.setFloat("uG", life_ratio);
            default_shader.setFloat("uB", 0.0f);
            registry.mesh_manager.status_bar_mesh.draw(default_shader);
        }

        // STATUS EFFECTS

        HAS_ASSERT(entity, ParticleRendererInfo);
        const ParticleSystem &particle_renderer = GET_COMPONENT(entity, ParticleRendererInfo).particle_system;

        const Effect& poisoned = status.effects[toIndex(EffectType::Poisoned)];

        if (poisoned.active)
        {
            particle_shader.use();
            particle_renderer.draw(registry.shader_manager.particle_shader, phys_body.position);
        }
    }
    );
}

void renderSystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);

    Shader& shader = registry.shader_manager.def_shader;

    shader.use();

    glm::mat4 model;

    for (auto [entity, renderInfo] : registry.m_RenderInfo) 
    {
        if (!renderInfo.visible) continue;

        if(renderInfo.mesh->m_type == MeshType::animated)
        {
            renderInfo.currframe_played_steps++;
            uint8_t step_per_frame = 60/renderInfo.fps; //TODO: here i fix the global fps rate
            if (renderInfo.currframe_played_steps >= step_per_frame)
            {
                renderInfo.currframe_played_steps = 0;
                renderInfo.mesh->nextFrame();
            }
        }

        if (registry.isTransparent(entity)) continue;

        Real3 position    = registry.getWorldPosition(entity);
        Real3 orientation = registry.getWorldOrientation(entity);

        model = glm::translate(glm::mat4(1.0f), position);
        model = glm::rotate(model, orientation.y, glm::vec3(0, 0, 1));
        model = glm::rotate(model, orientation.x, glm::vec3(1, 0, 0));
        model = glm::rotate(model, orientation.z, glm::vec3(0, 1, 0));

        shader.setMat4("model", model);
        shader.setFloat("uR", renderInfo.color.r);
        shader.setFloat("uG", renderInfo.color.g);
        shader.setFloat("uB", renderInfo.color.b);
        shader.setFloat("uAlpha", 1.0f);

        renderInfo.mesh->draw(shader);
    }

    for (Entity entity : registry.transparent_entities) 
    {
        HAS_ASSERT(entity, RenderInfo);
        HAS_ASSERT(entity, SimplePosition);

        RenderInfo &rend_info    = GET_COMPONENT(entity, RenderInfo);
        SimplePosition &simp_pos = GET_COMPONENT(entity, SimplePosition);

        if (!rend_info.visible) continue;

        model = glm::translate(glm::mat4(1.0f), simp_pos.position);

        shader.setMat4("model",   model);
        shader.setFloat("uR",     rend_info.color.r);
        shader.setFloat("uG",     rend_info.color.g);
        shader.setFloat("uB",     rend_info.color.b);
        shader.setFloat("uAlpha", rend_info.alpha);

        rend_info.mesh->draw(shader);
    }

    shader.setFloat("uAlpha", 1.0f);

    // registry.debug_depth_renderer.draw(registry.scene_buffer);

    renderLockedEnemies(registry, shader);

    renderInteractables(registry, shader);

    renderStatus(registry);

    // FOG

    HAS_ASSERT(registry.player_entity, PhysicsBody);
    Real3 player_pos = COMPONENT(registry.player_entity, PhysicsBody).position;

    registry.fog_renderer.draw
    (
        registry.shader_manager.fog_shader,
        2, 
        Real3(6.0f,  2.0f, 0.01f),
        Real3(5.0f,  5.0f, 0.51f),
        player_pos, 
        registry.scene_buffer
    );

    
    // DIG. GRID

    registry.digital_grid_renderer.draw
    (
        registry.shader_manager.digital_grid_shader, 
        Real3(6.0f,  -4.0f, 0.01f),
        Real3(3.0f,   3.0f,  3.0f)
    );

    // SNOW

    CameraInfo& camera = registry.camera;
    registry.snow_renderer.draw
    (
        registry.shader_manager.snow_shader, 
        camera.position
    );

    PROFILE_SYSTEM_END();
}


void renderCollisionShape(Registry& registry, const CollisionShape& coll_shape)
{
    Shader& wf_shader = registry.shader_manager.wireframe_shader;

    glm::mat4 model(1.0f);

    switch(coll_shape.type)
    {
        case CollisionShapeType::AABB:   [[fallthrough]];
        case CollisionShapeType::AARamp: [[fallthrough]];
        case CollisionShapeType::Segment:
        {
            AABB aabb = computeCollisionShapeAABB(coll_shape);

            Real3 size = aabb.maxv - aabb.minv;
            model = glm::translate(model, aabb.minv);
            model = glm::scale(model, size);
            wf_shader.setMat4("model", model);
            registry.mesh_manager.aabb_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::AACylinder:
        {
            Real3 st  = coll_shape.aacylinder.base_center;
            Real r    = coll_shape.aacylinder.radius;
            Real h    = coll_shape.aacylinder.height;
            Axis axis = coll_shape.aacylinder.axis;
            int axis_idx = (int)axis;

            model = glm::translate(model, st);

            if      (axis == Axis::X) model = glm::rotate(model, glm::radians(90.0f),  glm::vec3(0.0f, 1.0f, 0.0f));
            else if (axis == Axis::Y) model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

            model = glm::scale(model, glm::vec3(r, r, h));

            wf_shader.setMat4("model", model);

            registry.mesh_manager.zaacylinder_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::Sphere:
        {
            Real3 c = coll_shape.sphere.center;
            Real r  = coll_shape.sphere.radius;

            model = glm::translate(model, c);
            model = glm::scale(model, glm::vec3(r));

            wf_shader.setMat4("model", model);

            registry.mesh_manager.sphere_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::AACapsule:
        {
            Real3 st  = coll_shape.aacapsule.start;
            Real r    = coll_shape.aacapsule.radius;
            Real h    = coll_shape.aacapsule.height;
            Axis axis = coll_shape.aacapsule.axis;
            int axis_idx = (int)axis;

            // CYLINDER

            model = glm::mat4(1.0f);
            model = glm::translate(model, st);

            if      (axis == Axis::X) model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            else if (axis == Axis::Y) model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

            model = glm::scale(model, glm::vec3(r, r, h));

            wf_shader.setMat4("model", model);

            registry.mesh_manager.zacapsulebody_wfmesh.draw(wf_shader);
            
            // SPHERE 1

            model = glm::mat4(1.0f);
            model = glm::translate(model, st);

            if (axis == Axis::Z)      model = glm::rotate(model, glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            else if (axis == Axis::X) model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            else if (axis == Axis::Y) model = glm::rotate(model, glm::radians(90.0f),  glm::vec3(1.0f, 0.0f, 0.0f));

            model = glm::scale(model, glm::vec3(r, r, r)); 

            wf_shader.setMat4("model", model);

            registry.mesh_manager.zaahemisphere_wfmesh.draw(wf_shader);

            // SPHERE 2

            st[axis_idx] += h;

            model = glm::mat4(1.0f);
            model = glm::translate(model, st);

            if      (axis == Axis::X) model = glm::rotate(model, glm::radians(90.0f),  glm::vec3(0.0f, 1.0f, 0.0f));
            else if (axis == Axis::Y) model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

            model = glm::scale(model, glm::vec3(r, r, r)); 

            wf_shader.setMat4("model", model);

            registry.mesh_manager.zaahemisphere_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::Capsule:
        {
            Real3 st  = coll_shape.capsule.start;
            Real3 nd  = coll_shape.capsule.end;
            Real r    = coll_shape.capsule.radius;

            // CYLINDER

            Real3 dir = nd - st;
            Real h = glm::length(dir);

            if (h < 1e-6f) ASSERT(false, "invalid capsule body");

            Real3 norm_dir = dir / h;
            model = glm::translate(model, st);

            Real3 up = (std::abs(norm_dir.z) < 0.999f) ? Real3(0,0,1) : Real3(1,0,0);
            glm::quat rotation = glm::quatLookAt(-norm_dir, up);
            
            model *= glm::mat4_cast(rotation);
            model = glm::scale(model, glm::vec3(r, r, h));

            wf_shader.setMat4("model", model);
            registry.mesh_manager.zacapsulebody_wfmesh.draw(wf_shader);

            // SPHERE 1

            model = glm::mat4(1.0f);
            model = glm::translate(model, st);
            model *= glm::mat4_cast(rotation);
            model = glm::rotate(model, glm::radians(180.0f), glm::vec3(1, 0, 0));
            model = glm::scale(model, glm::vec3(r, r, r)); 

            wf_shader.setMat4("model", model);

            registry.mesh_manager.zaahemisphere_wfmesh.draw(wf_shader);

            // SPHERE 2

            model = glm::mat4(1.0f);
            model = glm::translate(model, nd);
            model *= glm::mat4_cast(rotation);
            model = glm::scale(model, glm::vec3(r, r, r)); 

            wf_shader.setMat4("model", model);

            registry.mesh_manager.zaahemisphere_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::AARectangle:
        {
            Real3 p1  = coll_shape.aarect.p1;
            Real3 p2  = coll_shape.aarect.p2;
            Real3 center = (p1 + p2) * 0.5f;
            Axis axis = coll_shape.aarect.axis;
            int ax    = (int)axis;

            Real2 proj_p1 = project(p1, axis);
            Real2 proj_p2 = project(p2, axis);

            Real2 ang_vec = proj_p2 - proj_p1;
            Real angle    = atan2(ang_vec.y, ang_vec.x);

            Real3 rot_axis(0.0f);
            rot_axis[ax] = 1.0f; 

            Real3 extents(1.0f);

            extents.x = glm::length(proj_p1-proj_p2);
            extents.z = std::abs(p1[ax] - p2[ax]);

            model = glm::mat4(1.0f);

            model = glm::translate(model, center);

            model = glm::rotate(model, angle, rot_axis);

            if      (axis == Axis::X) model = glm::rotate(model, 3.1415f*0.5f, Real3(0.0f, 1.0f, 0.0f));
            else if (axis == Axis::Y) model = glm::rotate(model, 3.1415f*0.5f, Real3(1.0f, 0.0f, 0.0f));

            model = glm::scale(model, extents);
            
            wf_shader.setMat4("model", model);

            registry.mesh_manager.zarect_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::AxialOBB:
        {
            Real3 center = coll_shape.axialobb.center;
            Real3 size   = 2.0f * coll_shape.axialobb.extents;
            Axis axis    = coll_shape.axialobb.rotation_axis;
            int axis_idx = (int) axis;

            Real3 rot_axis(0.0f);
            rot_axis[axis_idx] = 1.0f;

            Real angle = coll_shape.axialobb.angle;
            if (axis == Axis::Y) angle = -angle;

            model = glm::translate(model, center);
            model = glm::rotate(model, angle, rot_axis);
            model = glm::scale(model, size);

            wf_shader.setMat4("model", model);

            registry.mesh_manager.centered_aabb_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::AA4SidedPyramid:
        {
            Real3 bc     = coll_shape.aa4pyramid.base_center;
            Real2 ext    = 2.0f * coll_shape.aa4pyramid.half_extents;
            Real h       = coll_shape.aa4pyramid.height;
            Axis axis    = coll_shape.aa4pyramid.axis;
            int axis_idx = (int) axis;

            model = glm::mat4(1.0f);
            model = glm::translate(model, bc);

            if      (axis == Axis::X) model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            else if (axis == Axis::Y) model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

            model = glm::scale(model, glm::vec3(ext.x, ext.y, h));

            wf_shader.setMat4("model", model);

            registry.mesh_manager.za4pyramid_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::AAEllipsoid:
        {
            Real3 center  = coll_shape.aaellipsoid.center;
            Real3 extents = coll_shape.aaellipsoid.half_extents;

            model = glm::translate(model, center);
            model = glm::scale(model, extents);

            wf_shader.setMat4("model", model);

            registry.mesh_manager.sphere_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::AACone:
        {

            Real3 bc     = coll_shape.aacone.base_center;
            Real r       = coll_shape.aacone.radius;
            Real h       = coll_shape.aacone.height;
            Axis axis    = coll_shape.aacone.axis;
            int axis_idx = (int) axis;

            model = glm::mat4(1.0f);
            model = glm::translate(model, bc);

            if      (axis == Axis::X) model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            else if (axis == Axis::Y) model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

            model = glm::scale(model, glm::vec3(r, r, h));

            wf_shader.setMat4("model", model);

            registry.mesh_manager.zacone_wfmesh.draw(wf_shader);

            break;
        }
        case CollisionShapeType::AAHollowCylinder:
        {
            const auto& cyl = coll_shape.aahollcyl;
            
            auto setupCylinder = [&](Real radius) 
            {
                glm::mat4 m = glm::translate(glm::mat4(1.0f), cyl.base_center);
                if      (cyl.axis == Axis::X) m = glm::rotate(m, glm::radians(90.0f),  glm::vec3(0.0f, 1.0f, 0.0f));
                else if (cyl.axis == Axis::Y) m = glm::rotate(m, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
                return glm::scale(m, glm::vec3(radius, radius, cyl.height));
            };
            
            wf_shader.setMat4("model", setupCylinder(cyl.inner_radius));
            registry.mesh_manager.zaacylinder_wfmesh.draw(wf_shader);
            
            wf_shader.setMat4("model", setupCylinder(cyl.outer_radius));
            registry.mesh_manager.zaacylinder_wfmesh.draw(wf_shader);
            
            break;
        }
        case CollisionShapeType::AAHemisphere:
        {
            const auto& hemisphere = coll_shape.aahemisphere;
            
            model = glm::translate(model, hemisphere.center);
            
            constexpr struct { glm::vec3 axis; float angle; } rotations[] = {
                {{0.0f, 1.0f, 0.0f},  90.0f},  // Xp
                {{0.0f, 1.0f, 0.0f}, -90.0f},  // Xn
                {{1.0f, 0.0f, 0.0f}, -90.0f},  // Yp
                {{1.0f, 0.0f, 0.0f},  90.0f},  // Yn
                {{1.0f, 0.0f, 0.0f},   0.0f},  // Zp (no rotation)
                {{1.0f, 0.0f, 0.0f}, 180.0f}   // Zn
            };
            
            int idx = static_cast<int>(hemisphere.axis);
            model = glm::rotate(model, glm::radians(rotations[idx].angle), rotations[idx].axis);
            model = glm::scale(model, glm::vec3(hemisphere.radius));
            
            wf_shader.setMat4("model", model);
            registry.mesh_manager.zaahemisphere_wfmesh.draw(wf_shader);
            
            break;
        }
        case CollisionShapeType::AATrapezoid:
        {
            const auto& trapz = coll_shape.aatrapezoid;
            auto [up, u, v]   = getAxes(trapz.axis);

            Real3 bc = trapz.base_center;
            Real3 tc = bc;
            tc[up]  += trapz.height;

            static auto drawRing = [&](const Real3& pos, const Real2& half_extents) 
            {
                glm::mat4 m = glm::translate(glm::mat4(1.0f), pos);

                if      (trapz.axis == Axis::X) m = glm::rotate(m, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
                else if (trapz.axis == Axis::Y) m = glm::rotate(m, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

                m = glm::scale(m, glm::vec3(half_extents.x * 2.0f, half_extents.y * 2.0f, 1.0f));

                wf_shader.setMat4("model", m);
                registry.mesh_manager.xysquare_wfmesh.draw(wf_shader);
            };

            static auto translateOnUVPlane = [&](const Real3& p, Real2 t) 
            { 
                Real3 pn = p; 
                pn[u] += t.x; 
                pn[v] += t.y; 
                return pn; 
            };

            drawRing(bc, trapz.half_extents);
            drawRing(tc, trapz.top_half_extents);

            wf_shader.setMat4("model", glm::mat4(1.0f));

            for (int i : {-1, 1}) 
                for (int j : {-1, 1}) 
                    registry.mesh_manager.debug_line.draw(
                        translateOnUVPlane(bc, trapz.half_extents     * Real2(i, j)),
                        translateOnUVPlane(tc, trapz.top_half_extents * Real2(i, j)), 
                        wf_shader);
            break;
        }
        case CollisionShapeType::AATetrahedron:
        {
            const auto& tetra = coll_shape.aatetra;

            const auto& corner = tetra.corner;

            auto vertices = calculateAATetrahedronVertices(tetra);

            registry.mesh_manager.debug_line.draw(vertices[0], vertices[1], wf_shader);
            registry.mesh_manager.debug_line.draw(vertices[0], vertices[2], wf_shader);
            registry.mesh_manager.debug_line.draw(vertices[0], vertices[3], wf_shader);
            registry.mesh_manager.debug_line.draw(vertices[1], vertices[2], wf_shader);
            registry.mesh_manager.debug_line.draw(vertices[1], vertices[3], wf_shader);
            registry.mesh_manager.debug_line.draw(vertices[2], vertices[3], wf_shader);

            break;
        }
        default:
        {
            ASSERT(false, "Collision shape rendering for shape not yet supported");
        }
    }
} 

void renderCollisionShapesSystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    Shader& wf_shader = registry.shader_manager.wireframe_shader;

    wf_shader.use();
    wf_shader.setVec3("color", Real3(1.0f, 0.0f, 0.0f));
    wf_shader.setFloat("alpha", 1.0f);

    for (const auto& phys_body : registry.static_world) 
    {   
        renderCollisionShape(registry, phys_body.runtime_shape);
    }

    for (const auto& [entity, phys_body] : registry.m_PhysicsBody) 
    {   
        renderCollisionShape(registry, phys_body.runtime_shape);
    }

    wf_shader.setVec3("color", Real3(0.0f, 1.0f, 0.0f));

    for (const auto& [entity, proj_info] : registry.m_ProjectileInfo) 
    {    
        renderCollisionShape(registry, proj_info.runtime_shape);
    }

    wf_shader.setVec3("color", Real3(0.0f, 0.0f, 1.0f));

    for (const auto& [entity, inter_info] : registry.m_InteractableInfo) 
    {   
        renderCollisionShape(registry, inter_info.runtime_shape);
    }

    wf_shader.use();
    wf_shader.setVec3("color", Real3(1.0f, 1.0f, 1.0f));
    wf_shader.setFloat("alpha", 0.3f);

    // for (int i=0; i<registry.static_world_bvh.node_count; i++) 
    // {
    //     Node& node = registry.static_world_bvh.nodes[i];
    //     if (!node.is_leaf) continue;
    //     glm::mat4 model(1.0f);
    //     AABB aabb = node.box;
    //     Real3 size = aabb.maxv - aabb.minv;
    //     model = glm::translate(model, aabb.minv);
    //     model = glm::scale(model, size);
    //     wf_shader.setMat4("model", model);
    //     registry.mesh_manager.aabb_wfmesh.draw(wf_shader);
    // }

    static std::function<void(DynamicAABBTree &tree, Index idx)> draw_tree;
    draw_tree = [&](DynamicAABBTree &tree, Index idx) 
    {
        if (idx == nullAABBTreeIndex) return;
        DynamicAABBTreeNode& node = tree.nodes[idx];
        if (node.is_leaf)
        {
            glm::mat4 model(1.0f);
            AABB aabb = node.box;
            Real3 size = aabb.maxv - aabb.minv;
            model = glm::translate(model, aabb.minv);
            model = glm::scale(model, size);
            wf_shader.setMat4("model", model);
            registry.mesh_manager.aabb_wfmesh.draw(wf_shader);
        }
        else 
        {
            draw_tree(tree, node.child1);
            draw_tree(tree, node.child2);
        }
    };

    draw_tree(registry.dynamic_world_bvh, registry.dynamic_world_bvh.root_idx);

    PROFILE_SYSTEM_END();
}

void renderCrosshairSystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    const CrosshairInfo& crosshair = registry.crosshair;
    const CameraInfo& camera       = registry.camera;

    Shader& shader    = registry.shader_manager.def_shader;
    Shader& wf_shader = registry.shader_manager.wireframe_shader;

    glm::mat4 model = glm::mat4(1.0f);

    model = glm::translate(model, crosshair.position);

    glm::vec3 up       = glm::abs(crosshair.normal.z) < 0.999f ? glm::vec3(0,0,1) : glm::vec3(1,0,0);
    glm::quat rotation = glm::quatLookAt(crosshair.normal, up);

    model *= glm::mat4_cast(rotation);

    model = glm::translate(model, glm::vec3(0.0f, 0.0f, -0.01f));
    model = glm::scale(model,     glm::vec3(0.1f, 0.1f, 1.0f));

    wf_shader.use();
    wf_shader.setVec3("color", Real3(1.0f, 0.0f, 0.0f));
    wf_shader.setMat4("model", model);

    registry.mesh_manager.x_wfmesh.draw(wf_shader);

    shader.use();
    shader.setFloat("uAlpha", 1.0f);
    shader.setFloat("uR", 1.0f);
    shader.setFloat("uG", 0.0f);
    shader.setFloat("uB", 0.0f);
    shader.setMat4("model", model);

    registry.circle_mesh.drawWireframe(shader);

    // SHOOTING LIMITS

    HAS_ASSERT(registry.player_entity, PhysicsBody);
    const PhysicsBody& player_phys_body = GET_COMPONENT(registry.player_entity, PhysicsBody);

    registry.shooting_range_renderer.draw(
        registry.shader_manager.shooting_range_shader, 
        player_phys_body.position + Real3(0.0f, 0.0f, 0.01f),
        MIN_SHOOTING_DISTANCE,
        MAX_SHOOTING_DISTANCE
    );

    // TRAJECTORY

    if (registry.trajectory.valid) 
    {
        const TrajectoryInfo& trajectory = registry.trajectory;
        
        Real g  = GRAVITY_ACCELERATION;

        if (getTrait(registry.equipped_projectile).trajectory_type == ProjectileTrajectoryType::Straight2D ||
            getTrait(registry.equipped_projectile).trajectory_type == ProjectileTrajectoryType::Straight3D)
        {
            g = 0.0f;
        }

        registry.trajectory_renderer.draw(
            registry.shader_manager.trajectory_shader,
            trajectory.x0,
            trajectory.direction,
            trajectory.v0,
            trajectory.te, 
            g,
            getTrait(registry.equipped_projectile).trajectory_type == ProjectileTrajectoryType::Parabolic
        );
    }

    PROFILE_SYSTEM_END();
}

// =======================================================
//  END RENDERING SYSTEM
// =======================================================

// =======================================================
//  ENTITY REMOVAL SYSTEM
// =======================================================

void cleanPhysicsBody(Registry& registry, Entity entity)
{
    if (HAS_COMPONENT(entity, PhysicsBody))
    {
        const PhysicsBody& phys_body = GET_COMPONENT(entity, PhysicsBody);

        ASSERT(registry.dynamic_world_bvh.indexIsValid(phys_body.aabb_tree_idx), "Dead entity physics body must have valid aabb tree index");
        
        dynamicAABBTreeRemoveLeaf(registry.dynamic_world_bvh, phys_body.aabb_tree_idx);
    }
}

void removeAttachedEntities(Registry& registry, Entity entity)
{
    for (auto [attached_entity, attach] : registry.m_Attachment)
    {
        if (attach.target == entity) 
        {
            ASSERT(!HAS_COMPONENT(attached_entity, PhysicsBody), "Attached entity can not physics body");
            ASSERT(!HAS_COMPONENT(attached_entity, InteractableInfo), "Attached entity can not be interactable");

            registry.setEntityToRemove(attached_entity);
        }
    }
}

void lifeTimeSystem(Registry& registry, float delta_time) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    for (auto [entity, lifeTime] : registry.m_LifeTime) {
        lifeTime.life -= delta_time;
        if (lifeTime.life <= 0.0) 
        { 
            removeAttachedEntities(registry, entity);

            registry.setEntityToRemove(entity);
        }
    }

    PROFILE_SYSTEM_END();
}

void deathSystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    for (auto [entity, status] : registry.m_Status) 
    {
        // TODO: handle player death
        if (entity == registry.player_entity) continue;
        
        if (status.life <= 0.0f) 
        {
            HAS_ASSERT(entity, PhysicsBody);

            removeAttachedEntities(registry, entity);
            cleanPhysicsBody(registry, entity);

            registry.setEntityToRemove(entity);
        }
    }

    PROFILE_SYSTEM_END();
}

void removeDeathEntitySystem(Registry& registry) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    registry.removeDeathEntities();

    PROFILE_SYSTEM_END();
}

void flushComponentPools(Registry& registry)
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    registry.flushPools();

    PROFILE_SYSTEM_END();
}

void clearSystem(Registry& registry)
{
    removeDeathEntitySystem(registry);
}   

// =======================================================
//  END ENTITY REMOVAL SYSTEM
// =======================================================

void aimSystem(Registry& registry, InputManager& input_manager) 
{
    TRACE_SYSTEM();
    PROFILE_SYSTEM_BEGIN();

    HAS_ASSERT(registry.player_entity, PhysicsBody);
    const PhysicsBody& target_phys_body = GET_COMPONENT(registry.player_entity, PhysicsBody);

    CrosshairInfo& aim = registry.crosshair;
    CameraInfo&    cam = registry.camera;

    float mouseX = input_manager.curs_x;
    float mouseY = input_manager.curs_y;
    float screenWidth  = 800.0f;
    float screenHeight = 600.0f;

    float x = (2.0f * mouseX) / screenWidth  - 1.0f;
    float y = 1.0f - (2.0f * mouseY) / screenHeight;

    glm::vec4 rayClip(x, y, -1.0f, 1.0f);
    glm::vec4 rayEye = glm::inverse(cam.projection) * rayClip;

    rayEye = glm::vec4(rayEye.x, rayEye.y, -1.0f, 0.0f);

    glm::vec3 rayWorld = glm::normalize(glm::vec3(glm::inverse(cam.view) * rayEye));

    registry.pointer_ray.origin    = cam.position;
    registry.pointer_ray.direction = rayWorld;

    if (mouseX > screenWidth || mouseX < 0.0f || mouseY > screenHeight || mouseY < 0.0f)
        registry.valid_mouse_pointer = false;
    else
        registry.valid_mouse_pointer = true;

    // AIM TARGET POINT CALCULATION

    Real3 collision_point;
    Real3 collision_normal;
    Real  closest_t         = std::numeric_limits<Real>::max();
    bool found_target_point = false;

    QueryOverlapResult<StaticAABBTreeNodeData> static_possible_collision;
    queryOverlap(registry.static_world_bvh, registry.pointer_ray, static_possible_collision);

    QueryOverlapResult<DynamicAABBTreeNodeData> dynamic_possible_collision;
    queryOverlap(registry.dynamic_world_bvh, registry.pointer_ray, dynamic_possible_collision);

    auto updateIfCloser = [&](const CollisionShape& coll_shape)
    {
        RayCollisionResult coll_result = computeRayShapeCollision(registry.pointer_ray, coll_shape);

        if (coll_result.is_colliding)
        {
            if (coll_result.t < closest_t)
            {
                collision_point    = coll_result.entering_point;
                collision_normal   = coll_result.contact_normal; 
                closest_t          = coll_result.t;
                found_target_point = true;
            }
        }
    };

    for (const auto& data : static_possible_collision)
    {
        const StaticPhysicsBody& phys_body = registry.static_world[data.object_idx];
        const CollisionShape& coll_shape   = phys_body.runtime_shape;

        if (phys_body.visible == false) continue;

        updateIfCloser(coll_shape);
    }

    for (const auto& data : dynamic_possible_collision)
    {
        const PhysicsBody& phys_body = COMPONENT(data.entity, PhysicsBody);
        const CollisionShape& coll_shape   = phys_body.runtime_shape;

        updateIfCloser(coll_shape);
    }

    if (found_target_point)
    {
        aim.position = collision_point;
        aim.normal   = collision_normal;
    }
    else
    {
        float planeZ = target_phys_body.position.z; 
        glm::vec3 camPos = cam.position;            

        float t = (planeZ - camPos.z) / rayWorld.z;

        glm::vec3 hitPoint = camPos + rayWorld * t;

        aim.position = Real3(hitPoint.x, hitPoint.y, planeZ);
        aim.normal   = Real3(0.0f, 0.0f, 1.0f);
    }

    PROFILE_SYSTEM_END();
}

