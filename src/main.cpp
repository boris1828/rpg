#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>
#include <iostream>

#include "Shaders.h"
#include "Mesh.h"
#include "ECS.h"
#include "InputManager.h"
#include "Macro.h"
#include "Render.h"

#ifdef USE_REMOTARY
extern "C" 
{
#include "Remotery.h"
}
#endif

void framebuffer_size_callback(GLFWwindow* window, int width, int height) 
{
    glViewport(0, 0, width, height);
}

int gameLoop() 
{

    #ifdef USE_REMOTARY
    Remotery* rmt;
    rmt_CreateGlobalInstance(&rmt);
    #endif

    if (!glfwInit()) {
        std::cerr << "Errore: impossibile inizializzare GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

// #ifdef __APPLE__
//     glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
// #endif

    GLFWwindow* window = glfwCreateWindow(800, 600, "RPG", nullptr, nullptr);
    if (!window) 
    {
        std::cerr << "Errore: impossibile creare la finestra GLFW\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) 
    {
        std::cerr << "Errore: impossibile inizializzare GLAD\n";
        return -1;
    }

    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;

    // collisionTest();
    // benchmarkRampCollision();
    // runSpeedTest();
    // fastSegmentsAABBBEnchmark();
    // runAARectangleTests();
    // runRectangleBenchmark();
    // runProjectionBenchmark();
    // runAACylinderTests();
    // runAACapsuleTests(); 
    // runRectangleCircleCollisionBenchmark();
    // AABBOverlapBenchmark(); 

    // CONTEXT
    {
        glEnable(GL_DEPTH_TEST);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);

        glEnable(GL_CULL_FACE);  

        Registry registry(window);

        std::cout << "sizeof(Registry): " << sizeof(Registry) << " bytes \n";

        #define PRINT_SIZEOF_COMPONENT(type) std::cout << "sizeof(" #type "): " << sizeof(type) << " bytes \n";

        #define X(type, size, destructor) PRINT_SIZEOF_COMPONENT(type);
            COMPONENT_LIST
        #undef X

        std::cout << "sizeof(" "StaticPhysicsBody" "): " << sizeof(StaticPhysicsBody) << " bytes \n";
        std::cout << "sizeof(" "CollisonShape" "): " << sizeof(CollisionShape) << " bytes \n";

        InputManager& input_manager = registry.input_manager;

        input_manager.init_callbacks(window);

        MeshManager& mesh_manager   = registry.mesh_manager;

        mesh_manager.loadMesh("carrot.obj");
        mesh_manager.loadMesh("obstacle.obj");
        mesh_manager.loadMesh("projectile.obj");
        mesh_manager.loadMesh("aim.obj");
        mesh_manager.loadMesh("ground.obj");
        mesh_manager.loadMesh("target.obj");
        mesh_manager.loadMesh("stair.obj");
        mesh_manager.loadMesh("potion.obj");
        mesh_manager.loadMesh("sphere.obj");
        mesh_manager.loadMesh("hemisphere.obj");

        Mesh* idle_carrot_mesh   = mesh_manager.loadAnimatedMesh("idle_carrot",   "carrot", 12);
        Mesh* moving_carrot_mesh = mesh_manager.loadAnimatedMesh("moving_carrot", "carrot", 12);

        mesh_manager.player_state_mesh_map.insert(AnimState::idle,   idle_carrot_mesh,   12);
        mesh_manager.player_state_mesh_map.insert(AnimState::moving, moving_carrot_mesh, 12);

        registry.loadLevel("ground_collision.obj", {"house.obj"});

        Mesh* door_open_mesh   = mesh_manager.loadMesh("door_open.obj");
        Mesh* door_closed_mesh = mesh_manager.loadMesh("door_closed.obj");

        AABB closed_coll_aabb = {Real3(-0.2f, -0.02f, 0.0f), Real3(0.2f,   0.02f, 0.4f)};
        AABB open_coll_aabb   = {Real3(-0.2f, -0.02f, 0.0f), Real3(-0.16f, 0.38f, 0.4f)};

        assembleDoor(
            registry, 
            Real3(-3.0f, 1.0f, 0.0f), 
            door_open_mesh, door_closed_mesh, Real3(1.0f, 0.8f, 0.2f), 
            open_coll_aabb, closed_coll_aabb, 
            false);

        Entity player = assemblePlayer(registry, moving_carrot_mesh);
        initCamera(registry, player);
        initCrosshair(registry, mesh_manager.meshes[3]);

        // assembleRandomDynamic(registry, mesh_manager.meshes[0], Real3(3.0f, 2.0f, 0.0f));
        // assembleRandomDynamic(registry, mesh_manager.meshes[0], Real3(4.0f, 2.0f, 0.0f));
        // assembleRandomDynamic(registry, mesh_manager.meshes[0], Real3(4.0f, 1.0f, 0.0f));

        assembleStaticAACapsuleObject(
            registry, 
            nullptr, 
            Real3(-2.0f, -2.0f, 0.5f), 
            0.5f,   
            2.0f, 
            Axis::Z
        );

        assembleStaticAACapsuleObject(
            registry, 
            nullptr, 
            Real3(-4.0f, -4.0f, 0.5f), 
            0.3f,               
            3.0f,              
            Axis::X
        );

        assembleStaticAACapsuleObject(
            registry, 
            nullptr, 
            Real3(0.0f, -6.0f, 0.5f), 
            0.4f,   
            4.0f,   
            Axis::Y
        );

        assembleStaticCylinderObject(
            registry, 
            nullptr, 
            Real3(-2.0f, -6.0f, 0.0f), 
            1.2f,   
            1.0f,   
            Axis::Z, 
            Real3(1.0f, 0.8f, 0.2f) 
        );

        assembleStaticCapsuleObject(
            registry, 
            nullptr, 
            Real3(2.0f, -6.0f, 0.0f), 
            Real3(2.0f, 3.0f, 2.0f), 
            0.4f
        );

        assembleStaticSphereObject(
            registry, 
            nullptr, 
            Real3(4.0f, -6.0f, 2.0f),
            1.0f 
        );

        assembleStaticAARectangleObject(
            registry, 
            nullptr, 
            Real3(1.0f, 0.0f, 0.0f),
            Real3(1.0f, 1.0f, 2.0f),
            Axis::Z
        );

        assembleStaticAxialOBBObject(
            registry,
            Real3(4.0f, 1.0f, 0.5f),
            Real3(1.5f, 0.5f, 0.5f),
            glm::half_pi<Real>() * 0.5f, 
            Axis::Y
        );

        assembleStaticAA4SidedPyramidObject(
            registry, 
            Real3(-4.0f, -2.0f, 1.0f),
            Real2(1.0f, 1.0f), 
            -1.0f, 
            Axis::Z, 
            CollisionObjectType::staticObject
        );

        assembleStaticAA4SidedPyramidObject(
            registry, 
            Real3(-6.0f, -2.0f, 0.0f),
            Real2(1.0f, 1.0f), 
            1.0f, 
            Axis::Z, 
            CollisionObjectType::staticStair
        );

        assembleStaticAAEllipsoidObject(
            registry, 
            Real3(6.0f, -5.0f, 1.0f),
            Real3(1.0f, 1.0f, 0.2f)
        );

        assembleStaticAAConeObject(
            registry, 
            Real3(8.0f, -6.0f, 0.7f),
            1.0f, 
            2.0f,   
            Axis::Z
        );

        assembleStaticAAConeObject(
            registry, 
            Real3(10.0f, -6.0f, 2.7f),
            1.0f, 
            -2.0f,   
            Axis::Z
        );

        assembleStaticAAHollowCylinderObject(
            registry, 
            Real3(-2.0f, -8.5f, 0.5f), 
            0.8f,
            1.2f,   
            1.0f,   
            Axis::Z
        );

        assembleStaticAAHollowCylinderObject(
            registry, 
            Real3(-4.5f, -8.0f, 1.2f), 
            0.9f,
            1.1f,   
            2.5f,   
            Axis::Y
        );

        assembleStaticAAHemisphereObject(
            registry,
            Real3(-7.0f, -8.0f, 1.2f), 
            1.0f, 
            AxisDir::Yn
        );

        assembleStaticAAHemisphereObject(
            registry,
            Real3(-9.0f, -8.0f, 0.4f), 
            0.6f, 
            AxisDir::Zn
        );

        assembleStaticAATrapezoidObject(
            registry, 
            Real3(-7.5f, -7.0f, 0.5f), 
            Real2(0.75f, 0.75f), 
            Real2(0.25f, 0.25f), 
            1.0f, 
            Axis::Z
        );

        assembleStaticAATetrahedronObject(
            registry, 
            Real3(-8.5, -7.0, 0.0), 
            Real3(-1.0,  0.5, 1.0)
        );

        assembleStaticAARampObject(
            registry, 
            Real3(-14.0, -8.0, 0.0), 
            Real3(  2.0,  1.0, 1.0), 
            AARampDirection::PosY
        );



        /*

        assembleHouseLayerToggle(registry, Real3(5.0f, 1.5f, 0.0f));

        auto& circularPathPoints = [](glm::vec3 center, float radius, int n) -> std::vector<glm::vec3> {
            std::vector<glm::vec3> circlePoints;
            for (int i = 0; i < n; ++i) {
                float angle = i * glm::two_pi<float>() / n;
                glm::vec3 next_point = glm::vec3(radius * cos(angle), radius * sin(angle), 0.0f) + center;
                circlePoints.push_back(next_point);
            }
            return circlePoints;
        };

        auto path = circularPathPoints(glm::vec3(0.0), 3.0f, 40);

        assemblePathObject(
            registry, 
            mesh_manager.meshes[5], 
            glm::vec3(1.0, 0.1, 0.1), 
            path[0], 
            AABBShape{glm::vec3(-0.2, -0.1, 0.0), glm::vec3(0.2, 0.1, 0.4)}, 
            path, 3.0f);

        */

        glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
        glClearDepth(1.0f);

        Real deltaTime = 1.0/60.0;

        int step = 0;

        while (!glfwWindowShouldClose(window)) 
        {
            if (step%(60*5)==0) PROFILE_SYSTEM_PRINT(step);

            // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            registry.scene_buffer.reset();

            input_manager.update();

            if (input_manager.isPressed(KEY(ESCAPE))) glfwSetWindowShouldClose(window, true);

            // GAME LOOP

            assertSystem(registry);

            // TOCHECK: order of execution of reset and update
            resetSystem(registry);
            updateSystem(registry, deltaTime);

            aimSystem(registry, input_manager);
            shootingSystem(registry, input_manager, player);
            playerInputSystem(registry, input_manager);
            movementSystem(registry, deltaTime);

            // pathFollowingSystem(registry);

            effectSystem(registry, deltaTime);
            
            physicsSystem(registry, deltaTime);

            solveConstraintsSystem(registry, deltaTime);

            collisionShapeUpdateSystem(registry);
            collisionSystem(registry, input_manager);

            interactionSystem(registry);

            attachmentSystem(registry);

            updateAnimationStateSystem(registry);

            houseHidingSystem(registry);
            projectileOrientationSystem(registry);
            cameraFollowSystem(registry, deltaTime);
            cameraSetupSystem(registry);
            transparentOccludingSystem(registry);
            renderSystem(registry);
            renderCrosshairSystem(registry);
            renderCollisionShapesSystem(registry);

            lifeTimeSystem(registry, deltaTime);
            deathSystem(registry);

            clearSystem(registry);

            // END GAME LOOP

            CLEAR_SYSTEM_TRACE();

            // registry.scene_buffer.dumpDepth();
            registry.scene_buffer.drawToScreen();

            // PROFILE_SYSTEM_BEGIN();
            glfwSwapBuffers(window);
            glfwPollEvents();
            // PROFILE_SYSTEM_END();

            step++;
        }
    }

    TextureManager::Get().cleanup();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}


// #include "TestECS.h"

int main()
{
    gameLoop();
}
