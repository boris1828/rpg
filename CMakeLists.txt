cmake_minimum_required(VERSION 3.15)
project(RPG)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Imposta compilazione statica per GLFW
set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)
set(GLFW_BUILD_DOCS OFF)
set(GLFW_INSTALL OFF)
set(GLFW_BUILD_SHARED_LIBS OFF)

# --- GLAD ---
add_library(glad external/glad/src/glad.c)
target_include_directories(glad PUBLIC external/glad/include)

# --- GLFW ---
add_subdirectory(external/glfw)
include_directories(external/glfw/include)

# --- GLM ---
add_subdirectory(external/glm)
include_directories(external/glm)

# --- TinyObjLoader ---
include_directories(external/tinyobjloader)

# --- Remotery ---
set(REMOTERY_DIR "${CMAKE_SOURCE_DIR}/external/Remotery/lib")

# --- Progetto principale ---
add_executable(RPG
    src/main.cpp
    src/InputManager.cpp
    src/Mesh.cpp
    src/Shaders.cpp
    src/VertexArray.cpp
    src/VertexBuffer.cpp
    src/IndexBuffer.cpp
    src/VertexBufferLayout.cpp
    src/ECS.cpp
    src/Collision.cpp
    src/stb_image_impl.cpp

    ${REMOTERY_DIR}/Remotery.c
)

target_include_directories(RPG PRIVATE ${REMOTERY_DIR})
target_compile_definitions(RPG PRIVATE RMT_ENABLED=1)

target_link_libraries(RPG PRIVATE glad glfw glm::glm ${CMAKE_DL_LIBS})

# Includi header dal progetto
target_include_directories(RPG PRIVATE include)

if (MSVC)
    target_compile_options(RPG PRIVATE
        /O2          # Max optimization
        /GL          # Whole Program Optimization
        # /W4
    )
    target_link_options(RPG PRIVATE
        /LTCG        # Link-time code generation
    )
else()
    target_compile_options(RPG PRIVATE
        -O3          # Max optimization
        -march=native
        -ffast-math
        -funroll-loops
    )
endif()


# Imposta una variabile per accedere agli shader da C++
target_compile_definitions(RPG PRIVATE SHADER_PATH="${CMAKE_SOURCE_DIR}/shaders/")
target_compile_definitions(RPG PRIVATE ASSET_PATH="${CMAKE_SOURCE_DIR}/assets/")

# --- Rimuove glad.lib dopo la compilazione ---
add_custom_command(TARGET RPG POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove
        "$<TARGET_FILE_DIR:glad>/glad.lib"
)