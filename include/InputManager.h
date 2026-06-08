// InputManager.h
#pragma once

#include <GLFW/glfw3.h>

#define KEY(key) GLFW_KEY_##key
#define MOUSE(key) GLFW_MOUSE_BUTTON_##key

#define KEY_FIRST GLFW_KEY_SPACE
#define KEY_LAST  GLFW_KEY_LAST
#define NUM_CLICKABLE_KEYS (KEY_LAST - KEY_FIRST + 1)

#define MOUSE_FIRST GLFW_MOUSE_BUTTON_1
#define MOUSE_LAST GLFW_MOUSE_BUTTON_LAST
#define NUM_CLICKABLE_MOUSE (MOUSE_LAST - MOUSE_FIRST + 1)

struct InputManager 
{
public:
    void update();

    bool isClicked(int keyCode);
    bool isPressed(int keyCode);

    bool isMouseClicked(int buttonCode);
    bool isMousePressed(int buttonCode);

    float scroll();

    GLFWwindow* window;

    float curs_x, curs_y;

    void init_callbacks(GLFWwindow* window);

private:

    float scrollv       = 0.0f;
    float prev_scrollv  = 0.0f;
    float scroll_update = 0.0f;

    bool isPressedKey[NUM_CLICKABLE_KEYS]{};
    bool wasPressedKey[NUM_CLICKABLE_KEYS]{};

    bool wasMouseButtonPressed[NUM_CLICKABLE_MOUSE]{}; 
    bool isMouseButtonPressed[NUM_CLICKABLE_MOUSE]{};

    // CALLBACKS

    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
};
