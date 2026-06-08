// InputManager.cpp
#include <iostream>

#include "InputManager.h"

void InputManager::update() 
{

    // KEYS

    for (int key = 0; key<NUM_CLICKABLE_KEYS; key++) 
    {
        wasPressedKey[key] = isPressedKey[key];
        isPressedKey[key]  = glfwGetKey(window, key+KEY_FIRST) == GLFW_PRESS;
    }

    // MOUSE

    for (int button = 0; button<NUM_CLICKABLE_MOUSE; button++) 
    {
        wasMouseButtonPressed[button] = isMouseButtonPressed[button];
        isMouseButtonPressed[button]  = glfwGetMouseButton(window, button) == GLFW_PRESS;
    }

    // MOUSE CURSOR

    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    curs_x = static_cast<float>(xpos);
    curs_y = static_cast<float>(ypos);

    // SCROLL

    // TODO: input manager scroll: re do this thing to output scroll update precisely
    prev_scrollv   = scrollv;
    scrollv       += scroll_update;
    scroll_update  = 0.0f;
}

bool InputManager::isPressed(int keyCode) 
{
    keyCode -= KEY_FIRST;
    return isPressedKey[keyCode];
}

bool InputManager::isClicked(int keyCode) 
{
    keyCode -= KEY_FIRST;
    return !wasPressedKey[keyCode] && isPressedKey[keyCode];
}

bool InputManager::isMousePressed(int buttonCode) 
{
    return isMouseButtonPressed[buttonCode];
}

bool InputManager::isMouseClicked(int buttonCode) 
{
    return !wasMouseButtonPressed[buttonCode] && isMouseButtonPressed[buttonCode];
}

float InputManager::scroll()
{
    return scrollv - prev_scrollv;
}

void InputManager::init_callbacks(GLFWwindow* window)
{
    glfwSetWindowUserPointer(window, this);
    glfwSetScrollCallback(window, InputManager::scroll_callback);
}

void InputManager::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    auto* input = static_cast<InputManager*>(glfwGetWindowUserPointer(window));

    input->scroll_update = static_cast<float>(yoffset);
}