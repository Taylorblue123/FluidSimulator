// filepath: d:\2025Spring\CMSC838M\Homework1\ProjectSourceCode\CollisionDetection\src\Spring-Mass Simulator\main.cpp
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/glm.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include "SimulatorWorld.h"
#include "SphereMesh.h"
#include "Utils.h"
#include "CollisionDetection.h"
#include "SphereBV.h"

// Global variables
SimulatorWorld* worldSimulator = nullptr;
GLuint shaderProgram;
float deltaTime = 0.016f;
GLFWwindow* window;

int numSpheres = 10;        
float minRadius = 0.2f;     
float maxRadius = 3.0f;     
float minVelocity = -5.0f; 
float maxVelocity = 5.0f;  
float minMass = 0.5f;       
float maxMass = 10.0f;      
float worldSize = 20.0f;    

// Camera towards the world center origin
glm::vec3 cameraPos(0.0f, 1.0f, 70.0f);
glm::vec3 cameraFront(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);

void init_shaders() {
    const char* vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;
        out vec3 ourColor;
        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        void main() {
            gl_Position = projection * view * model * vec4(aPos, 1.0);
            ourColor = aColor;
        }
    )";

    const char* fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        in vec3 ourColor;
        void main() {
            FragColor = vec4(ourColor, 1.0f);
        }
    )";

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glUseProgram(shaderProgram);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

void renderSpheres() {
    for (int i = 0; i < numSpheres; i++) {
        SphereBV& sphere = worldSimulator->spheres[i];
        glm::mat4 model = sphere.transform;
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        
        // Render sphere mesh here (assuming you have a function to do this)
        // For example: renderSphereMesh(sphere.mesh);
    }
}

int main() {
    // Initialize GLFW and create a window
    glfwInit();
    window = glfwCreateWindow(800, 600, "Spring-Mass Simulator", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glViewport(0, 0, 800, 600);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Initialize shaders
    init_shaders();

    // Create the simulation world
    worldSimulator = new SimulatorWorld(4, 4, numSpheres, minRadius, maxRadius, minVelocity, maxVelocity, 0.0f, 10.0f, minMass, maxMass, worldSize);

    // Apply an impulse to the first sphere
    worldSimulator->spheres[0].velocity = glm::vec3(5.0f, 0.0f, 0.0f); // Apply impulse to the first sphere

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        processInput(window);
        
        // Step the simulation
        worldSimulator->stepSimulation(deltaTime);

        // Render
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderSpheres();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Clean up
    delete worldSimulator;
    glfwTerminate();
    return 0;
}