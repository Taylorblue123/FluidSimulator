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
GLuint shaderProgram; // Add missing declaration
float deltaTime = 0.016f; 
GLFWwindow* window; // Make window global

int g_complexity = 4;      
int g_numSpheres = 1000;        
float g_radius = 0.45f;
float g_velocity_param = 0.0f; 
float g_mass = 0.20f;      
float g_worldSize = 12.0f;    
// float step = 0.016f; // deltaTime is used for simulation step
float g_damping = 0.6f; 
int g_integratorType_idx = 3; 
float g_gravityCoeff = 12.0f; 
float g_spheresSpacing = 0.00f; 
float g_influenceRadius = 2.0f; 
float g_targetDensity = 0.6f; 
float g_pressureCoefficient = 5.0f;
float g_viscosityCoefficient = 0.285f;

// Camera towards the world center origin
glm::vec3 cameraPos(0.0f, 1.0f, 70.0f);
glm::vec3 cameraFront(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);
float yaw = -90.0f, pitch = 0.0f;
float lastX = 400, lastY = 300;
bool firstMouse = true;
float fov = 60.0f;
bool cursorEnabled = false;

void init_shaders(){

    // Vertex shader source code
    const char* vertexShaderSource = R"(
        #version 330 core
        layout(location=0) in vec3 aPos;
        uniform mat4 model, view, projection;
        void main() {
        gl_Position = projection * view * model * vec4(aPos,1);
        }
    )";

    // Fragment shader source code
    const char* fragmentShaderSource = R"(
        #version 330 core
        uniform vec3 uColor;
        out vec4 FragColor;
        void main() {
        FragColor = vec4(uColor,1);
        }
    )";

    // Compile and link shaders (omitted for brevity)
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

// ---------------------
// Mouse Callback
// ---------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    // if cursorEnabled, then no control for camera view
    if (cursorEnabled)
        return;

    if (firstMouse) {
        lastX = (float)xpos;
        lastY = (float)ypos;
        firstMouse = false;
    }
    float xoffset = (float)xpos - lastX;
    float yoffset = lastY - (float)ypos; // y inverse
    lastX = (float)xpos;
    lastY = (float)ypos;
    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;
    yaw += xoffset;
    pitch += yoffset;
    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;
    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
}

// ---------------------
// Keyboard Callbacks
// ---------------------
void processInput(GLFWwindow* window) {
    float speed = 10.0f * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += speed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= speed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * speed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * speed;

    //  ESC Keyboard
    static bool EscKeyPressed = false;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        if (!EscKeyPressed) {
            cursorEnabled = !cursorEnabled;
            if (cursorEnabled) {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
            else {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                firstMouse = true; 
            }
            EscKeyPressed = true;
        }
    }
    else {
        EscKeyPressed = false;
    }
}

//Render world boundary (wireframe cube)
void renderWorld(){
    if (!worldSimulator) return;

    GLuint VBO, VAO, EBO; // Fix type from Guint to GLuint
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, worldSimulator->cubicWorldVertices.size() * sizeof(vertice), &worldSimulator->cubicWorldVertices[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, worldSimulator->indices.size() * sizeof(int), &worldSimulator->indices[0], GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertice), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertice), (void*)offsetof(vertice, color));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
    glBindVertexArray(0); // Unbind VAO

    // Set the shader program and uniforms
    glUseProgram(shaderProgram);
    GLint loc = glGetUniformLocation(shaderProgram, "uColor");
    glUniform3f(loc, 1.0f, 1.0f, 1.0f); // Set color to white


    // Use the worldSimulator's transform matrix for the bounding box model matrix
    glm::mat4 model = worldSimulator->worldTransformMatrix; 
    glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
    glm::mat4 projection = glm::perspective(glm::radians(fov), (float)800 / (float)600, 0.1f, 100.0f);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    // Render the world boundary
    glBindVertexArray(VAO); //Use previous VAO configeruation
    glLineWidth(2.0f); 
    glDrawElements(GL_LINES, static_cast<GLsizei>(worldSimulator->indices.size()), GL_UNSIGNED_INT, 0); // Draw the cube as wireframe
    glBindVertexArray(0); // Unbind VAO
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glUseProgram(0); // Unbind shader program
    glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); // Unbind EBO
    glBindVertexArray(0); // Unbind VAO
    
}

// Render the spheres with their mesh using OpenGL non-traditional pipeline (triangle)
void renderSpheres() {
    if (!worldSimulator) return;
    
    for (int i = 0; i < worldSimulator->numSpheres; i++) {
        SphereBV& s = worldSimulator->spheres[i];
        // Check if the mesh has vertices and indices before rendering
        const auto& verts = worldSimulator->spheres[i].mesh->getVertices();
        const auto& inds = worldSimulator->spheres[i].mesh->getIndices();
        if(verts.empty() || inds.empty())
            continue; // Skip rendering if mesh data is missing

        //Unique VAO, VBO, EBO for each sphere
        GLuint VBO, VAO, EBO;
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(vertice), &verts[0], GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, inds.size() * sizeof(int), &inds[0], GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertice), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertice), (void*)offsetof(vertice, color));
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
        glBindVertexArray(0); // Unbind VAO

        // Set the shader program and uniforms
        glUseProgram(shaderProgram);
        glm::mat4 model = worldSimulator->spheres[i].transform; // Use the sphere's transform matrix
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        glm::mat4 projection = glm::perspective(glm::radians(fov), (float)800 / (float)600, 0.1f, 100.0f);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniform3fv(glGetUniformLocation(shaderProgram,"uColor"),
             1, glm::value_ptr(s.color));
        // First render the solid sphere
        glBindVertexArray(VAO);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Ensure we're in fill mode for solid rendering
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(inds.size()), GL_UNSIGNED_INT, 0);
        
        // Now render the wireframe overlay
        // Enable polygon offset to prevent z-fighting between solid and wireframe
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);
        
        // Set wireframe mode and render
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(1.0f); // Set the line width for the wireframe
        
        // Optional: Use a different color for wireframe by setting a uniform
        // You'll need to add this uniform to your shader if you want to use it
        // glUniform3f(glGetUniformLocation(shaderProgram, "wireframeColor"), 0.0f, 0.0f, 0.0f); // Black wireframe
        
        // Draw the wireframe
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(inds.size()), GL_UNSIGNED_INT, 0);
        
        // Reset polygon mode and disable polygon offset
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDisable(GL_POLYGON_OFFSET_FILL);
        
        glBindVertexArray(0); // Unbind VAO
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        glUseProgram(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
}

void drawImGuiControls() {
    // ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    ImGui::Begin("Simulator Settings");
    // Sphere Parameters
    ImGui::Text("Sphere Parameters:");
    ImGui::SliderInt("Number of Spheres", &g_numSpheres, 1, 5000);               
    ImGui::SliderInt("Complexity", &g_complexity, 6, 100); // Min complexity 6 for SphereMesh
    ImGui::SliderFloat("Radius", &g_radius, 0.1f, 5.0f);                         
    ImGui::SliderFloat("Initial Velocity", &g_velocity_param, -20.0f, 20.0f); // Use renamed g_velocity_param
    ImGui::SliderFloat("Mass", &g_mass, 0.1f, 10.0f);                             
    ImGui::SliderFloat("World Size", &g_worldSize, 0.0f, 50.0f);                  
    ImGui::SliderFloat("Spheres Spacing", &g_spheresSpacing, 0.0f, 1.0f);        
    ImGui::SliderFloat("Damping Factor", &g_damping, 0.0f, 1.0f);                
    ImGui::Text("Gravity Coefficient: %.2f", g_gravityCoeff); 
    ImGui::SliderFloat("Gravity Coefficient", &g_gravityCoeff, -20.0f, 20.0f); 
    ImGui::Text("Influence Radius: %.2f", g_influenceRadius); 
    ImGui::SliderFloat("Influence Radius", &g_influenceRadius, 0.1f, 2.0f); // Ensure min > 0
    ImGui::Text("Target Density: %.2f", g_targetDensity); 
    ImGui::SliderFloat("Target Density", &g_targetDensity, 0.2f, 5.0f); // Ensure min > 0
    ImGui::Text("Pressure Coefficient: %.2f", g_pressureCoefficient); 
    ImGui::SliderFloat("Pressure Coefficient", &g_pressureCoefficient, 0.0f, 10.0f); 
    ImGui::Text("Viscosity Coefficient: %.2f", g_viscosityCoefficient);
    ImGui::SliderFloat("Viscosity Coefficient", &g_viscosityCoefficient, 0.0f, 5.0f); 
    
    // World Transformation Controls
    if (worldSimulator) {
        ImGui::Separator();
        ImGui::Text("World Bounding Box Transform:");
        bool transformChanged = false;
        if (ImGui::SliderFloat3("Translation##World", glm::value_ptr(worldSimulator->worldTranslationVec), -worldSimulator->worldSize * 2.0f, worldSimulator->worldSize * 2.0f)) {
            transformChanged = true;
        }
        if (ImGui::SliderFloat3("Rotation (Euler Deg)##World", glm::value_ptr(worldSimulator->worldRotationEuler), -180.0f, 180.0f)) {
            transformChanged = true;
        }
        if (transformChanged) {
            worldSimulator->updateWorldTransform();
        }
        ImGui::Separator();
    }


    if (ImGui::Button("Shoot")) {
        if (worldSimulator) {
            worldSimulator->shoot(cameraFront); 
        }
    }
    
    ImGui::Text("Integrator Type: ");
    const char* integratorNames[] = { "Euler", "Midpoint", "RK4", "Verlet" }; // Renamed from integratorTypes
    // static int integratorMethod = 2; // This was local, use g_integratorType_idx
    ImGui::Combo("Integrator", &g_integratorType_idx, integratorNames, IM_ARRAYSIZE(integratorNames));
    
    ImGui::Text("Simulation Method (Collision): "); // Clarified this is for collision
    const char* methods[] = { "Sweep and Prune", "Brute Force", "Grid (NYI)" }; // NYI = Not Yet Implemented
    static int collision_method_idx = 0; // For collision method selection
    ImGui::Combo("Method", &collision_method_idx, methods, IM_ARRAYSIZE(methods));
    // ImGui::Text("Simulation Step: "); // deltaTime is used, not a fixed step from UI here
    
    ImGui::Text("Simulation Control: ");
    if (ImGui::Button("Start/Reset Simulation")) { // Combined Start and Reset
        if (worldSimulator) {
            // Preserve current world transform settings if desired, or reset them
            glm::vec3 oldTranslation = worldSimulator->worldTranslationVec;
            glm::vec3 oldRotation = worldSimulator->worldRotationEuler;
            delete worldSimulator; 
            worldSimulator = new SimulatorWorld(g_complexity, g_numSpheres, g_radius, g_velocity_param, g_mass, g_worldSize, g_spheresSpacing, g_influenceRadius, g_targetDensity, g_pressureCoefficient, g_viscosityCoefficient);
            worldSimulator->worldTranslationVec = oldTranslation; // Restore
            worldSimulator->worldRotationEuler = oldRotation;   // Restore
            worldSimulator->updateWorldTransform(); // Apply restored transform
        } else {
             worldSimulator = new SimulatorWorld(g_complexity, g_numSpheres, g_radius, g_velocity_param, g_mass, g_worldSize, g_spheresSpacing, g_influenceRadius, g_targetDensity, g_pressureCoefficient, g_viscosityCoefficient);
        }
    }
    // Removed "Stop Simulation" as "Start/Reset" handles re-initialization.
    // Removed "Reset Simulation" as "Start/Reset" handles re-initialization.
    // If a separate reset (without re-creating with UI params) is needed, it can use worldSimulator->resetSimulation();
    
    if (ImGui::Button("Exit")) {
        glfwSetWindowShouldClose(window, true);
    }
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Text("Camera Position: (%.1f, %.1f, %.1f)", cameraPos.x, cameraPos.y, cameraPos.z);
    //Use Wasd keys to control camera view
    ImGui::Text("Use WASD to control camera view.");
    ImGui::Text("Press ESC to toggle cursor mode.");
    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}



int main() {
    // Initialize GLFW
    glfwInit();
    window = glfwCreateWindow(800, 600, "World System", NULL, NULL); // Assign to global window
    glfwMakeContextCurrent(window);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // Hide cursor
    glfwSetCursorPosCallback(window, mouse_callback); // Set mouse callback

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);


     // Initialize GLAD
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

     // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Initialize Spring System from global variable input
    // Use g_ prefixed variables for initial constructor call
    worldSimulator = new SimulatorWorld(g_complexity, g_numSpheres, g_radius, g_velocity_param, g_mass, g_worldSize, g_spheresSpacing, g_influenceRadius, g_targetDensity, g_pressureCoefficient, g_viscosityCoefficient);

    init_shaders();

    //Init buffer
    // worldSimulator->updateDensity(); // Initial call to updateDensities (renamed)
    if (worldSimulator) {
        // worldSimulator->updateDensities(); // This is the old SPH, new one is in step
        worldSimulator->updateWorldTransform(); // Ensure matrices are up-to-date
        if (worldSimulator->numSpheres > 0) worldSimulator->updateSpatialLookup(); // Initial spatial grid
    }

    float lastFrameTime = 0.0f;

    while (!glfwWindowShouldClose(window)) {
        float currentFrameTime = (float)glfwGetTime();
        deltaTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        // Process input first
        processInput(window); 

        // Step the simulation state BEFORE rendering
        if (worldSimulator) { // Ensure simulator exists
            // worldSimulator->updateDensity(); // updateDensities is now part of stepSimulation's SPH block
            worldSimulator->stepSimulation(deltaTime, g_damping, g_integratorType_idx, g_gravityCoeff); // Use fixed step from UI
        }

        // Clear screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        // Render world and objects
        renderWorld();
        renderSpheres();

        // Draw UI controls (which might reset/change the simulator)
        drawImGuiControls();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // clean up
    if (worldSimulator) {
        delete worldSimulator;
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;
}