#pragma once
#include <glm/glm.hpp>
#include <vector>

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct vertice {
    glm::vec3 pos;
    glm::vec3 color;
};

class SphereMesh {
public:
    SphereMesh(int complexityLevel, glm::vec3 color) {
        this->complexityLevel = complexityLevel;
        this->color = color;
        generateMesh();
    }

    // Getter methods to access the mesh data
    const std::vector<vertice>& getVertices() const {
        return vertices;
    }
    const std::vector<int>& getIndices() const {
        return indices;
    }

private:
    int complexityLevel; // Complexity level of the sphere mesh
    glm::vec3 color;

    std::vector<vertice> vertices; 
    std::vector<int> indices;

    void generateMesh() {
        // Generate the mesh vertices data for the sphere by lathing and longitude
        for(int i = 0; i < complexityLevel; i++){
            for(int j = 0; j < complexityLevel; j++){
                float theta = i * (float)M_PI / complexityLevel; // Latitude angle
                float phi = j * 2.0f * (float)M_PI / complexityLevel; // Longitude angle
                float y = cos(theta); 
                float x = sin(theta) * cos(phi);
                float z = sin(theta) * sin(phi;
                vertice v;
                v.pos = glm::vec3(x, y, z);
                v.color = color;
                vertices.push_back(v); // Add vertex to the mesh
            }
        }

        // Generate the mesh indices data for the sphere
        for(int i = 0; i < complexityLevel - 1; i++){
            for(int j = 0; j < complexityLevel - 1; j++){
                int a = i * complexityLevel + j;
                int b = (i + 1) * complexityLevel + j;
                int c = (i + 1) * complexityLevel + (j + 1);
                int d = i * complexityLevel + (j + 1);
                indices.push_back(a); indices.push_back(b); indices.push_back(c);
                indices.push_back(a); indices.push_back(c); indices.push_back(d);
            }
        }

        // Wrap around the sphere mesh to make it a closed sphere
        for(int i = 0; i < complexityLevel - 1; i++){
            int a = i * complexityLevel + (complexityLevel - 1);
            int b = (i + 1) * complexityLevel + (complexityLevel - 1);
            int c = (i + 1) * complexityLevel + 0;
            int d = i * complexityLevel + 0;
            indices.push_back(a); indices.push_back(b); indices.push_back(c);
            indices.push_back(a); indices.push_back(c); indices.push_back(d);
        }
    }
};