# Collision Detection Simulation

## Overview
This project simulates a 2D environment where multiple spheres (balls) interact with each other using rigid body dynamics and collision detection algorithms. The simulation includes features for applying impulses to spheres and observing their interactions within a bounded plane.

## Project Structure
The project is organized into the following files:

- **src/Spring-Mass Simulator/CollisionDetection.h**: Defines the `CollisionDetection` class, which handles collision detection between spheres. It includes methods for broad and narrow phase collision detection, as well as collision handling using rigid body dynamics.

- **src/Spring-Mass Simulator/main.cpp**: The entry point for the simulation. Initializes the simulation environment, sets up the rendering loop, and handles user input. This file has been modified to create and manage 10 balls and apply an impulse to one of them.

- **src/Spring-Mass Simulator/PerformanceAnalysis.cpp**: Contains functions to measure and analyze the performance of the collision detection algorithms. This file may not require modifications for the simulation of the balls.

- **src/Spring-Mass Simulator/SimulatorWorld.cpp**: Implements the `SimulatorWorld` class, which manages the simulation environment, including the initialization of spheres and their interactions. This file has been modified to create 10 balls, apply an impulse to one, and update their positions and interactions.

- **src/Spring-Mass Simulator/SimulatorWorld.h**: Declares the `SimulatorWorld` class and its methods. It includes properties for managing spheres and their interactions. Modifications have been made to accommodate the new simulation requirements.

- **src/Spring-Mass Simulator/SphereBV.h**: Defines the `SphereBV` class, which represents a sphere in the simulation. It includes properties for mass, velocity, position, and methods for collision detection. Minor adjustments may have been made to support 2D dynamics.

- **src/Spring-Mass Simulator/SphereMesh.h**: Defines the `SphereMesh` class, which represents the visual representation of spheres. This file may not require modifications for the simulation of the balls.

- **src/Spring-Mass Simulator/Utils.h**: Contains utility functions for random number generation and sorting. This file may be useful for initializing the spheres with random properties.

## Setup Instructions
1. Clone the repository to your local machine.
2. Ensure you have the necessary dependencies installed, including OpenGL and GLFW.
3. Build the project using your preferred C++ compiler.
4. Run the `main.cpp` file to start the simulation.

## Usage Guidelines
- The simulation will display 10 spheres moving within a bounded 2D plane.
- An impulse will be applied to one of the spheres, demonstrating the effects of collision and interaction with other spheres.
- You can modify parameters such as the number of spheres, their initial properties, and the impulse magnitude in the `main.cpp` file.

## Future Improvements
- Implement more advanced collision response techniques.
- Add user controls to manipulate the spheres during the simulation.
- Enhance the visual representation of the spheres and the simulation environment.