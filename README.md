# Robot-Localization-using-Monte-Carlo-Localization-in-C

# Introduction

Localization is a crucial task for mobile robots to understand their position in an environment.Robot localization is the process of determining the position and orientation of a robot in an environment. This is a crucial task for robots that operate autonomously, as it helps them understand their surroundings and make decisions about where to go and what to do.One popular method for robot localization is Monte Carlo Localization (MCL), which uses a probabilistic approach to estimate the robot's pose. This method uses a particle filter to represent the distribution of possible poses, and updates this distribution as the robot moves and senses its environment.Monte Carlo Localization (MCL) is a probabilistic approach to localize a robot based on sensor measurements and control inputs. It uses random sampling to estimate the position of the robot and updates the probability distribution over time.

# Algorithm

The algorithm of Monte Carlo Localization can be broken down into four steps:

Initialization: The initial position of the robot is uncertain, and the probability distribution over its possible positions is modeled as a set of samples (particles) representing different locations.

Prediction: The robot's motion model is used to predict the movement of particles from the current state to the next.

Update: The robot's sensor measurements are used to update the probability of each particle. Particles with a higher probability are more likely to be the actual position of the robot.

Resampling: Particles with a low probability are replaced by particles with high probability to maintain a high-density representation of the distribution.

# Algorithm Implementation

The following is a step-by-step implementation of Monte Carlo Localization in C:

The code uses a particle filter to represent the distribution of possible poses, and updates the particle distribution with each iteration of the algorithm.
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

#define NUM_PARTICLES 1000
#define NUM_ITERATIONS 100
#define MEASUREMENT_NOISE 0.1
#define MOTION_NOISE 0.1

typedef struct {
  double x;
  double y;
  double theta;
} Particle;

Particle particles[NUM_PARTICLES];
Particle estimatedPose;

double measurement_prob(double measurement, double expectedMeasurement) {
  double error = measurement - expectedMeasurement;
  return (1.0 / (sqrt(2.0 * M_PI * MEASUREMENT_NOISE))) * exp(-0.5 * (error * error) / MEASUREMENT_NOISE);
}

void prediction(double deltaX, double deltaY, double deltaTheta) {
  for (int i = 0; i < NUM_PARTICLES; i++) {
    double noiseX = (rand() / (double)RAND_MAX) * MOTION_NOISE;


Initialize the particles randomly throughout the map. Each particle represents a possible location for the robot.

for (int i = 0; i < num_particles; i++) {
    particle[i].x = rand() % map_width;
    particle[i].y = rand() % map_height;
    particle[i].theta = rand() % 360;
    particle[i].weight = 1.0 / num_particles;
}

For each time step, update the particle's position based on the motion model and the control inputs.

for (int i = 0; i < num_particles; i++) {
    particle[i].x += motion_model(control_inputs, particle[i].theta);
    particle[i].y += motion_model(control_inputs, particle[i].theta);
    particle[i].theta += motion_model(control_inputs, particle[i].theta);
}


Use the sensor measurements to update the weight of each particle. The weight represents the likelihood of the particle's location being the actual location of the robot.

for (int i = 0; i < num_particles; i++) {
    particle[i].weight *= sensor_model(sensor_measurements, particle[i].x, particle[i].y, particle[i].theta);
}

Normalize the weights so that they sum up to 1.

double sum_weights = 0.0;
for (int i = 0; i < num_particles; i++) {
    sum_weights += particle[i].weight;
}
for (int i = 0; i < num_particles; i++) {
    particle[i].weight /= sum_weights;
}


Resample the particles based on their weights. This step ensures that particles with higher weights are more likely to be selected, while particles with lower weights are less likely to be selected.

Particle new_particles[num_particles];
int index = rand() % num_particles;
double beta = 0.0;
double max_weight = 0.0;
for (int i = 0; i < num_particles; i++) {
    max_weight = fmax(max_weight, particle[i].weight);
}
for (int i = 0; i < num_particles; i++) {
    beta += rand() * 2.0 * max_weight;
    while (beta > particle[index].weight) {
        beta -= particle[index].weight;
        index = (index + 1) % num_particles;
    }
   



