#pragma once

#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include <random>
#include <vector>

// Particle structure definition
struct Particle {
  lemlib::Pose pose; // Use the lemlib::Pose for x, y, theta
  float weight;      // Weight of the particle (probability)

  // Constructor - required since lemlib::Pose has no default constructor
  Particle(const lemlib::Pose &p = lemlib::Pose(0, 0, 0), float w = 0.0f)
      : pose(p), weight(w) {}
};

// Function declarations
/**
 * Initialize particles around an initial pose estimate
 * @param initialPose The initial pose estimate from chassis.getPose()
 */
void initializeParticles(const lemlib::Pose &initialPose);

/**
 * Update particles based on robot motion (prediction step)
 * @param deltaMotion The change in pose since the last update
 */
void motionUpdate(const lemlib::Pose &deltaMotion);

/**
 * Calculate expected sensor readings for a particle
 * @param particlePose The pose of the particle
 * @param direction Character indicating sensor direction (x and y)
 * @return Expected distance reading
 */
float predictSensorReading(const lemlib::Pose &particlePose,
                            const char direction);

/**
 * Update particle weights based on sensor measurements
 * @param x_dist Distance reading from x sensor
 * @param y_dist Distance reading from y sensor

 */
void measurementUpdate(float x_dist, float y_dist);

/**
 * Resample particles based on their weights
 */
void resampleParticles();

/**
 * Get the estimated pose from particle distribution
 * @return Estimated pose of the robot
 */
lemlib::Pose getEstimatedPose();

/**
 * Update the Monte Carlo Localization system with new sensor data
 * @param chassis The chassis object to get/set pose from
 * @param x_dist Distance reading from x sensor
 * @param y_dist Distance reading from y sensor

 */
void updateMCL(lemlib::Chassis &chassis, float x_dist, float y_dist);

/**
 * Background task for running Monte Carlo Localization
 * @param param Unused parameter required by PROS task API
 */
void mclTask(void *param);

/**
 * Start the MCL background task
 * @param chassis The chassis to use for localization
 */
void startMCL(lemlib::Chassis &chassis);

/**
 * Stop the MCL background task
 */
void stopMCL();

/**
 * Test function to verify particle initialization
 * Initializes particles at (0,0,0) and prints statistics about their
 * distribution
 */
void testParticleInitialization();

lemlib::Pose calculateMotionDelta(const lemlib::Pose &currentPose);