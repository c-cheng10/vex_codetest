#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include "robot/monte.hpp"
using namespace std;

namespace Robot {
    const int num_particles = 1000; //Tune later

    namespace { // blank namespace creates static global variables
        //vector for particles
        vector<Particle> particles;
        random_device rd;
        mt19937 gen(rd()); // Mersenne Twister RNG (PRNG engine that generates pseudorandom numbers from a given seed)

        normal_distribution<float> noise_x(0.0f, 0.1f); // mean is 0, stddev is 0.1 (TUNE LATER)
        normal_distribution<float> noise_y(0.0f, 0.1f); // mean is 0, stddev is 0.1 (TUNE LATER)
        normal_distribution<float> noise_theta(0.0f, 5.0f); //mean is 0, stddev is 5 degrees (TUNE LATER)
        
        lemlib::Pose lastOdomPose(0, 0, 0); // Store last known pose for update calculations

       // pros::Task *mclTaskHandle = nullptr; // no clue how pros tasks work
        bool mclRunning = false;
        lemlib::Chassis *chassisPtr = nullptr; // pointer to the chassis object 
        const float FIELD_DIMENSIONS = 144.0f; // Field is 144 inches 
        //offsets to tune later
        const float x_sensor_offset = 0.0f;
        const float y_sensor_offset = 0.0f;

        // Add boolean flags to control sensor usage
        bool use_x_sensor = false;
        bool use_y_sensor = false;
        
        // this looks important but you have to tune/figure later
        // Add sigma values (stddev) for distance-dependent noise 
        // const float sigma_close_range = 0.3f; // Sigma for distances below 200mm (approx 7.87 inches)
        // const float sigma_far_range = 1.0f; // Sigma for distances above 200mm
        // const float DISTANCE_THRESHOLD_INCHES = 7.87f; // 200mm in inches

        int lastUpdateTime = 0;           // Timestamp of the last MCL update
        const int UPDATE_INTERVAL = 1000; // Update interval in milliseconds (1 second)
        lemlib::Pose lastUpdatedPose(0, 0, 0); // Last pose at which MCL was updated
        const float MIN_MOTION_THRESHOLD = 0.25f; // Minimum motion in inches to trigger update

        // Add constant for motion noise threshold
        const float MOTION_NOISE_THRESHOLD = 0.25f; // Threshold to consider motion as static

        const float UNIFORM_WEIGHT_FACTOR = 0.0001f; // Small uniform weight factor, tune as needed
        const float MIN_WEIGHT = 0.1f; // Minimum weight floor for particles
        const int RESAMPLING_INTERVAL = 3; // Only resample every 3rd update

        // Add variables to store previous sensor readings
        float prev_x_dist = -1.0f; // Initialize with an out-of-range value
        float prev_y_dist = -1.0f;

        // Add this at the global scope
        lemlib::Pose filteredPose(0, 0, 0); // {{ Added filteredPose }}
        } // namespace
    void initializeParticles(const lemlib::Pose &initialPose){
        particles.resize(num_particles);
        
    }
} // namespace Robot