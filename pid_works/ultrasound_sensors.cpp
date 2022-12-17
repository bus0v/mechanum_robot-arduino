#include <Arduino.h>
#include <NewPing.h>

#define SONAR_NUM 4      // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(34, 51, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(34, 39, MAX_DISTANCE), 
  NewPing(14, 28, MAX_DISTANCE),
  NewPing(14, 32, MAX_DISTANCE)
};

int *read_distances() {
  static int distances[4];
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
    distances[i] = sonar[i].ping_cm();
    return distances;
  }
  
}
