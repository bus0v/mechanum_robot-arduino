#include <Arduino.h>
#include <NewPing.h>

#define SONAR_NUM 4      // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(34, 51, MAX_DISTANCE), //back m200
  NewPing(34, 39, MAX_DISTANCE), //right 100
  NewPing(14, 28, MAX_DISTANCE),//left 168 cm
  NewPing(14, 32, MAX_DISTANCE) //front about 42cm on table
};

unsigned int read_distances() {
  unsigned int distances[4];
  for (uint8_t i = 0; i < 1; i++) { // Loop through each sensor and display results.
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    //back right left front
    distances[i] = (sonar[i].ping_cm());
    return distances[0];
  }
  
}
