#include <EEPROM.h>

extern float calculateRPS();

// input, output: accel, rps
struct CalibrationPoint {
  float accel;  // Acceleration magnitude
  float rps;    // Actual revolutions per second
  // Any other fields you want to keep
};

// EEPROM storage constants
const int EEPROM_HEADER_ADDRESS = 0;
const int EEPROM_DATA_START_ADDRESS = 4;
const int MAX_CALIBRATION_POINTS = 100;
const int CALIBRATION_POINT_SIZE = sizeof(CalibrationPoint);
const int TOTAL_REQUIRED_EEPROM = EEPROM_DATA_START_ADDRESS + (MAX_CALIBRATION_POINTS * CALIBRATION_POINT_SIZE);

// Structure for header data
struct EEPROMHeader {
  uint16_t numPoints;     // Number of points stored
  uint16_t nextIndex;     // Index where next point will be stored
};

// Function to initialize the EEPROM storage
void initializeCalibrationStorage() {
  // Check if EEPROM has been initialized
  EEPROMHeader header;
  EEPROM.get(EEPROM_HEADER_ADDRESS, header);
  
  // If values look invalid, initialize the header
  if (header.numPoints > MAX_CALIBRATION_POINTS || header.nextIndex >= MAX_CALIBRATION_POINTS) {
    header.numPoints = 0;
    header.nextIndex = 0;
    EEPROM.put(EEPROM_HEADER_ADDRESS, header);
    
    Serial.println("Initialized calibration storage");
  } else {
    Serial.print("Calibration storage has ");
    Serial.print(header.numPoints);
    Serial.println(" points");
  }
}

// TODO: add "5-second reset" code?
void storeCalibrationPoint(float currentAccel, float actualRPS, bool switch3) {
  static bool prevSwitch3 = false;
  
  // Only store when switch3 changes from OFF to ON
  if (switch3 && !prevSwitch3) {
    EEPROMHeader header;
    EEPROM.get(EEPROM_HEADER_ADDRESS, header);
    
    // Check if we have a similar point already
    bool tooSimilar = false;
    const float SIMILARITY_THRESHOLD = 0.05f; // 5% similarity threshold - adjust as needed
    
    if (header.numPoints > 0) {
      // Read all existing points
      for (int i = 0; i < header.numPoints; i++) {
        int pointIndex = (header.nextIndex + i) % MAX_CALIBRATION_POINTS;
        int pointAddress = EEPROM_DATA_START_ADDRESS + (pointIndex * CALIBRATION_POINT_SIZE);
        
        CalibrationPoint existingPoint;
        EEPROM.get(pointAddress, existingPoint);
        
        // Check if new point is too similar to this existing point
        float accelDiff = fabs(existingPoint.accel - currentAccel) / max(existingPoint.accel, currentAccel);
        
        if (accelDiff < SIMILARITY_THRESHOLD) {
          // If the points are similar but the new one has a different RPS,
          // we can replace the old point instead of ignoring
          float rpsDiff = fabs(existingPoint.rps - actualRPS) / max(existingPoint.rps, actualRPS);
          
          if (rpsDiff > SIMILARITY_THRESHOLD) {
            Serial.print("updating ");
            Serial.print(pointAddress);
            Serial.print(" => ");
            Serial.println(actualRPS);
            // The RPS is significantly different, so update this point
            existingPoint.rps = actualRPS;
            EEPROM.put(pointAddress, existingPoint);
          }
          
          tooSimilar = true;
          break;
        }
      }
    }
    
    // If not too similar to any existing point, add it
    if (!tooSimilar) {
      // Calculate point address in the circular buffer
      int pointAddress = EEPROM_DATA_START_ADDRESS + (header.nextIndex * CALIBRATION_POINT_SIZE);
      
      // Create and store the calibration point
      CalibrationPoint point;
      point.accel = currentAccel;
      point.rps = actualRPS;
      Serial.print("storing ");
      Serial.print(currentAccel);
      Serial.print(" => ");
      Serial.println(actualRPS);

      EEPROM.put(pointAddress, point);
      
      // Update header information
      if (header.numPoints < MAX_CALIBRATION_POINTS) {
        header.numPoints++;
      }
      
      // Increment nextIndex with wraparound
      header.nextIndex = (header.nextIndex + 1) % MAX_CALIBRATION_POINTS;
      
      // Save updated header
      EEPROM.put(EEPROM_HEADER_ADDRESS, header);
    }
  }
  
  // Update previous switch state
  prevSwitch3 = switch3;
}

// Function to read all calibration data
void printCalibrationData() {
  EEPROMHeader header;
  EEPROM.get(EEPROM_HEADER_ADDRESS, header);
  
  Serial.print("Calibration Data (");
  Serial.print(header.numPoints);
  Serial.println(" points):");
  Serial.println("accel,RPS");
  
  // Calculate where to start reading in the circular buffer
  int startIndex = 0;
  if (header.numPoints == MAX_CALIBRATION_POINTS) {
    startIndex = header.nextIndex; // If buffer is full, start at oldest point
  }
  
  // Read and print each point
  for (int i = 0; i < header.numPoints; i++) {
    int pointIndex = (startIndex + i) % MAX_CALIBRATION_POINTS;
    int pointAddress = EEPROM_DATA_START_ADDRESS + (pointIndex * CALIBRATION_POINT_SIZE);
    
    CalibrationPoint point;
    EEPROM.get(pointAddress, point);
    
    Serial.print(point.accel);
    Serial.print(",");
    Serial.println(point.rps);
  }
}

// Function to clear all calibration data
void clearCalibrationData() {
  EEPROMHeader header = {0, 0};
  EEPROM.put(EEPROM_HEADER_ADDRESS, header);
  Serial.println("Calibration data cleared");
}

// given current acceleration force, use LUT to estimate RPS
float calculateActualRPS(float currentAccel) {
  EEPROMHeader header;
  EEPROM.get(EEPROM_HEADER_ADDRESS, header);
  
  // If we have no calibration points, return a default RPS
  if (header.numPoints == 0) {
    return calculateRPS(); // if we don't know, use the default. this should make it behave the same way as before if there is no config.
  }
  
  // If we only have one point, return that RPS
  if (header.numPoints == 1) {
    CalibrationPoint point;
    EEPROM.get(EEPROM_DATA_START_ADDRESS, point);
    return point.rps;
  }
  
  // Read all calibration points into an array
  CalibrationPoint points[MAX_CALIBRATION_POINTS];
  
  // Calculate where to start reading in the circular buffer
  int startIndex = 0;
  if (header.numPoints == MAX_CALIBRATION_POINTS) {
    startIndex = header.nextIndex; // If buffer is full, start at oldest point
  }
  
  // Read each point into the array
  for (int i = 0; i < header.numPoints; i++) {
    int pointIndex = (startIndex + i) % MAX_CALIBRATION_POINTS;
    int pointAddress = EEPROM_DATA_START_ADDRESS + (pointIndex * CALIBRATION_POINT_SIZE);
    EEPROM.get(pointAddress, points[i]);
  }
  
  // Find the closest points below and above our current accel value
  int lowerIndex = -1;
  int upperIndex = -1;
  float lowerDiff = 1000000.0f; // Large value
  float upperDiff = 1000000.0f; // Large value
  
  for (int i = 0; i < header.numPoints; i++) {
    float diff = points[i].accel - currentAccel;
    
    if (diff <= 0 && -diff < lowerDiff) {
      // This point is below or equal to our target and closer than previous closest
      lowerDiff = -diff;
      lowerIndex = i;
    }
    
    if (diff >= 0 && diff < upperDiff) {
      // This point is above or equal to our target and closer than previous closest
      upperDiff = diff;
      upperIndex = i;
    }
  }
  
  // Handle edge cases
  if (lowerIndex == -1) {
    // No points below, use lowest point available
    return points[upperIndex].rps;
  }
  
  if (upperIndex == -1) {
    // No points above, use highest point available
    return points[lowerIndex].rps;
  }
  
  // If we found the exact match, return it
  if (lowerIndex == upperIndex) {
    return points[lowerIndex].rps;
  }
  
  // Perform linear interpolation between the points
  float accelDelta = points[upperIndex].accel - points[lowerIndex].accel;
  float rpsDelta = points[upperIndex].rps - points[lowerIndex].rps;
  float factor = (currentAccel - points[lowerIndex].accel) / accelDelta;
  float interpolatedRPS = points[lowerIndex].rps + (factor * rpsDelta);
  
  return interpolatedRPS;
}

