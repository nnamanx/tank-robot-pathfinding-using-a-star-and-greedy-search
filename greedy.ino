#include <Wire.h>
#include <avr/pgmspace.h>

// Motor pins
const int motorA1 = 2; // Motor A forward
const int motorA2 = 3; // Motor A backward
const int motorB1 = 4; // Motor B forward
const int motorB2 = 5; // Motor B backward

// Speed control
const int motorSpeed = 128; // Set motor speed (0-255)

// Button pin
const int buttonPin = 7;

// Define maze size
#define ROWS 5
#define COLS 5

// Store the maze in PROGMEM to save SRAM
const uint8_t maze[ROWS][COLS] PROGMEM = {
  {0, 0, 0, 0, 0},
  {1, 0, 1, 0, 0},
  {0, 0, 1, 0, 1},
  {0, 1, 0, 0, 0},
  {0, 0, 0, 0, 0}
};

// Define start and end points
struct Point {
  uint8_t x;
  uint8_t y;
};

const Point startPoint = {0, 0}; // Top-left corner
const Point endPoint = {4, 4};   // Desired end point

// A* Structures and Variables
struct Node {
  uint8_t x, y;    // Coordinates
  uint8_t gCost;   // Cost from start to this node
  uint8_t hCost;   // Heuristic cost to end node
  uint8_t fCost;   // fCost = gCost + hCost
  uint8_t parent;  // Index of parent node (0-24 or 255 if none)
};

#define MAX_NODES 25
#define MAX_PATH 25

Node openList[MAX_NODES];
uint8_t openCount = 0;

Node closedList[MAX_NODES];
uint8_t closedCount = 0;

Node path[MAX_PATH];
uint8_t pathCount = 0;

// Path Execution Variables
bool pathComputed = false;

// Orientation and Movement Enums
enum Orientation { NORTH, EAST, SOUTH, WEST };
enum Movement { MOVE_FORWARD, TURN_LEFT, TURN_AROUND, TURN_RIGHT, NO_MOVEMENT };

// MPU6050 Registers
const uint8_t MPU6050_ADDR = 0x68;
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t GYRO_ZOUT_H = 0x47;

// Function Prototypes
uint8_t heuristic(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
bool AStar();
bool isInClosedList(uint8_t x, uint8_t y);
bool isInOpenList(uint8_t x, uint8_t y, uint8_t &index);
uint8_t getLowestFCost();
void executePath();
void moveForward();
void turnLeft();
void turnRight();
void stopMotors();
uint8_t getMazeValue(uint8_t x, uint8_t y);
Movement determineMovement(int8_t dx, int8_t dy, Orientation orientation);
void initMPU6050();
float readGyroZ();
void turnDegrees(int degrees, bool clockwise);

void setup() {
  // Motor pin setup
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Button pin setup
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize Serial Communication
  Serial.begin(9600);
  Serial.println("Robot Ready. Press the button to start A* Search.");

  // Initialize MPU6050
  initMPU6050();
}

void loop() {
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(buttonPin);

  // Simple Button Debouncing
  if (buttonState != lastButtonState) {
    delay(50); // Debounce delay
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW) { // Button pressed
      Serial.println("\nButton pressed. Starting A* Search...");
      // Reset previous path data
      pathCount = 0;
      openCount = 0;
      closedCount = 0;
      delay(2000);

      // Run A* algorithm
      if (AStar()) {
        Serial.println("Path found:");
        // Print the path in order from start to end
        for (int8_t i = pathCount - 1; i >= 0; i--) {
          Serial.print("Step ");
          Serial.print(pathCount - i);
          Serial.print(": (");
          Serial.print(path[i].x);
          Serial.print(", ");
          Serial.print(path[i].y);
          Serial.println(")");
        }
        Serial.println();
        pathComputed = true;
      } else {
        Serial.println("No path found.");
        pathComputed = false;
      }
    }
  }
  lastButtonState = buttonState;

  // Execute the path if computed
  if (pathComputed) {
    executePath();
    pathComputed = false; // Reset flag after execution
  }
}

// Heuristic Function (Manhattan Distance)
uint8_t heuristic(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
  return abs((int)x1 - (int)x2) + abs((int)y1 - (int)y2);
}

// Get the value of the maze cell from PROGMEM
uint8_t getMazeValue(uint8_t x, uint8_t y) {
  return pgm_read_byte(&(maze[x][y]));
}

// Check if a node is in the closed list
bool isInClosedList(uint8_t x, uint8_t y) {
  for (uint8_t i = 0; i < closedCount; i++) {
    if (closedList[i].x == x && closedList[i].y == y)
      return true;
  }
  return false;
}

// Check if node is in open list
bool isInOpenList(uint8_t x, uint8_t y, uint8_t &index) {
  for (uint8_t i = 0; i < openCount; i++) {
    if (openList[i].x == x && openList[i].y == y) {
      index = i;
      return true;
    }
  }
  return false;
}

// Get index of node with lowest fCost from openList
uint8_t getLowestFCost() {
  if (openCount == 0) {
    return 255; // Indicate no node found
  }

  uint8_t lowestIndex = 0;
  for (uint8_t i = 1; i < openCount; i++) {
    if (openList[i].fCost < openList[lowestIndex].fCost) {
      lowestIndex = i;
    }
  }
  Node current = openList[lowestIndex];
  for (uint8_t i = lowestIndex; i < openCount - 1; i++) {
    openList[i] = openList[i + 1];
  }
  openCount--;
  closedList[closedCount++] = current;
  return closedCount - 1; // Return index in closedList
}

// A* Search Implementation
bool AStar() {
  // Initialize start node
  Node startNode;
  startNode.x = startPoint.x;
  startNode.y = startPoint.y;
  startNode.gCost = 0;
  startNode.hCost = heuristic(startPoint.x, startPoint.y, endPoint.x, endPoint.y);
  startNode.fCost = startNode.gCost + startNode.hCost;
  startNode.parent = 255; // No parent

  openList[openCount++] = startNode;

  while (openCount > 0) {
    uint8_t currentClosedIndex = getLowestFCost();
    if (currentClosedIndex == 255) {
      break; // No node found
    }
    Node current = closedList[currentClosedIndex];

    // Check if reached end
    if (current.x == endPoint.x && current.y == endPoint.y) {
      // Reconstruct path
      path[pathCount++] = current;
      uint8_t parentIndex = current.parent;
      while (parentIndex != 255 && pathCount < MAX_PATH) {
        Node parentNode = closedList[parentIndex];
        path[pathCount++] = parentNode;
        parentIndex = parentNode.parent;
      }
      return true;
    }

    // Explore neighbors (4-directional)
    const int dirs[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };
    for (uint8_t i = 0; i < 4; i++) {
      int newX = current.x + dirs[i][0];
      int newY = current.y + dirs[i][1];

      // Check boundaries
      if (newX < 0 || newX >= ROWS || newY < 0 || newY >= COLS)
        continue;

      // Check obstacles
      if (getMazeValue(newX, newY) == 1)
        continue;

      // Check if in closed list
      if (isInClosedList(newX, newY))
        continue;

      uint8_t newG = current.gCost + 1; // Cost to move forward by one cell
      uint8_t newH = heuristic(newX, newY, endPoint.x, endPoint.y);
      uint8_t newF = newG + newH;

      uint8_t openIndex;
      if (isInOpenList(newX, newY, openIndex)) {
        // Already in openList, check if this path is better
        if (newG < openList[openIndex].gCost) {
          openList[openIndex].gCost = newG;
          openList[openIndex].hCost = newH;
          openList[openIndex].fCost = newF;
          openList[openIndex].parent = currentClosedIndex;
        }
      } else {
        if (openCount < MAX_NODES) {
          Node neighbor;
          neighbor.x = newX;
          neighbor.y = newY;
          neighbor.gCost = newG;
          neighbor.hCost = newH;
          neighbor.fCost = newF;
          neighbor.parent = currentClosedIndex;
          openList[openCount++] = neighbor;
        }
      }
    }
  }
  // No path found
  return false;
}

// Function to determine the required movement
Movement determineMovement(int8_t dx, int8_t dy, Orientation orientation) {
  switch (orientation) {
    case NORTH:
      if (dx == 0 && dy == -1)
        return MOVE_FORWARD;
      else if (dx == 1 && dy == 0)
        return TURN_RIGHT;
      else if (dx == -1 && dy == 0)
        return TURN_LEFT;
      else if (dx == 0 && dy == 1)
        return TURN_AROUND;
      break;
    case EAST:
      if (dx == 1 && dy == 0)
        return MOVE_FORWARD;
      else if (dx == 0 && dy == 1)
        return TURN_RIGHT;
      else if (dx == 0 && dy == -1)
        return TURN_LEFT;
      else if (dx == -1 && dy == 0)
        return TURN_AROUND;
      break;
    case SOUTH:
      if (dx == 0 && dy == 1)
        return MOVE_FORWARD;
      else if (dx == -1 && dy == 0)
        return TURN_RIGHT;
      else if (dx == 1 && dy == 0)
        return TURN_LEFT;
      else if (dx == 0 && dy == -1)
        return TURN_AROUND;
      break;
    case WEST:
      if (dx == -1 && dy == 0)
        return MOVE_FORWARD;
      else if (dx == 0 && dy == -1)
        return TURN_RIGHT;
      else if (dx == 0 && dy == 1)
        return TURN_LEFT;
      else if (dx == 1 && dy == 0)
        return TURN_AROUND;
      break;
  }
  return NO_MOVEMENT;
}


// Execute the computed path
void executePath() {
  Serial.println("Executing Path...");

  // Assume robot starts facing North
  Orientation currentOrientation = NORTH;

  for (int8_t i = pathCount - 1; i > 0; i--) {
    uint8_t currentX = path[i].x;
    uint8_t currentY = path[i].y;
    uint8_t nextX = path[i - 1].x;
    uint8_t nextY = path[i - 1].y;

    int8_t dx = nextX - currentX;
    int8_t dy = nextY - currentY;

    Movement movement = determineMovement(dx, dy, currentOrientation);

    switch (movement) {
      case MOVE_FORWARD:
        moveForward();
        Serial.println("Moving forward.");
        delay(500);
        break;
      case TURN_RIGHT:
        turnRight(); // Implementing right turn
        currentOrientation = (Orientation)((currentOrientation + 1) % 4);
        Serial.println("Turning right and moving forward.");
        delay(500);
        moveForward();
        delay(500);
        break;
      case TURN_LEFT:
        turnLeft(); // Implementing left turn
        currentOrientation = (Orientation)((currentOrientation + 3) % 4);
        Serial.println("Turning left and moving forward.");
        delay(500);
        moveForward();
        delay(500);
        break;
      case TURN_AROUND:
        // turnLeft();
        // turnLeft();
        currentOrientation = (Orientation)((currentOrientation + 2) % 4);
        moveForward();
        Serial.println("Backflip forward.");
        delay(500);
        break;
      default:
        Serial.println("No movement needed.");
        break;
    }

    delay(320); // Adjust based on cell size and robot speed
    stopMotors();
    delay(500); // Brief pause between movements
  }
  Serial.println("Path execution completed.");
}

// Motor control functions
void moveForward() {
  analogWrite(motorA1, 255); // Right motor forward
  digitalWrite(motorA2, LOW);
  analogWrite(motorB1, 255); // Left motor forward
  digitalWrite(motorB2, LOW);
}

void turnRight() {
  // Implement a precise 90-degree right turn using MPU6050
  // Serial.println("Turning Right...");
  turnDegrees(87, true);
}

void turnLeft() {
  // Implement a precise 90-degree left turn using MPU6050
  // Serial.println("Turning Left...");
  turnDegrees(83, false);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

// Initialize MPU6050
void initMPU6050() {
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); // Wake up MPU6050
  Wire.endTransmission(true);
  delay(100); // Wait for MPU6050 to stabilize
  Serial.println("MPU6050 initialized.");
}

// Read Gyro Z-axis data and convert to degrees per second
float readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR); // Begin communication with MPU6050
  Wire.write(GYRO_ZOUT_H);              // Request the Gyro Z-axis register
  Wire.endTransmission(false);          // Restart for read

  Wire.requestFrom(MPU6050_ADDR, (uint8_t)2); // Request 2 bytes from MPU6050

  if (Wire.available() == 2) {
    int16_t raw = Wire.read() << 8 | Wire.read(); // Combine high and low bytes
    float gyro_z = raw / 131.0;         // Convert raw value to deg/s for ±250 dps
    return gyro_z;
  }
  Serial.println("Error: Not enough data from MPU6050.");
  return 0; // Default return if failure
}

// Turn a specific number of degrees clockwise or counter-clockwise
void turnDegrees(int degrees, bool clockwise) {
  float targetDegrees = degrees;
  float accumulatedDegrees = 0.0;
  float gyro_z;

  // Set motor directions for turning
  if (clockwise) {
    // Right turn: Right motor backward, Left motor forward
    digitalWrite(motorA1, LOW);
    analogWrite(motorA2, 200); // Right motor backward
    analogWrite(motorB1, 200); // Left motor forward
    digitalWrite(motorB2, LOW);
  } else {
    // Left turn: Right motor forward, Left motor backward
    analogWrite(motorA1, 155); // Right motor forward
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    analogWrite(motorB2, 255); // Left motor backward
  }

  unsigned long previousMillis = millis();
  while (accumulatedDegrees < targetDegrees) {
    unsigned long currentMillis = millis();
    float deltaTime = (currentMillis - previousMillis) / 1000.0; // Convert to seconds
    previousMillis = currentMillis;

    gyro_z = readGyroZ();
    if (!clockwise) {
      gyro_z = -gyro_z; // Invert for counter-clockwise
    }

    accumulatedDegrees += gyro_z * deltaTime;

    // Optional: Add a timeout to prevent infinite loops
    if (accumulatedDegrees < -degrees || accumulatedDegrees > (degrees * 2)) {
      Serial.println("Turn timeout or error detected.");
      break;
    }

    delay(10); // Small delay for sensor update
  }

  stopMotors();
  Serial.print("Turn completed. Accumulated Degrees: ");
  Serial.println(accumulatedDegrees);
}