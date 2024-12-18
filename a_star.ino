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
// 0 = free cell, 1 = obstacle
const uint8_t maze[ROWS][COLS] PROGMEM = {
  {0, 0, 0, 0, 0},
  {1, 0, 1, 0, 0},
  {0, 0, 1, 0, 1},
  {0, 1, 0, 0, 0},
  {0, 0, 0, 0, 0}
};

// Store the cost matrix in PROGMEM
// Each cell has a traversal cost (1-10). Higher means more costly.
const uint8_t costMatrix[ROWS][COLS] PROGMEM = {
  {1, 1, 3, 2, 1},
  {0, 2, 0, 3, 5},
  {7, 2, 4, 20, 0},
  {3, 0, 5, 4, 1},
  {1, 2, 4, 1, 1}
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
  uint16_t gCost;  // Cost from start to this node
  uint16_t hCost;  // Heuristic cost to end node
  uint16_t fCost;  // fCost = gCost + hCost
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

// Cost Matrices (for visualization)
uint16_t gCostMatrixLocal[ROWS][COLS];
uint16_t hCostMatrixLocal[ROWS][COLS];
uint16_t fCostMatrixLocal[ROWS][COLS];

// Function Prototypes
uint16_t heuristic(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
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
uint8_t getCostValue(uint8_t x, uint8_t y);
Movement determineMovement(int8_t dx, int8_t dy, Orientation orientation);
void initMPU6050();
float readGyroZ();
void turnDegrees(int degrees, bool clockwise);
void printCostMatrices();
void printPath();
void initializeCostMatrices();

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

  // Initialize Cost Matrices for visualization
  initializeCostMatrices();
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

      // Reset Cost Matrices
      initializeCostMatrices();

      // Run A* algorithm
      if (AStar()) {
        Serial.println("Path found:");
        printPath();
        Serial.println();
        printCostMatrices();
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
uint16_t heuristic(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
  return abs((int)x1 - (int)x2) + abs((int)y1 - (int)y2);
}

// Get the value of the maze cell from PROGMEM
uint8_t getMazeValue(uint8_t x, uint8_t y) {
  if (x >= ROWS || y >= COLS) return 1; // Treat out-of-bounds as obstacle
  return pgm_read_byte(&(maze[x][y]));
}

// Get the traversal cost of the cell from PROGMEM
uint8_t getCostValue(uint8_t x, uint8_t y) {
  if (x >= ROWS || y >= COLS) return 255; // High cost for out-of-bounds
  return pgm_read_byte(&(costMatrix[x][y]));
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
  
  // Remove the node from openList
  for (uint8_t i = lowestIndex; i < openCount - 1; i++) {
    openList[i] = openList[i + 1];
  }
  openCount--;

  // Add to closedList
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
  gCostMatrixLocal[startNode.x][startNode.y] = startNode.gCost;
  hCostMatrixLocal[startNode.x][startNode.y] = startNode.hCost;
  fCostMatrixLocal[startNode.x][startNode.y] = startNode.fCost;

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

      // Check boundaries and obstacles
      if (newX < 0 || newX >= ROWS || newY < 0 || newY >= COLS)
        continue;
      if (getMazeValue(newX, newY) == 1)
        continue;

      // Check if in closed list
      if (isInClosedList(newX, newY))
        continue;

      // Calculate tentative gCost
      uint8_t cellCost = getCostValue(newX, newY);
      if (cellCost == 0) continue; // Treat 0 as impassable (alternative to maze)

      uint16_t tentativeG = current.gCost + cellCost;

      // Check if this path to neighbor is better
      uint8_t existingG = gCostMatrixLocal[newX][newY];
      if (tentativeG < existingG) {
        uint8_t openIndex;
        if (isInOpenList(newX, newY, openIndex)) {
          // Already in openList, update if new path is better
          openList[openIndex].gCost = tentativeG;
          openList[openIndex].fCost = tentativeG + hCostMatrixLocal[newX][newY];
          openList[openIndex].parent = currentClosedIndex;
          gCostMatrixLocal[newX][newY] = tentativeG;
          fCostMatrixLocal[newX][newY] = openList[openIndex].fCost;
        } else {
          if (openCount < MAX_NODES) {
            Node neighbor;
            neighbor.x = newX;
            neighbor.y = newY;
            neighbor.gCost = tentativeG;
            neighbor.hCost = heuristic(newX, newY, endPoint.x, endPoint.y);
            neighbor.fCost = neighbor.gCost + neighbor.hCost;
            neighbor.parent = currentClosedIndex;
            openList[openCount++] = neighbor;
            gCostMatrixLocal[newX][newY] = neighbor.gCost;
            hCostMatrixLocal[newX][newY] = neighbor.hCost;
            fCostMatrixLocal[newX][newY] = neighbor.fCost;
          }
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

  // Iterate through the path in reverse (from start to end)
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
        // Implement a 180-degree turn
        // turnLeft();
        // turnLeft();
        currentOrientation = (Orientation)((currentOrientation + 2) % 4);
        Serial.println("Turning around and moving forward.");
        moveForward();
        delay(500);
        break;
      default:
        Serial.println("No movement needed.");
        break;
    }

    delay(330); // Adjust based on cell size and robot speed
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
  turnDegrees(88, true);
}

void turnLeft() {
  // Implement a precise 90-degree left turn using MPU6050
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

// Print the cost matrices to Serial
void printCostMatrices() {
  Serial.println("\n--- Cost Matrices ---");

  // Print gCost Matrix
  Serial.println("\nG-Cost Matrix:");
  for (uint8_t i = 0; i < ROWS; i++) {
    for (uint8_t j = 0; j < COLS; j++) {
      if (gCostMatrixLocal[i][j] == 65535) { // Using 65535 as INF
        Serial.print("INF\t");
      } else {
        Serial.print(gCostMatrixLocal[i][j]);
        Serial.print("\t");
      }
    }
    Serial.println();
  }

  // Print hCost Matrix
  Serial.println("\nH-Cost Matrix:");
  for (uint8_t i = 0; i < ROWS; i++) {
    for (uint8_t j = 0; j < COLS; j++) {
      Serial.print(hCostMatrixLocal[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }

  // Print fCost Matrix
  Serial.println("\nF-Cost Matrix:");
  for (uint8_t i = 0; i < ROWS; i++) {
    for (uint8_t j = 0; j < COLS; j++) {
      if (fCostMatrixLocal[i][j] == 65535) { // Using 65535 as INF
        Serial.print("INF\t");
      } else {
        Serial.print(fCostMatrixLocal[i][j]);
        Serial.print("\t");
      }
    }
    Serial.println();
  }
}

// Print the path found
void printPath() {
  for (int8_t i = pathCount - 1; i >= 0; i--) {
    Serial.print("Step ");
    Serial.print(pathCount - i);
    Serial.print(": (");
    Serial.print(path[i].x);
    Serial.print(", ");
    Serial.print(path[i].y);
    Serial.println(")");
  }
}

// Initialize Cost Matrices for visualization
void initializeCostMatrices() {
  for (uint8_t i = 0; i < ROWS; i++) {
    for (uint8_t j = 0; j < COLS; j++) {
      gCostMatrixLocal[i][j] = 65535; // Initialize with a high value (INF)
      hCostMatrixLocal[i][j] = heuristic(i, j, endPoint.x, endPoint.y);
      fCostMatrixLocal[i][j] = 65535; // Initialize with a high value (INF)
    }
  }
}
