#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

// Constants
#define TRIG_PIN A1
#define ECHO_PIN A0
#define MAX_DISTANCE 200
#define MAX_SPEED 190
#define MAX_SPEED_OFFSET 20
#define SERVO_PIN 10
#define SERVO_CENTER 115
#define SERVO_RIGHT 50
#define SERVO_LEFT 170

// Struct to hold motor configuration
struct MotorConfig {
    AF_DCMotor* motor;
    int speed;
};

// Struct to hold sensor data
struct SensorData {
    int distance;
    int distanceRight;
    int distanceLeft;
};

// Struct to hold robot state
struct RobotState {
    boolean goesForward;
    int speedSet;
    MotorConfig* motors[4];
    Servo* servo;
    SensorData* sensors;
};

// Global variables
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
RobotState* robotState;

// Function prototypes
void initializeRobot();
void cleanupRobot();
int readPing();
int lookDirection(int angle);
void moveMotors(int direction);
void adjustSpeed(MotorConfig* motor, int speed);

void setup() {
    Serial.begin(9600);
    
    // Initialize robot state and components
    robotState = new RobotState();
    robotState->goesForward = false;
    robotState->speedSet = 0;
    robotState->sensors = new SensorData();
    robotState->servo = new Servo();
    
    // Initialize motors
    robotState->motors[0] = new MotorConfig{new AF_DCMotor(1, MOTOR12_1KHZ), 0};
    robotState->motors[1] = new MotorConfig{new AF_DCMotor(2, MOTOR12_1KHZ), 0};
    robotState->motors[2] = new MotorConfig{new AF_DCMotor(3, MOTOR34_1KHZ), 0};
    robotState->motors[3] = new MotorConfig{new AF_DCMotor(4, MOTOR34_1KHZ), 0};
    
    // Setup servo
    robotState->servo->attach(SERVO_PIN);
    robotState->servo->write(SERVO_CENTER);
    delay(2000);
    
    // Initial distance reading
    robotState->sensors->distance = readPing();
    delay(100);
}

void loop() {
    delay(40);
    
    // Read and print sensor data
    Serial.println();
    Serial.print("Right Distance: ");
    Serial.print(robotState->sensors->distanceRight);
    Serial.println();
    Serial.print("Left Distance: ");
    Serial.print(robotState->sensors->distanceLeft);
    Serial.println();
    Serial.print("Distance: ");
    Serial.print(robotState->sensors->distance);
    
    if(robotState->sensors->distance <= 15) {
        moveStop();
        delay(100);
        moveBackward();
        delay(300);
        moveStop();
        delay(200);
        
        // Look around
        robotState->sensors->distanceRight = lookDirection(SERVO_RIGHT);
        delay(200);
        robotState->sensors->distanceLeft = lookDirection(SERVO_LEFT);
        delay(200);
        
        // Choose direction
        if(robotState->sensors->distanceRight >= robotState->sensors->distanceLeft) {
            turnRight();
            moveStop();
        } else {
            turnLeft();
            moveStop();
        }
    } else {
        moveForward();
    }
    
    robotState->sensors->distance = readPing();
    delay(200);
}

int lookDirection(int angle) {
    robotState->servo->write(angle);
    delay(500);
    int distance = readPing();
    delay(100);
    robotState->servo->write(SERVO_CENTER);
    return distance;
}

int readPing() {
    delay(70);
    int cm = sonar.ping_cm();
    return (cm == 0) ? 250 : cm;
}

void moveStop() {
    for(int i = 0; i < 4; i++) {
        robotState->motors[i]->motor->run(RELEASE);
    }
}

void moveForward() {
    if(!robotState->goesForward) {
        robotState->goesForward = true;
        
        for(int i = 0; i < 4; i++) {
            robotState->motors[i]->motor->run(BACKWARD);
        }
        
        // Gradually increase speed
        for(robotState->speedSet = 0; robotState->speedSet < MAX_SPEED; robotState->speedSet += 2) {
            for(int i = 0; i < 4; i++) {
                robotState->motors[i]->motor->setSpeed(robotState->speedSet);
            }
            delay(5);
        }
    }
}

void moveBackward() {
    robotState->goesForward = false;
    
    for(int i = 0; i < 4; i++) {
        robotState->motors[i]->motor->run(FORWARD);
    }
    
    // Gradually increase speed
    for(robotState->speedSet = 0; robotState->speedSet < MAX_SPEED; robotState->speedSet += 2) {
        for(int i = 0; i < 4; i++) {
            robotState->motors[i]->motor->setSpeed(robotState->speedSet);
        }
        delay(5);
    }
}

void turnRight() {
    robotState->motors[0]->motor->run(BACKWARD);
    robotState->motors[1]->motor->run(BACKWARD);
    robotState->motors[2]->motor->run(FORWARD);
    robotState->motors[3]->motor->run(FORWARD);
    delay(500);
    
    for(int i = 0; i < 4; i++) {
        robotState->motors[i]->motor->run(FORWARD);
    }
}

void turnLeft() {
    robotState->motors[0]->motor->run(FORWARD);
    robotState->motors[1]->motor->run(FORWARD);
    robotState->motors[2]->motor->run(BACKWARD);
    robotState->motors[3]->motor->run(BACKWARD);
    delay(500);
    
    for(int i = 0; i < 4; i++) {
        robotState->motors[i]->motor->run(FORWARD);
    }
}
