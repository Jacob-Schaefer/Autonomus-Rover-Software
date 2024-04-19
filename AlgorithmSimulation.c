#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h> // For sleep function

// Structure to represent coordinates
typedef struct {
    float x;
    float y;
} Coordinate;

// Structure to represent obstacles
typedef struct {
    Coordinate position;
    float radius;
} Obstacle;

// Function to calculate Euclidean distance between two coordinates
float distance(Coordinate a, Coordinate b) {
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

// Function to simulate obstacle detection using ultrasonic sensor
int checkObstacleDetection(Coordinate currentPos, Obstacle obstacles[], int obstacleCount) {
    // Simulated ultrasonic sensor logic
    for (int i = 0; i < obstacleCount; ++i) {
        if (distance(currentPos, obstacles[i].position) < obstacles[i].radius + 1) {
            return 1; // Obstacle detected
        }
    }
    return 0; // No obstacle detected
}

// Function to visualize drone's path and obstacles
void visualizePath(Coordinate startPos, Coordinate endPos, Coordinate currentPos, Obstacle obstacles[], int obstacleCount) {
    int maxX = (int) fmax(startPos.x, fmax(endPos.x, currentPos.x)) + 2; // Add buffer for visualization
    int maxY = (int) fmax(startPos.y, fmax(endPos.y, currentPos.y)) + 2; // Add buffer for visualization
    char map[maxY][maxX]; // 2D array to represent map

    // Initialize map with empty space
    for (int y = 0; y < maxY; y++) {
        for (int x = 0; x < maxX; x++) {
            map[y][x] = '.';
        }
    }

    // Mark obstacles on the map
    for (int i = 0; i < obstacleCount; i++) {
        int x = (int) obstacles[i].position.x;
        int y = (int) obstacles[i].position.y;
        map[y][x] = 'X';
    }

    // Mark start, end, and current positions on the map
    int startX = (int) startPos.x;
    int startY = (int) startPos.y;
    int endX = (int) endPos.x;
    int endY = (int) endPos.y;
    int currentX = (int) currentPos.x;
    int currentY = (int) currentPos.y;
    map[startY][startX] = 'S';
    map[endY][endX] = 'E';
    map[currentY][currentX] = 'D'; // 'D' for drone

    // Print map
    for (int y = 0; y < maxY; y++) {
        for (int x = 0; x < maxX; x++) {
            printf("%c ", map[y][x]);
        }
        printf("\n");
    }
}

// Function to navigate drone around obstacles
void navigateAroundObstacle(Coordinate *currentPos, Coordinate destination, Obstacle obstacles[], int obstacleCount) {
    Coordinate direction = {destination.x - currentPos->x, destination.y - currentPos->y};
    float magnitude = sqrt(direction.x * direction.x + direction.y * direction.y);
    direction.x /= magnitude;
    direction.y /= magnitude;

    float temp = direction.x;
    direction.x = direction.y;
    direction.y = -temp;

    currentPos->x += direction.x;
    currentPos->y += direction.y;
}

// Function to navigate drone towards destination
void navigateToDestination(Coordinate *currentPos, Coordinate destination, Obstacle obstacles[], int obstacleCount) {
    printf("Starting navigation...\n\n");
    while (distance(*currentPos, destination) > 1) { // Threshold distance to consider reached destination
        printf("Current Position: (%.0f, %.0f)\n", currentPos->x, currentPos->y);
        visualizePath((Coordinate){5, 3}, (Coordinate){20, 17}, *currentPos, obstacles, obstacleCount); // Pass current position to visualize
        if (checkObstacleDetection(*currentPos, obstacles, obstacleCount)) {
            printf("Obstacle detected! Navigating around...\n");
            navigateAroundObstacle(currentPos, destination, obstacles, obstacleCount);
        } else {
            // Move drone towards destination
            if (currentPos->x < destination.x)
                currentPos->x += 1; // Move drone 1 unit in x direction
            else
                currentPos->x -= 1; // Move drone 1 unit in negative x direction

            if (currentPos->y < destination.y)
                currentPos->y += 1; // Move drone 1 unit in y direction
            else
                currentPos->y -= 1; // Move drone 1 unit in negative y direction
        }
        printf("\n");
        // Simulate delay for visualization (replace with actual drone movement delay)
        sleep(1);
        //  system("cls"); // Clear console for updated visualization
    }
    printf("Destination reached!\n");
}

int main() {
    Coordinate start = {5, 3}; // Start coordinates
    Coordinate destination = {20, 17}; // Destination coordinates
    Obstacle obstacles[] = {
            {{8, 6}, 1}, {{8, 8}, 1}, {{8, 10}, 1}, {{9, 7}, 1}, {{9, 9}, 1},
            {{9, 11}, 1}, {{10, 6}, 1}, {{10, 8}, 1}, {{10, 10}, 1}, {{10, 12}, 1},
            {{11, 7}, 1}, {{11, 9}, 1}, {{11, 11}, 1}, {{12, 6}, 1}, {{12, 8}, 1},
            {{12, 10}, 1}, {{12, 12}, 1}, {{13, 7}, 1}, {{13, 9}, 1}, {{13, 11}, 1}
    }; // Example obstacles forming a clump
    int obstacleCount = sizeof(obstacles) / sizeof(obstacles[0]);

    // Simulate drone navigation
    navigateToDestination(&start, destination, obstacles, obstacleCount);

    return 0;
}