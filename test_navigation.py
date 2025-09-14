import pygame
import time
import math
from Navigation import BugNavigator
import random  # For simulating sensor values

# Define simulation parameters
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
BOAT_COLOR = (0, 0, 255)
TARGET_COLOR = (255, 0, 0)
OBSTACLE_COLOR = (0, 255, 0)
BACKGROUND_COLOR = (200, 200, 200)

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Navigation Simulation")
clock = pygame.time.Clock()

# Define a simulated environment
class SimulatedBoat:
    def __init__(self, start_x, start_y, target_x, target_y):
        self.x = start_x
        self.y = start_y
        self.target_x = target_x
        self.target_y = target_y
        self.angle = 0  # Simulated heading
        self.speed = 2  # Movement speed
        self.navigator = BugNavigator(
            target_lat=target_x, target_lon=target_y, manual_mode=True,
            manual_lat=start_x, manual_lon=start_y, manual_heading=0
        )
        self.obstacle = pygame.Rect(350, 300, 100, 50)  # A rectangular obstacle

    def simulate_sensor(self):
        """ Simulate ultrasonic sensor readings based on obstacle proximity. """
        # Calculate distance from boat to the center of the obstacle
        distance_to_obstacle = math.sqrt((self.x - (self.obstacle.x + self.obstacle.width / 2)) ** 2 + 
                                         (self.y - (self.obstacle.y + self.obstacle.height / 2)) ** 2)

        # If the boat is near the obstacle, return the distance to it; otherwise, return a large number
        if distance_to_obstacle < 200:  # If within 200 pixels, simulate sensor reading
            return distance_to_obstacle
        else:
            return 100  # No obstacle detected, large distance

    def update(self):
        # Simulate obstacle detection and avoidance
        distance_to_obstacle = self.simulate_sensor()  # Get the simulated sensor value

        """ Simulate movement towards the target and obstacle avoidance. """
        # Use BugNavigator's bug_algorithm for autonomous navigation
        current_lat, current_lon = self.x, self.y
        self.navigator.gps.set_position(current_lat, current_lon)  # Update simulated GPS position

        # Simulate movement towards the target using Bug2 algorithm
        self.navigator.bug_algorithm()

        
        if distance_to_obstacle < 10:  # If the distance is less than 10 cm, consider it an obstacle
            print(f"Obstacle detected at {distance_to_obstacle:.2f} cm, avoiding...")
            self.avoid_obstacle()

        # Check if the boat has reached the target
        target_distance = self.navigator.distance_to_target(current_lat, current_lon)
        print(f"Target distance: {target_distance:.2f}") 
        if target_distance < 2:
            print("Boat has reached the target!")  # Debug print
            return

    def avoid_obstacle(self):
        """ Simple obstacle avoidance: move to the right if there's a collision. """
        self.x += 10  # Move right to avoid the obstacle
        print(f"Avoiding obstacle, new boat position: ({self.x}, {self.y})")

    def draw(self, screen):
        print(f"Drawing boat at: ({int(self.x)}, {int(self.y)})")  # Debug print to track drawing
        pygame.draw.circle(screen, BOAT_COLOR, (int(self.x), int(self.y)), 10)
        pygame.draw.circle(screen, TARGET_COLOR, (int(self.target_x), int(self.target_y)), 5)
        pygame.draw.rect(screen, OBSTACLE_COLOR, self.obstacle)  # Draw the obstacle

# Initialize the simulated boat
boat = SimulatedBoat(start_x=100, start_y=500, target_x=700, target_y=100)

running = True
while running:
    screen.fill(BACKGROUND_COLOR)
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    boat.update()
    boat.draw(screen)
    
    pygame.display.flip()
    clock.tick(30)

pygame.quit()
