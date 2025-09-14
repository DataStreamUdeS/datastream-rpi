import threading
import pygame
import time
import math
from Navigation import BugNavigator

# Declare boat globally
boat = None

def run_navigation_system():
    global boat
    boat = SimulatedBoat(start_x=100, start_y=500, target_x=700, target_y=100)
    
    while not boat.target_reached:
        boat.update()
        time.sleep(1)  # Simulate time for each iteration

# Simulate the boat and its environment
class SimulatedBoat:
    def __init__(self, start_x, start_y, target_x, target_y):
        self.x = start_x
        self.y = start_y
        self.target_x = target_x
        self.target_y = target_y
        self.navigator = BugNavigator(target_lat=target_x, target_lon=target_y, manual_mode=True,
                                      manual_lat=start_x, manual_lon=start_y, manual_heading=0)
        self.target_reached = False

    def simulate_sensor(self):
        """ Simulate ultrasonic sensor readings based on obstacle proximity. """
        distance_to_obstacle = math.sqrt((self.x - self.target_x)**2 + (self.y - self.target_y)**2)
        return distance_to_obstacle

    def update(self):
        # Simulate the boat's movement and navigation
        self.navigator.bug_algorithm()  # Run the navigation step
        distance_to_target = self.navigator.distance_to_target(self.x, self.y)

        if distance_to_target < 2:
            self.target_reached = True
            print("Target reached!")
        
        print(f"Boat position: ({self.x}, {self.y}) - Target distance: {distance_to_target:.2f}")

    def draw(self, screen):
        pygame.draw.circle(screen, (0, 0, 255), (int(self.x), int(self.y)), 10)
        pygame.draw.circle(screen, (255, 0, 0), (int(self.target_x), int(self.target_y)), 5)

# Start the navigation system in a separate thread
navigation_thread = threading.Thread(target=run_navigation_system)
navigation_thread.start()

# Set up Pygame for the graphical simulation
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Simulated Boat Navigation")
clock = pygame.time.Clock()

# Main Pygame loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if boat:  # Check if boat is initialized
        screen.fill((200, 200, 200))
        boat.draw(screen)
        pygame.display.flip()

    clock.tick(30)

pygame.quit()
