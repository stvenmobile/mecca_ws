import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import math
import time

class FaceDisplayNode(Node):
    def __init__(self):
        super().__init__('face_display_node')

        # ROS2 Subscriber
        self.subscription = self.create_subscription(
            String,
            'face_expression',
            self.expression_callback,
            10
        )

        self.get_logger().info("Face Display Node Initialized")

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((800, 480))
        pygame.display.set_caption("Robot Face")

        # Eye parameters (scaled up by 1.3)
        self.eye_radius = int(50 * 1.3)
        self.pupil_radius = int(20 * 1.3)
        self.max_pupil_distance = self.eye_radius - self.pupil_radius
        self.left_eye_center = (260, 180)  # Moved slightly higher
        self.right_eye_center = (500, 180)
        self.pupil_offset = {'x': 0, 'y': 0}

        # ROS2 Timer to refresh the display
        self.timer = self.create_timer(0.05, self.update_display)

    def expression_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {command}")
        self.animate_eyes(command)

    def animate_eyes(self, direction):
        target_offset = {'x': 0, 'y': 0}
        if direction == "look_left":
            target_offset['x'] = -self.max_pupil_distance
        elif direction == "look_right":
            target_offset['x'] = self.max_pupil_distance
        elif direction == "look_up":
            target_offset['y'] = -self.max_pupil_distance
        elif direction == "look_down":
            target_offset['y'] = self.max_pupil_distance

        steps = 10
        for _ in range(steps):
            self.pupil_offset['x'] += (target_offset['x'] - self.pupil_offset['x']) / 2
            self.pupil_offset['y'] += (target_offset['y'] - self.pupil_offset['y']) / 2
            self.update_display()
            time.sleep(0.05)

        time.sleep(0.5)
        self.pupil_offset = {'x': 0, 'y': 0}
        self.update_display()

    def update_display(self):
        self.screen.fill((255, 255, 255))
        self.draw_eye(self.left_eye_center)
        self.draw_eye(self.right_eye_center)
        pygame.display.update()

    def draw_eye(self, eye_center):
        eye_x, eye_y = eye_center

        # Draw eye outline (no fill)
        pygame.draw.circle(self.screen, (0, 0, 0), (eye_x, eye_y), self.eye_radius, width=3)

        # Calculate pupil position
        pupil_x = eye_x + self.pupil_offset['x']
        pupil_y = eye_y + self.pupil_offset['y']

        # Ensure pupil stays within eye boundary
        distance = math.sqrt((pupil_x - eye_x) ** 2 + (pupil_y - eye_y) ** 2)
        if distance > self.max_pupil_distance:
            angle = math.atan2(self.pupil_offset['y'], self.pupil_offset['x'])
            pupil_x = eye_x + math.cos(angle) * self.max_pupil_distance
            pupil_y = eye_y + math.sin(angle) * self.max_pupil_distance

        # Draw pupil
        pygame.draw.circle(self.screen, (55, 55, 255), (int(pupil_x), int(pupil_y)), self.pupil_radius)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    pygame.quit()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
