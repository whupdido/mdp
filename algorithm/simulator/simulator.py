import pygame

from algorithm.constants import (
    GRID_SIZE,
    WINDOW_SIZE_PX,
    LABEL_SIZE_PX,
    CELL_SIZE_PX,
    START_X,
    START_Y,
)
from algorithm.models import Robot, Arena, Obstacle
from algorithm.enums import Direction


MOVE_DELAY = 150  # milliseconds


def load_asset(path, size):
    image = pygame.image.load(path).convert_alpha()
    return pygame.transform.scale(image, size)


def draw_grid(screen):
    for i in range(GRID_SIZE + 1):
        position = LABEL_SIZE_PX + i * CELL_SIZE_PX

        pygame.draw.line(
            screen,
            "black",
            (position, LABEL_SIZE_PX),
            (position, WINDOW_SIZE_PX)
        )

        pygame.draw.line(
            screen,
            "black",
            (LABEL_SIZE_PX, position),
            (WINDOW_SIZE_PX, position)
        )


def draw_labels(screen, font):
    # X-axis
    for x in range(GRID_SIZE):
        text = font.render(str(x), True, "black")

        text_rect = text.get_rect(
            center=(
                LABEL_SIZE_PX
                + x * CELL_SIZE_PX
                + CELL_SIZE_PX // 2,
                LABEL_SIZE_PX // 2
            )
        )

        screen.blit(text, text_rect)

    # Y-axis
    for y in range(GRID_SIZE):
        text = font.render(str(y), True, "black")

        text_rect = text.get_rect(
            center=(
                LABEL_SIZE_PX // 2,
                LABEL_SIZE_PX
                + y * CELL_SIZE_PX
                + CELL_SIZE_PX // 2
            )
        )

        screen.blit(text, text_rect)


def draw_start_zone(screen, start_image):
    start_x = LABEL_SIZE_PX + START_X * CELL_SIZE_PX
    start_y = LABEL_SIZE_PX + START_Y * CELL_SIZE_PX

    screen.blit(
        start_image,
        (start_x, start_y)
    )


def draw_robot(screen, robot, robot_images):
    robot_image = robot_images[robot.heading]

    robot_x = LABEL_SIZE_PX + robot.x * CELL_SIZE_PX
    robot_y = LABEL_SIZE_PX + robot.y * CELL_SIZE_PX

    screen.blit(
        robot_image,
        (robot_x, robot_y)
    )


def move_robot(robot, direction):
    # Turning is always allowed
    robot.heading = direction

    new_x = robot.x
    new_y = robot.y

    if direction == Direction.NORTH:
        new_y -= 1

    elif direction == Direction.SOUTH:
        new_y += 1

    elif direction == Direction.WEST:
        new_x -= 1

    elif direction == Direction.EAST:
        new_x += 1

    # Check arena boundary
    if not (0 <= new_x < GRID_SIZE and 0 <= new_y < GRID_SIZE):
        return

    # Check obstacles
    for obstacle in obstacles:
        if obstacle.x == new_x and obstacle.y == new_y:
            return
        
    robot.x = new_x
    robot.y = new_y

def draw_obstacle(screen, obstacle, obstacle_images):
    obstacle_image = obstacle_images[obstacle.face]

    obstacle_x = LABEL_SIZE_PX + obstacle.x * CELL_SIZE_PX
    obstacle_y = LABEL_SIZE_PX + obstacle.y * CELL_SIZE_PX

    screen.blit(
        obstacle_image,
        (obstacle_x, obstacle_y)
    )


robot = Robot(
    x=START_X,
    y=START_Y,
    heading=Direction.NORTH
)

obstacles = [
    Obstacle(
        obstacle_id=1,
        x=5,
        y=5,
        face=Direction.NORTH,
        image_id=1
    ),
    Obstacle(
        obstacle_id=2,
        x=10,
        y=8,
        face=Direction.EAST,
        image_id=2
    ),
]

arena = Arena(
    robot=robot,
    obstacles=obstacles
)


pygame.init()

font = pygame.font.Font(None, 20)

screen = pygame.display.set_mode(
    (WINDOW_SIZE_PX, WINDOW_SIZE_PX)
)

pygame.display.set_caption("MDP Arena Simulator")

clock = pygame.time.Clock()


# Load start zone
start_image = load_asset(
    "algorithm/simulator/assets/Start.png",
    (CELL_SIZE_PX, CELL_SIZE_PX)
)


# Load robot images
robot_images = {
    Direction.NORTH: load_asset(
        "algorithm/simulator/assets/robot_north.png",
        (CELL_SIZE_PX, CELL_SIZE_PX)
    ),

    Direction.EAST: load_asset(
        "algorithm/simulator/assets/robot_east.png",
        (CELL_SIZE_PX, CELL_SIZE_PX)
    ),

    Direction.SOUTH: load_asset(
        "algorithm/simulator/assets/robot_south.png",
        (CELL_SIZE_PX, CELL_SIZE_PX)
    ),

    Direction.WEST: load_asset(
        "algorithm/simulator/assets/robot_west.png",
        (CELL_SIZE_PX, CELL_SIZE_PX)
    ),
}

# Load obstacle images
obstacle_images = {
    Direction.NORTH: load_asset(
        "algorithm/simulator/assets/obstacle_north.png",
        (CELL_SIZE_PX, CELL_SIZE_PX)
    ),

    Direction.EAST: load_asset(
        "algorithm/simulator/assets/obstacle_east.png",
        (CELL_SIZE_PX, CELL_SIZE_PX)
    ),

    Direction.SOUTH: load_asset(
        "algorithm/simulator/assets/obstacle_south.png",
        (CELL_SIZE_PX, CELL_SIZE_PX)
    ),

    Direction.WEST: load_asset(
        "algorithm/simulator/assets/obstacle_west.png",
        (CELL_SIZE_PX, CELL_SIZE_PX)
    ),
}


running = True
last_move_time = 0

while running:

    # Handle window events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

    # Handle keyboard movement
    keys = pygame.key.get_pressed()
    current_time = pygame.time.get_ticks()

    if current_time - last_move_time >= MOVE_DELAY:

        if keys[pygame.K_UP]:
            move_robot(arena.robot, Direction.NORTH)
            last_move_time = current_time

        elif keys[pygame.K_DOWN]:
            move_robot(arena.robot, Direction.SOUTH)
            last_move_time = current_time

        elif keys[pygame.K_LEFT]:
            move_robot(arena.robot, Direction.WEST)
            last_move_time = current_time

        elif keys[pygame.K_RIGHT]:
            move_robot(arena.robot, Direction.EAST)
            last_move_time = current_time

    # Clear screen
    screen.fill("white")

    # Draw start zone first
    draw_start_zone(screen, start_image)

    # Draw obstacles
    for obstacle in arena.obstacles:
        draw_obstacle(screen, obstacle, obstacle_images)

    # Draw grid over start zone
    draw_grid(screen)

    # Draw coordinate labels
    draw_labels(screen, font)

    # Draw robot last so it appears on top
    draw_robot(screen, arena.robot, robot_images)

    # Display frame
    pygame.display.flip()

    # Limit to 60 FPS
    clock.tick(60)


pygame.quit()