from algorithm.enums import Direction 

class Obstacle:
    obstacle_id: int       # Obstacle number
    x: int                 # Grid x-coordinate
    y: int                 # Grid y-coordinate
    face: Direction        # Which side contains the image
    image_id: int          # Recognised image ID