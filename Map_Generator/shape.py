#!/usr/bin/env python
from PIL import Image, ImageDraw
import random
import math
import sys

def main(x,y, name, dest):
    # Constants
    SQUARE_SIZE = 200
    WIDTH, HEIGHT = (int(x)+1)*SQUARE_SIZE , (int(y)+1)*SQUARE_SIZE
    SHAPE_SIZE = SQUARE_SIZE // 4  # Make shapes smaller
    BACKGROUND_COLOR = (0,0,0)
    SHAPE_COLOR = (100, 100, 100)

    # Function to create a random shape
    def create_random_shape(draw, x, y, size):
        shape = random.choice(["empty","circle", "triangle", "hexagon", "octagon","decagon", "dodecagon", "star"])
        if shape == "circle":
            draw.ellipse((x, y, x + size, y + size), fill=SHAPE_COLOR)
        elif shape == "triangle":
            points = [
                (x + size // 2, y),
                (x, y + size),
                (x + size, y + size),
            ]
            draw.polygon(points, fill=SHAPE_COLOR)
        elif shape == "hexagon":
            draw_regular_polygon(draw, x, y, size, 6)
        elif shape == "octagon":
            draw_regular_polygon(draw, x, y, size, 8)
        elif shape == "decagon":
            draw_regular_polygon(draw, x, y, size, 10)
        elif shape == "dodecagon":
            draw_regular_polygon(draw, x, y, size, 12)
        elif shape == "star":
            draw_star(draw, x + size // 2, y + size // 2, size, size // 2, SHAPE_COLOR)


    # Function to draw a regular polygon with 'n' sides
    def draw_regular_polygon(draw, x, y, size, n):
        angle = 360 / n
        points = []
        for i in range(n):
            x_i = x + size * math.cos(math.radians(i * angle + angle / 2))
            y_i = y + size * math.sin(math.radians(i * angle + angle / 2))
            points.append((x_i, y_i))
        draw.polygon(points, fill=SHAPE_COLOR)

    # Function to draw a star with a black inner part and the outer part filled with the specified color
    # Function to draw a star with both inner and outer parts filled
    def draw_star(draw, x, y, outer_radius, inner_radius, shape_color):
        # Define the points for the outer and inner vertices of the star
        star_points = []

        # Define the angles for the 12 points (6 outer and 6 inner)
        angles = [0, 60, 120, 180, 240, 300]

        for angle in angles:
            angle_rad = angle * math.pi / 180  # Convert angle to radians
            outer_x = x + outer_radius * math.cos(angle_rad)
            outer_y = y - outer_radius * math.sin(angle_rad)
            star_points.append((outer_x, outer_y))

            angle_rad += 30 * math.pi / 180  # Rotate by 30 degrees for inner points
            inner_x = x + inner_radius * math.cos(angle_rad)
            inner_y = y - inner_radius * math.sin(angle_rad)
            star_points.append((inner_x, inner_y))

        # Draw the solid 6-pointed star with the specified shape color
        draw.polygon(star_points, outline=shape_color, fill=shape_color)

    # Create a new image with gray background
    image = Image.new("RGB", (WIDTH+SQUARE_SIZE//2, HEIGHT+SQUARE_SIZE//2), BACKGROUND_COLOR)
    draw = ImageDraw.Draw(image)

    # Add random shapes
    for x in range(SQUARE_SIZE, WIDTH, SQUARE_SIZE):
        for y in range(SQUARE_SIZE, HEIGHT, SQUARE_SIZE):
            create_random_shape(draw, x + SQUARE_SIZE // 4, y + SQUARE_SIZE // 4, SHAPE_SIZE)

    draw.rectangle([0,0, WIDTH+SQUARE_SIZE//2, SQUARE_SIZE//2],outline=SHAPE_COLOR, fill=SHAPE_COLOR)
    draw.rectangle([0,0, SQUARE_SIZE//2, HEIGHT+SQUARE_SIZE//2],outline=SHAPE_COLOR, fill=SHAPE_COLOR)
    draw.rectangle([0,HEIGHT, WIDTH+SQUARE_SIZE//2, HEIGHT+SQUARE_SIZE//2],outline=SHAPE_COLOR, fill=SHAPE_COLOR)
    draw.rectangle([WIDTH,0, WIDTH+SQUARE_SIZE//2, HEIGHT+SQUARE_SIZE//2],outline=SHAPE_COLOR, fill=SHAPE_COLOR)

    # Save the image
    dir_name = name[:-1]
    image.save(dest+"/"+name + ".jpg")

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
