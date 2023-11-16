#!/usr/bin/env python

import random
from PIL import Image
import sys

# Colors
GRAY = (0, 0, 0)
BLACK = (169, 169, 169)

def main(w0, h0, name,dest):
        #make square grid
        if int(w0) % 2 == 0:
                w0 = int(w0) - 1
        if int(h0) % 2 == 0:
                h0 = int(h0) - 1
        CELL_SIZE = 20
        WIDTH = (int(w0)+2)*CELL_SIZE
        HEIGHT = (int(h0)+2)*CELL_SIZE
	
        GRID_WIDTH = WIDTH // CELL_SIZE
        GRID_HEIGHT = HEIGHT // CELL_SIZE


	# Initialize grid with walls
        grid = [[1 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]

	# Function to generate the labyrinth
        def generate_labyrinth(x, y):
                directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
                random.shuffle(directions)

                for dx, dy in directions:
                        new_x, new_y = x + 2 *dx, y + 2 * dy

                        if 0 <= new_x < GRID_WIDTH and 0 <= new_y < GRID_HEIGHT and grid[new_y][new_x] == 1:
                                grid[new_y][new_x] = 0  # Mark the path
                                grid[y + dy][x + dx] = 0  # Remove the wall between current and next cell
                                generate_labyrinth(new_x, new_y)

	# Generate the labyrinth starting from the top-left corner
        generate_labyrinth(1, 1)

	# Create an image to save the labyrinth
        image = Image.new("RGB", (WIDTH, HEIGHT), BLACK)
        pixels = image.load()

	# Draw the labyrinth onto the image
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                color = GRAY if grid[y][x] == 0 else BLACK
                for i in range(CELL_SIZE):
                    for j in range(CELL_SIZE):
                        pixels[x * CELL_SIZE + i, y * CELL_SIZE + j] = color

        dir_name = name[:-1]
        image.save(dest+"/"+name + ".jpg")



if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3],sys.argv[4])
