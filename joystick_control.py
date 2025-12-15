#!/usr/bin/env python3
import pygame
import time
from pynput.mouse import Controller

# Initialise mouse
mouse = Controller()

# Initialise joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected")

js = pygame.joystick.Joystick(0)
js.init()

# Parameters
BASE_SPEED = 300.0
BOOST_MULTIPLIER = 3.0
FPS = 60
DT = 1.0 / FPS
DEADZONE = 0.1

# Cursor toggle state
cursor_hidden = False
prev_toggle_button_state = False  # track previous frame state
toggle_button_index = 2           # X button

print("Joystick mouse control running. Ctrl+C to exit.")

try:
    while True:
        pygame.event.pump()

        # --- Left stick axes ---
        x_axis = js.get_axis(0)
        y_axis = js.get_axis(1)

        # Deadzone
        if abs(x_axis) < DEADZONE:
            x_axis = 0.0
        if abs(y_axis) < DEADZONE:
            y_axis = 0.0

        # --- Speed boost using LT (axis 2) or RT (axis 5) ---
        lt = (js.get_axis(2) + 1.0) / 2.0
        rt = (js.get_axis(5) + 1.0) / 2.0
        speed = BASE_SPEED * (1 + BOOST_MULTIPLIER * max(lt, rt))

        # Move mouse
        vx = x_axis * speed
        vy = y_axis * speed
        mouse.move(vx * DT, vy * DT)

        # --- Cursor toggle using X button ---
        button_pressed = js.get_button(toggle_button_index)
        if button_pressed and not prev_toggle_button_state:
            cursor_hidden = not cursor_hidden
            pygame.mouse.set_visible(not cursor_hidden)
        prev_toggle_button_state = button_pressed

        time.sleep(DT)

except KeyboardInterrupt:
    pygame.mouse.set_visible(True)
    print("Exiting cleanly.")
