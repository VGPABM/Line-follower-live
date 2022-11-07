import cv2
import numpy as np

# Artificial input
input = np.uint8(128 * np.ones((200, 100, 3)))
cv2.rectangle(input, (10, 10), (40, 60), (255, 240, 172), cv2.FILLED)
cv2.circle(input, (70, 100), 20, (172, 172, 255), cv2.FILLED)

# Input to grayscale
gray = cv2.cvtColor(input, cv2.COLOR_RGB2GRAY)

# Simple binary threshold
_, gray = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)

# Find contours
cnts, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# Generate mask
mask = np.ones(gray.shape)
mask = cv2.drawContours(mask, cnts, -1, 0, cv2.FILLED)

# Generate output
output = input.copy()
output[mask.astype(np.bool), :] = 0

cv2.imwrite("input.png", input)
cv2.imwrite("mask.png", np.uint8(255 * mask))
cv2.imwrite("output.png", output)