import csv
import cv2
import numpy as np
import math

def getFrame(videoCapture, frameIndex):
    # Check if video opened successfully
    if not videoCapture.isOpened():
        print("Error: Could not open video.")
        return None

    # Set the position to the specific frame (0-indexed)
    videoCapture.set(cv2.CAP_PROP_POS_FRAMES, frameIndex)
    # Read the frame
    success, frame = videoCapture.read()

    if success:
        return frame
    else:
        print(f"Error: Could not read frame {frameIndex}.")
        return None

def searchContours(contours):
    sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)
    for cnt in sortedContours:
        if cv2.contourArea(cnt) > 200:  # Filter out tiny specs
            x, y, w, h = cv2.boundingRect(cnt)
            return x, y
    return None, None


def identifyTape(videoCapture, frameIndex, show=False):
    img = getFrame(videoCapture, frameIndex)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Make the blue mask
    lowerBlue = np.array([90, 100, 30])
    upperBlue = np.array([125, 255, 255])
    blueMask = cv2.inRange(hsv, lowerBlue, upperBlue)

    # Range for lower red (near 0)
    lowerRedLow = np.array([0, 10, 30])
    upperRedLow = np.array([10, 200, 255])
    maskLow = cv2.inRange(hsv, lowerRedLow, upperRedLow)

    # Range for upper red (near 180)
    lowerRedHigh = np.array([165, 10, 30])
    upperRedHigh = np.array([180, 200, 255])
    maskHigh = cv2.inRange(hsv, lowerRedHigh, upperRedHigh)

    # Combine the red masks using a bitwise OR
    redMask = cv2.bitwise_or(maskLow, maskHigh)

    # Clean up the noise (especially helpful for blurred motion)
    kernel = np.ones((5, 5), np.uint8)
    blueMask = cv2.morphologyEx(blueMask, cv2.MORPH_OPEN, kernel)
    redMask = cv2.morphologyEx(redMask, cv2.MORPH_OPEN, kernel)

    failed = False

    # Find and draw contours for Blue
    contours, _ = cv2.findContours(blueMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blueX, blueY = searchContours(contours)
    if not blueX:
        print("Failed to find blue!")
        failed = True

    # Find and draw contours for Red
    contours, _ = cv2.findContours(redMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    redX, redY = searchContours(contours)
    if not redX:
        print("Failed to find red!")
        failed = True

    if failed:
        cv2.imshow('Detection', img)
        cv2.waitKey(0)

    if show:
        cv2.rectangle(img, (blueX, blueY), (blueX + 5, blueY + 5), (255, 0, 0), 2)
        cv2.putText(img, "Blue Tape", (blueX, blueY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.rectangle(img, (redX, redY), (redX + 5, redY + 5), (0, 0, 255), 2)
        cv2.putText(img, "Red Tape", (redX, redY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow('Detection', img)
        cv2.waitKey(0)

    return blueX, blueY, redX, redY


def calcAngle(blueX, blueY, redX, redY):
    return math.atan2(redY - blueY, blueX - redX)


video = cv2.VideoCapture("IMG_4480 (1).mov")
totalFrames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
startFrame = 1
endFrame = totalFrames
fps = 240


with open("rps_data.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["frame", "rps"])
    prevAngle = calcAngle(*identifyTape(video, startFrame - 1, show=False))
    for i in range(startFrame, endFrame):
        angle = calcAngle(*identifyTape(video, i, show=True))
        deltaAngle = (angle - prevAngle + math.pi) % (2 * math.pi) - math.pi
        rps = (deltaAngle * fps) / (2 * math.pi)
        writer.writerow([i, rps])
        prevAngle = angle
        print(f"{100 * (i - startFrame) / (endFrame - startFrame):6.2f}%", i)

video.release()
