import cv2
import numpy as np

# === CAMERA PARAMETERS ===
# Intrinsic matrix
K = np.array([[1711.7906747459706, 0, 1143.9845804322763],
              [0, 1715.6094835464146, 797.811701204036],
              [0, 0, 1]])

# Distortion coefficients
dist = np.array([0.048709591756734665, -0.1762533277860505,
                 0.0031535408637951086, -0.0021155823984076957, 0])

# Baseline (distance between cameras in meters)
baseline = 0.085  # 8.5 cm

# Open webcams (0 and 1 may change depending on your system)
left_cam = cv2.VideoCapture(0)
right_cam = cv2.VideoCapture(19)

# Set same resolution for both
width = 640
height = 480
for cam in [left_cam, right_cam]:
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

# StereoBM matcher
stereo = cv2.StereoBM_create(numDisparities=16*6, blockSize=15)

print("Press 'q' to quit...")

while True:
    retL, frameL = left_cam.read()
    retR, frameR = right_cam.read()

    if not retL or not retR:
        print("Failed to grab frames")
        break

    # Undistort both images
    frameL_undist = cv2.undistort(frameL, K, dist)
    frameR_undist = cv2.undistort(frameR, K, dist)

    # Convert to grayscale
    grayL = cv2.cvtColor(frameL_undist, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(frameR_undist, cv2.COLOR_BGR2GRAY)

    # Compute disparity
    disparity = stereo.compute(grayL, grayR).astype(np.float32) / 16.0

    # Avoid division by zero and invalid disparity
    disparity[disparity <= 0.0] = 0.1

    # Compute depth map: Z = fx * baseline / disparity
    depth_map = (K[0, 0] * baseline) / disparity

    # Normalize for display
    disp_display = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    disp_display = np.uint8(disp_display)

    depth_display = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
    depth_display = np.uint8(depth_display)

    # Display
    cv2.imshow("Disparity", disp_display)
    cv2.imshow("Depth Map", depth_display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

left_cam.release()
right_cam.release()
cv2.destroyAllWindows()
