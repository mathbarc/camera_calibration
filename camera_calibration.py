import numpy as np
import cv2
import glob


 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*11,3), np.float32)
objp[:,:2] = np.mgrid[0:11,0:7].T.reshape(-1,2)

objp *= 2
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
cap = cv2.VideoCapture(4,cv2.CAP_V4L2)
if not cap.isOpened():
    exit(-1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

calib_file = cv2.FileStorage("camera_calib.xml", cv2.FileStorage_WRITE)
calib_file.write("cameraWidth", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
calib_file.write("cameraHeight", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

key = 0
ret, img = cap.read()
while(ret and key!=27):
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (11,7), None)

    # If found, add object points, image points (after refining them)
    if ret == True:    
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (11,7), corners2, ret)
    
    cv2.imshow('img', img)
    key = cv2.waitKey(10)
    if key == 32:
        if len(corners):
            objpoints.append(objp)
            imgpoints.append(corners2)
    
    ret, img = cap.read()
 
cv2.destroyAllWindows()
cap.release()

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

calib_file.write("cameraMatrix", camera_matrix)
calib_file.write("distortionCoeffs", dist_coeffs)

calib_file.release()