import cv2
import apriltag
import math

cap = cv2.VideoCapture(0)

while cap.isOpened():
    _,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
        tany = abs(((ptC[1]+ptD[1])/2) - cY)
        tanx = abs(((ptC[0]+ptD[0])/2) - cX)
        rad = math.atan2(tanx,tany)
        deg = int(rad * 180 / math.pi)
        # draw the tag family on the image
        cv2.putText(frame, "({},{}),{}".format(cX,cY,deg), (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("Center Point : ({},{}), Angle : {}".format(cX,cY,deg))

    # show the output image after AprilTag detection
    cv2.imshow("Frame", frame)
    if cv2.waitKey(20) == 27:
        break

cap.release()
cv2.destroyAllWindows()