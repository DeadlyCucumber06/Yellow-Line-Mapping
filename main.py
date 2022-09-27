import math
import cv2
import numpy as np

high = (110, 255, 255)
low = (0, 130, 130)

video = cv2.VideoCapture('task.mp4')

while(video.isOpened()):
  #plays and filters for yellow only
  ret, frame = video.read()
  mask = cv2.inRange(frame, low, high)
  #cv2.imshow("original", frame)
  
  #blur for better canny effect
  blurred = cv2.GaussianBlur(mask,(7,7),0)
  cv2.imshow('blur', blurred)

  #finds edge of the line
  edges = cv2.Canny(blurred, 50, 150)
  
  bluredges = cv2.GaussianBlur(edges,(7,7),0)

  cv2.imshow('Blended Image', bluredges)
    
  """linesP = cv2.HoughLinesP(bluredges, 1, np.pi / 180, 50, 1000 , 1000, 5000)
  if linesP is not None:
    for i in range(0, len(linesP)):
      l = linesP[i][0]
      cv2.line(frame, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
      print(l[0], l[1], l[2], l[3])
      templ = linesP[i][0]"""
      
  lines = cv2.HoughLines(bluredges, 1, np.pi/180, 325, None, 0 ,0)
  if lines is not None:
    for i in range(0, len(lines)):
      rho = lines[i][0][0]
      theta = lines[i][0][1]
      a = math.cos(theta)
      b = math.sin(theta)
      x0 = a * rho
      y0 = b * rho
      pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
      pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

      cv2.line(frame, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
      cv2.imshow("stuff", frame)
      print("rho", rho, "   theta", theta)

  #cv2.imshow("stuff", frame)
  
  #erosion = cv2.erode(mask,kernel,iterations = 8)


  #cv2.imshow("erode", erosion)

  #combined = cv2.addWeighted(edges,0.5,erosion,0.7,0)
  #cv2.line(combined, (0, 0), (511, 511), (0, 0, 255), 5)


  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

  
cv2.destroyAllWindows()