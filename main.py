import cv2
import numpy as np
import matplotlib.pyplot as plt

high = (110, 255, 255)
low = (0, 130, 130)

blue = (0, 0, 255)

video = cv2.VideoCapture('task.mp4')

while(video.isOpened()):
  #plays and filters for yellow only
  ret, frame = video.read()
  mask = cv2.inRange(frame, low, high)
  #cv2.imshow('mask', mask)
  
  #blur for better canny effect
  blurred = cv2.GaussianBlur(mask,(7,7),0)
  #cv2.imshow('blur', blurred)

  #finds edge of the line
  edges = cv2.Canny(blurred, 50, 150)
  #cv2.imshow("edges",edges)

  kernel = np.ones((5,5),np.uint8)
  erosion = cv2.erode(mask,kernel,iterations = 8)


  #cv2.imshow("erode", erosion)

  combined = cv2.addWeighted(edges,0.5,erosion,0.7,0)
  cv2.imshow('Blended Image',combined)

  if cv2.waitKey(1) & 0xFF == ord('q'):
    break


cv2.destroyAllWindows()