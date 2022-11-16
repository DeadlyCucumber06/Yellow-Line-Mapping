import math
import cv2
import numpy as np
from paramiko import SubsystemHandler

def gradient(pt1, pt2): #False is horizontal. True is vertical
  ax = pt1[0]
  ay = pt1[1]
  bx = pt2[0]
  by = pt2[1]
  if bx - ax == 0: #vertical
    gradient = True
  elif ((by - ay) / (bx - ax)) > -1 and ((by - ay) / (bx - ax)) < 1:
    gradient = False
  else:
    gradient = True
  return gradient

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
  #cv2.imshow("mask", mask)
  #cv2.imshow('blur', blurred)

  #finds edge of the line
  edges = cv2.Canny(blurred, 50, 150)
  
  bluredges = cv2.GaussianBlur(edges,(7,7),0)

  #cv2.imshow('Blended Image', bluredges) 
    #cv2.line(frame, (int(sumx1 /linesPlen) , int(sumy1 / linesPlen)), (int(sumx2 / linesPlen),int(sumy2 / linesPlen)), (255, 0 ,0), 2, cv2.LINE_AA) 
    #print(meanx, meany)
  
  lines = cv2.HoughLines(bluredges, 1, np.pi/180, 325, None, 0 ,0)
  vlines = [[[0],[0]]]
  hlines = [[[0],[0]]]
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
      ax = pt1[0]
      ay = pt1[1]
      bx = pt2[0]
      by = pt2[1]

      cv2.line(frame, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
      cv2.imshow("stuff", frame)
      
      #print("rho", rho, "   theta", theta)
      #print("a", pt1, "   b", pt2)
      
      m = gradient(pt1,pt2)
      m = gradient(pt1,pt2)

      if m:
        flag = False
        for i in range(len(vlines)):
          templines = vlines[i][0]
          appendlines = vlines[i]
          flag = False
          if ax > (templines[0] - 80) and ax < (templines[0]+80):
            vlines[i][0].append(ax)
            vlines[i][1].append(bx)
            flag = True
            break
          if flag is False:
            vlines.append([[ax],[bx]])
      else:
        flag = False
        for i in range(len(hlines)):
          templines = hlines[i][0]
          appendlines = hlines[i]
          if ax > (templines[0] - 80) and ax < (templines[0]+80):
            hlines[i][0][0] += (ax)
            hlines[i][0][1] += (bx)
            hlines[i][0][2] += (ay)
            hlines[i][0][3] += (by)
            flag = True
            break
          if flag is False:
            hlines.append([[ax],[bx],[ay],[by]])
      print(vlines)
      #print(hlines)
      print("")
        
        
  cv2.imshow("stuff", frame)   
  
  
  #erosion = cv2.erode(mask,kernel,iterations = 8)


  #cv2.imshow("erode", erosion)

  #combined = cv2.addWeighted(edges,0.5,erosion,0.7,0)
  #cv2.line(combined, (0, 0), (511, 511), (0, 0, 255), 5)

  key = cv2.waitKey(1)
  if key == 32:
        cv2.waitKey() 
  if key == ord('q'):
    cv2.waitKey(1)
    break

  
cv2.destroyAllWindows()