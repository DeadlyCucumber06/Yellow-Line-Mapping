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
    
  
  frame_copy = frame.copy()
  
  kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
  dilate = cv2.dilate(edges, kernel, iterations = 1)
  contours, hierarchy = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  cv2.drawContours(frame_copy, contours, -1, (0,255,0), 2)
  font = cv2.FONT_HERSHEY_COMPLEX
  for cnt in contours :
  
    approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
  
    # draws boundary of contours.
    cv2.drawContours(frame_copy, [approx], 0, (0, 0, 255), 5) 
  
    # Used to flatted the array containing
    # the co-ordinates of the vertices.
    n = approx.ravel() 
    i = 0
  
    for j in n :
        if(i % 2 == 0):
            x = n[i]
            y = n[i + 1]
  
            # String containing the co-ordinates.
            string = str(x) + " " + str(y) 
  
            if(i == 0):
                # text on topmost co-ordinate.
                cv2.putText(frame_copy, "Arrow tip", (x, y),
                                font, 0.5, (255, 0, 0)) 
            else:
                # text on remaining co-ordinates.
                cv2.putText(frame_copy, string, (x, y), 
                          frame_copy, 0.5, (0, 255, 0)) 
        i = i + 1
  cv2.imshow("contours", frame_copy)
  lines = cv2.HoughLines(bluredges, 1, np.pi/180, 325, None, 0 ,0)
  vlines = [[[0],[0],[0],[0]]]
  hlines = [[[0],[0],[0],[0]]]
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

      cv2.line(frame, pt1, pt2, (255,0,0), 3, cv2.LINE_AA)
      cv2.imshow("stuff", frame)
      
      #print("rho", rho, "   theta", theta)
      #print("a", pt1, "   b", pt2)
      
      m = gradient(pt1,pt2)

      if m:
        flag = False
        for i in range(len(vlines)):
          if ax > (vlines[i][0][0] - 150) and ax < (vlines[i][0][0]+150):
            vlines[i][0].append(ax)
            vlines[i][1].append(bx)
            vlines[i][2].append(ay)
            vlines[i][3].append(by)
            flag = True
            break
        if flag is False:
          vlines.append([[ax],[bx],[ay],[by]])
          
    for j in range(1,len(vlines)):
      vlineax = int((sum(vlines[j][0])) / (len(vlines[j][0])))
      vlinebx = int((sum(vlines[j][1])) / (len(vlines[j][1])))
      if int(sum(vlines[j][2]) / len(vlines[j][2])) >= 0:
        vlineay = -1000
        vlineby = 1000
      else:
        vlineay = 1000
        vlineby = -1000
      cv2.line(frame, (vlineax, vlineay), (vlineax, vlineby), (0, 0, 255), 3, cv2.LINE_AA)
      #print(hlines)
      #print("")
        
        
  cv2.imshow("stuff", frame)   

  key = cv2.waitKey(1)
  if key == 32:
        cv2.waitKey() 
  if key == ord('q'):
    cv2.waitKey(1)
    break

  
cv2.destroyAllWindows()