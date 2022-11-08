import math
import cv2
import numpy as np
from paramiko import SubsystemHandler

"""def compmean(mean, list, c1, c2, verthorz):
  for i in range(len(list)):
    if (mean +20) >= list[i][i] and (mean - 20) <= list[i][i]:
      list[i].insert(mean, 1)
    else:
      templist = []
      templist += (mean, c1, c2, verthorz)
      list.append(templist)"""

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
    
  """linesP = cv2.HoughLinesP(bluredges, 1, np.pi / 180, 80, 300 , 10)
  if linesP is not None:
    sumh = [0, 0, 0, 0, 0]
    sumv = [0, 0, 0, 0, 0]
    linesPlen = len(linesP)
    for i in range(0, linesPlen):
      l = linesP[i][0]
      cv2.line(frame, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
      #print(l[0], l[1], l[2], l[3], linesPlen)
      gradient = (l[3] - l[1]) / (l[2] - l[0]) #find gradient
      if gradient >= -0.25 and gradient <= 0.25:
        for i in range(4):
          sumh[i] += l[i]
        sumh[4] += 1
      else:
        for i in range(4):
          sumv[i] += l[i]
        sumv[4] += 1
      print(linesPlen, sumh[4], sumv[4])
    if gradient >= -0.25 and gradient <= 0.25:
      cv2.line(frame, (int(sumh[0] /sumh[4]) , int(sumh[1] / sumh[4])), (int(sumh[2] / sumh[4]),int(sumh[3] / sumh[4])), (255, 0 ,0), 2, cv2.LINE_AA)
      print(sumh)
      sumh = [0, 0, 0, 0]
      
    else:
      cv2.line(frame, (int(sumv[0] /sumv[4]) , int(sumv[1] / sumv[4])), (int(sumv[2] / sumv[4]),int(sumv[3] / sumv[4])), (255, 0 ,0), 2, cv2.LINE_AA)
      print(sumv)
      sumv = [0, 0, 0, 0]"""
      
        
    #cv2.line(frame, (int(sumx1 /linesPlen) , int(sumy1 / linesPlen)), (int(sumx2 / linesPlen),int(sumy2 / linesPlen)), (255, 0 ,0), 2, cv2.LINE_AA) 
    #print(meanx, meany)
  
  lines = cv2.HoughLines(bluredges, 1, np.pi/180, 325, None, 0 ,0)
  linesax = []
  linesay = []
  linesbx = []
  linesby = []
  if lines is not None:
    for i in range(0, len(lines)):
      lineslist = [[0]]
      rho = lines[i][0][0]
      theta = lines[i][0][1]
      a = math.cos(theta)
      b = math.sin(theta)
      x0 = a * rho
      y0 = b * rho
      pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
      pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
      a = pt1
      b = pt2

      cv2.line(frame, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
      cv2.imshow("stuff", frame)
      
      #print("rho", rho, "   theta", theta)
      #print("a", pt1, "   b", pt2)
      horizontal = []
      
      if b[0] - a[0] == 0: #Horizontal
        gradient = 1
      else:
        gradient = (b[1] - a[1]) / (b[0] - a[0])
      if gradient > 1 :
        mean = int((b[0] + a[0]) / 2)
        compmean(mean, lineslist, b[1], a[1], True) #True is vertical
      else:
        mean = (b[1] + a[1]) / 2
        compmean(mean, lineslist, b[0], a[0], False) #False is horizontal
        
      
    for i in range((len(lineslist))-1):
      meansum = 0
      k = i + 1
      for j in range(len(lineslist[k]),(len(lineslist[k])-3)):
        meansum += lineslist[k][j]

      if lineslist[i][-1] is True:
        cv2.line(frame, (meansum, lineslist[k][-3]), (meansum, lineslist[k][-2]), (0,255,255), 3, cv2.LINE_AA)
      else:
        cv2.line(frame, (lineslist[k][-3],meansum), (lineslist[k][-2], meansum), (0,255,255), 3, cv2.LINE_AA)"""
  
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