import cv2 as cv
from cv2 import blur
import numpy as np
from .ImageFunctions import  FilterColorBlack
from .SquareFunctions import FindSquares

debug = False
showFinalImage = False

def GetArTag(image, limitCosine, thresholdBlockSize, minArea, maxArea, maxError):

    squares = []
    imageDebug = []
    xCenter = 0
    yCenter = 0

    reduceImages = FilterColorBlack(image)

    #blurImage = cv.GaussianBlur(reduceImages,(3,3),0,0)

    if debug:
        cv.imshow("debug",reduceImages)
        cv.waitKey(0)

    grayImage = cv.cvtColor(reduceImages, cv.COLOR_BGR2GRAY)

    if debug:
        cv.imshow("debug",grayImage)
        cv.waitKey(0)

    # apply morphology
    kernel = cv.getStructuringElement(cv.MORPH_RECT , (3,3))
    smooth = cv.morphologyEx(grayImage, cv.MORPH_DILATE, kernel)


    # divide gray by morphology image
    division = cv.divide(grayImage, smooth, scale=255)

    if debug:
        cv.imshow("morph",division)
        cv.waitKey(0)


    thresImage = cv.adaptiveThreshold(division,255,cv.ADAPTIVE_THRESH_MEAN_C,cv.THRESH_BINARY,thresholdBlockSize,0)

    if debug:
        cv.imshow("debug",thresImage)
        cv.waitKey(0)
    
    squares, xCenter, yCenter = FindSquares(thresImage, limitCosine, minArea,maxArea, maxError)

    if showFinalImage:
        imageDebug =  image.copy()
        print(squares)
        print (xCenter)
        print(yCenter)
        cv.drawContours(imageDebug, squares, -1, (255, 255, 0), 1)
        cv.circle(imageDebug, (xCenter, yCenter), 1, (255, 0, 255), -1)
        cv.imshow("final_image",imageDebug)
        cv.waitKey(0)

    return  squares, xCenter, yCenter
