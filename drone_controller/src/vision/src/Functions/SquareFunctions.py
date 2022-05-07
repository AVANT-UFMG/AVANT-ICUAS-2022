import cv2 as cv
import math

def AngleCornerPointsCos( b, c, a):
    dx1 = abs(b[0][0] - a[0][0])
    dy1 = abs(b[0][-1] - a[0][-1])
    dx2 = abs(c[0][0] - a[0][0])
    dy2 = abs(c[0][-1] - a[0][-1])

    values = float(dx1*dx1 + dy1*dy1)*float(dx2*dx2 + dy2*dy2)
    values2 = (dx1*dx2 + dy1*dy2)
    return values2 / math.sqrt( values + 1e-12);

def sideRectangle(points):

    a = ((((points[0][0][0] - points[1][0][0]) ** 2) + ((points[0][-1][-1] - points[1][-1][-1])**2))**(1/2))
    b = ((((points[0][0][0] - points[2][0][0]) ** 2) + ((points[0][-1][-1] - points[2][-1][-1])**2))**(1/2))
    c = ((((points[0][0][0] - points[3][0][0]) ** 2) + ((points[0][-1][-1] - points[3][-1][-1])**2))**(1/2))
    if a>b and a>c:
            return [b,c]
    elif b>c:
            return [a,c]
    else:
            return [a,b]

def proporcionalRectangle(point):
    dimens = sideRectangle(point)
    return dimens[0]/dimens[1] > 0.65 and dimens[0]/dimens[1] < 1.5  
    #and dimens[1]*dimens[0] >100

def CheckSquare(contours, x,y):
    inside = False
    for contour in contours:
        result = cv.pointPolygonTest(contour, (x,y), False)
        if result == 1:
            inside = True
            break
    
    return inside

def FindSquares(thresh, limitCosine, minArea, maxArea,  maxError):

    squares = list()
    countourToAnalise = list()
    cX = 0
    cY = 0
    area = 0
    countours, _ = cv.findContours(thresh,cv.RETR_LIST,cv.CHAIN_APPROX_SIMPLE)

    for contour in countours:

        epsilon =  cv.arcLength(contour, True) * maxError
        aprox = cv.approxPolyDP(contour,epsilon, True)

        area = float(cv.contourArea(aprox))

        if (len(aprox) == 4 and area > minArea and area < maxArea and cv.isContourConvex(aprox)):
            print("int")
            maxCosine = 0

            for j in range(2,4):
                cosine = float(AngleCornerPointsCos(aprox[j%4], aprox[j-2], aprox[j-1]))
                maxCosine = max(maxCosine, cosine)
            
            if  proporcionalRectangle(aprox): #and  maxCosine < float(limitCosine)
                M = cv.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                if not CheckSquare(countourToAnalise,cX, cY ): 
                    squares.append(aprox)
                    countourToAnalise.append(contour)
            
    return squares, cX, cY
