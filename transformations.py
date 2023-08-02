from vector import Vector
from matrix import Matrix
import math
from math import sin, cos , tan

def getRotationMatrix(angleX = 0, angleY = 0, angleZ = 0, useRadians=False):
    """Returns a matrix for rotations in all 3 dimensions using the specified angles (autos to degrees)"""
    if not useRadians:
        angleX = math.radians(angleX)
        angleY = math.radians(angleY)
        angleZ = math.radians(angleZ)

    xRotationMatrixList = [
    [1,0,0,0],
    [0,cos(angleX),-sin(angleX),0],
    [0, sin(angleX), cos(angleX), 0],
    [0,0,0,1]
    ]
    xRotationMatrix = Matrix.fromList(xRotationMatrixList)

    yRotationMatrixList = [
    [cos(angleY),0,sin(angleY),0],
    [0,1,0,0],
    [-sin(angleY), 0, cos(angleY), 0],
    [0,0,0,1]
    ]
    yRotationMatrix = Matrix.fromList(yRotationMatrixList)

    zRotationMatrixList = [
    [cos(angleZ),-sin(angleZ),0,0],
    [sin(angleZ),cos(angleZ),0,0],
    [0,0,1,0],
    [0,0,0,1]
    ]
    zRotationMatrix = Matrix.fromList(zRotationMatrixList)

    fullRotationMatrix = xRotationMatrix * yRotationMatrix * zRotationMatrix

    return fullRotationMatrix

def getScaleMatrix(scaleFactor):
    """Returns a matrix for scaling using a provided scale factor parameter"""
    scaleMatrixList = [
        [scaleFactor,0,0,0],
        [0,scaleFactor,0,0],
        [0,0,scaleFactor,0],
        [0,0,0,1]
    ]
    scaleMatrix = Matrix.fromList(scaleMatrixList)
    return scaleMatrix

def getTranslationMatrix(deltaX, deltaY, deltaZ):
    translationMatrixList = [
        [1,0,0,deltaX],
        [0,1,0,deltaY],
        [0,0,1,deltaZ],
        [0,0,0,1]
    ]
    translationMatrix = Matrix.fromList(translationMatrixList)
    return translationMatrix

def getProjectionMatrix(nearPlane, farPlane):
    n, f = nearPlane, farPlane # for easier reading of matrix
    projectionMatrixList = [
        [n,0,0,0],
        [0,n,0,0],
        [0,0,(n+f),(-1 * f * n)],
        [0,0,1,0]
    ]
    projectionMatrix = Matrix.fromList(projectionMatrixList)
    return projectionMatrix

def getTSTMatrix(right, left, top, bottom, nearPlane, farPlane):
    r,l,t,b,n,f = right, left, top, bottom, nearPlane, farPlane
    TSTMatrixList = [
        [(2/(r-l)),0,0,0],
        [0,(2/(t-b)),0,0],
        [0,0,(2/(n-f)),(-1 * ((n+f)/(n-f)))],
        [0,0,0,1]
    ]
    TSTMatrix = Matrix.fromList(TSTMatrixList)
    return TSTMatrix

def getViewportMatrix(m, n):
    VPList = [
        [m/2,0,0,(m-1)/2],
        [0,n/2,0,(n-1)/2],
        [0,0,1,0],
        [0,0,0,1]
    ]
    VPMatrix = Matrix.fromList(VPList)
    return VPMatrix