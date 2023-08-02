from image import Image, Color
from model import Model
from shape import Point, Line, Triangle
from vector import Vector
from matrix import Matrix
from quaternion import Quaternion
from camera import Camera
import cv2
import numpy as np
import time
import math
from math import sin, cos, tan #for easier reading
import sys
from transformations import getTranslationMatrix, getRotationMatrix, getScaleMatrix, getProjectionMatrix, getTSTMatrix, getViewportMatrix
import gc
import copy
import dataManager
import physicsEngine
import random

#region (1) Initialization

# init image and depth buffer
width = 400
height = 400
aspectRatio = width / height
baseImage = Image(width, height, Color(255,255,255,255))
zBuffer = [-float('inf')] * width * height
image = baseImage

# init camera
cam = Camera(Point(0,0,-1), Vector(0,0,0,0), Vector(0,1,0,0), 80) # fov auto sets to 100

cam.lookAtPoint(Point(0,0,0))
print("Loading...")

# Load in all models used in the scene using info we specify when defining their physics object counterpart
def loadModelUsingObjectData(physicsObj):
    model = Model('data/headset.obj')
    model.normalizeGeometry()
    modelData = {"model":model, "vertexNormals":None, "rotation":Vector(0,0,0), "translation":physicsObj.pos, "scale":physicsObj.scale}
    return modelData

# Initialize all physics objects in the scene
allObjects = []
object1 = physicsEngine.PhysicsObject(initialPos=Vector(3,1,4), initialVelocity=Vector(-4,0,0), useGravity=True)
object2 = physicsEngine.PhysicsObject(initialPos=Vector(-3,1,4), initialVelocity=Vector(4,0,0), useGravity=True)
object3 = physicsEngine.PhysicsObject(initialPos=Vector(0,5,7), initialVelocity=Vector(0,0,0), useGravity=True)
object4 = physicsEngine.PhysicsObject(initialPos=Vector(0,5,-7), initialVelocity=Vector(0,0,0), useGravity=False)
object5 = physicsEngine.PhysicsObject(initialPos=Vector(8,6,0), initialVelocity=Vector(0,0,0), useGravity=False)
object6 = physicsEngine.PhysicsObject(initialPos=Vector(-8,5,2), initialVelocity=Vector(0,0,0), useGravity=False)
object7 = physicsEngine.PhysicsObject(initialPos=Vector(-5,6,0), initialVelocity=Vector(-0.5,0,0), useGravity=False)
allObjects.append(object1)
allObjects.append(object2)
allObjects.append(object3)
allObjects.append(object4)
#allObjects.append(object5)
#allObjects.append(object6)
allObjects.append(object7)

# Initialize all models associated withe each physics object
allModelData = []
for i in range(len(allObjects)):
    modelData = loadModelUsingObjectData(allObjects[i])
    modelData["rotation"] = Vector(random.randint(0,90), random.randint(0,180), random.randint(0,90))
    allModelData.append(modelData)  
    

# Define symmetric viewing frustum
nearPlane = 0.01
farPlane = 20
top = nearPlane * math.tan((math.radians(cam.fov) / 2))
bottom = -top
right = top * aspectRatio
left = -right

# Load in all of the transformation matrices we will be using throughout the rendering process
viewMatrix = cam.getViewMatrix()
projectionMatrix = getProjectionMatrix(nearPlane, farPlane)
TSTMatrix = getTSTMatrix(right, left, top, bottom, nearPlane, farPlane)

translationMatrix = getTranslationMatrix(0,0,0)
rotationMatrix = getRotationMatrix(0,0,0)
scaleMatrix = getScaleMatrix(1)

#endregion

#region (2) Perform pre-calculations for every model
def getVertexNormal(vertIndex, faceNormalsByVertex):
    # Compute vertex normals by averaging the normals of adjacent faces
    normal = Vector(0, 0, 0, 0)
    for adjNormal in faceNormalsByVertex[vertIndex]:
        normal = normal + adjNormal

    return normal / len(faceNormalsByVertex[vertIndex])

for i in range(len(allModelData)):
    model = allModelData[i]["model"]
    # Calculate face normals of the model
    faceNormals = {}
    for face in model.faces:
        p0, p1, p2 = [model.vertices[i] for i in face]
        faceNormal = (p2-p0).cross(p1-p0).normalize()

        for j in face:
            if not j in faceNormals:
                faceNormals[j] = []
            
            faceNormals[j].append(faceNormal)
    
    # Using the face normals, calculate the vertex normals
    vertexNormals = []
    for vertIndex in range(len(model.vertices)):
        vertNorm = getVertexNormal(vertIndex, faceNormals)
        vertexNormals.append(vertNorm)
    
    allModelData[i]["vertexNormals"] = vertexNormals

#endregion

#region (3) Get all IMU Data and perform related calculations for setup
df = dataManager.getCleanedCSV()
frameCount = 0
dt = 1/256 # This is the refresh rate of the sensors

q = Quaternion.Identity()
r = q.getRotationMatrix()

# We define an initial look orientation so we can start from a specified viewpoint and apply rotations to that viewpoint
initialLookOrientation = cam.lookAt
initialUpOrientation = cam.up

# output the intial camera view direction
print([math.degrees(comp) for comp in cam.lookAt.components])

#endregion

print("Done!")
#region (4) Perform all the frame-by-frame calculations
while True:
    # Reset image and depth buffer
    image =  copy.deepcopy(baseImage)
    zBuffer = [-float('inf')] * width * height

    # Gyro Integration with gravity based tilt correction
    q, q_angles = dataManager.getNextRotation(q, dt, frameCount, df)
    
    camRotation = q.getRotationMatrix()

    cam.lookAt = camRotation * initialLookOrientation 
    cam.up = camRotation * initialUpOrientation 

    ''' Testing camera rotation without IMU data
    eulerAngles = Vector(0,10,0)
    rotationMatrix = getRotationMatrix(eulerAngles.x,eulerAngles.y,eulerAngles.z)
    cam.lookAt = rotationMatrix * cam.lookAt
    cam.up = rotationMatrix * cam.up
    #'''
    viewMatrix = cam.getViewMatrix()
    frameCount += 1

    # Perform all physics calculations
    for i in range(len(allObjects)):
        # Check for collision
        for j in range(len(allObjects)):
            if j != i:
                allObjects[i].checkCollision(allObjects[j])
        
        allObjects[i].applyGravity(dt)
        allObjects[i].pos += allObjects[i].velocity * dt

        # If headset gets too low reset its height so we can have continuous falling headsets
        if allObjects[i].pos.y < -10:
            allObjects[i].pos = Vector(allObjects[i].pos.x, 10, allObjects[i].pos.z)
            allObjects[i].velocity = Vector(allObjects[i].velocity.x,0,allObjects[i].velocity.z)

    #'''
    # iterate over all models in the scene, rendering each one
    for i in range(len(allModelData)):
        #print(i)
        modelData = allModelData[i]
        model = modelData["model"]

        modelData["translation"] = allObjects[i].pos

        # Rotate all models about the y axis
        modelData["rotation"].y += 0.5
        
        # Create the modelview matrix
        scale = getScaleMatrix(modelData["scale"])
        translate = getTranslationMatrix(modelData["translation"].x,modelData["translation"].y,modelData["translation"].z)
        rotate = getRotationMatrix(modelData["rotation"].x, modelData["rotation"].y, modelData["rotation"].z)

        modelMatrix = scale * translate * rotate
        modelViewMatrix = viewMatrix * modelMatrix

        for face in model.faces:
            p0, p1, p2 = [model.vertices[i] for i in face]
            n0, n1, n2 = [modelData["vertexNormals"][i] for i in face]

            # Define the light direction
            lightDir = Vector(0, 0, 1, 0)
            lightDir = lightDir.normalize()
            #lightDir = modelViewMatrix

            # Set to true if face should be culled
            cull = False

            # Transform vertices and calculate lighting intensity per vertex
            transformedPoints = []
            viewSpacePoints = []
            lightLevels = []
            t0 = time.time()

            points = [p0, p1, p2]
            normals = [n0, n1, n2]

            # Transform to view space
            for i in range(len(points)):
                p = points[i]
                n = normals[i]

                p = modelViewMatrix * p
                z = p.z
                viewSpacePoints.append(p)

                n = modelViewMatrix * n
                n.w = 0
                n = n.normalize()
                # Calculate light intensity
                intensity = n * lightDir
                if intensity < 0:
                    intensity = 0
                lightLevels.append(intensity)
            
            p0, p1, p2 = viewSpacePoints
            
            # Backface Culling in view space 
            transformedFaceNormal = (p2-p0).cross(p1-p0).normalize()
            if (p0 * -1) * transformedFaceNormal >= 0:
                continue # We want to skip rendering this face
            
            outsideFrustumCount = 0
            # If we get here the vertex doesn't need to be culled, so continue transforms
            for i in range(len(viewSpacePoints)):
                p = viewSpacePoints[i]
                p = projectionMatrix * p
                p = TSTMatrix * p
                if p.w != 0:
                    p.x = p.x / p.w
                    p.y = p.y / p.w
                    p.z = p.z / p.w
                
                # Perform frustum culling in clip space
                if p.x < -1 or p.x > 1 or p.y < -1 or p.y > 1 or z < nearPlane or z > farPlane:
                    outsideFrustumCount += 1

                # Apply distortion pre-correction
                ru = math.sqrt(p.x**2 + p.y**2) # undistorted radius
                c1 = 0.2
                c2 = 0.2
                rd = ru + c1 * ru**3 + c2 * ru**5

                correctionFactor = ru / rd
                p.x *= correctionFactor
                p.y *= correctionFactor
                
                xWindow = int((width / 2) * (p.x + 1))
                yWindow = int((height / 2) * (p.y + 1))
                zWindow = (1 / 2) * (p.z + 1)
                transformedPoints.append(Point(xWindow, yWindow, zWindow, Color(lightLevels[i]*255, lightLevels[i]*255, lightLevels[i]*255, 255)))

            if outsideFrustumCount == len(points):
                continue # We want to skip rendering this face

            if not cull:
                Triangle(transformedPoints[0], transformedPoints[1], transformedPoints[2]).draw(image, zBuffer)
    #'''
#endregion
#'''
#region (5) Output resulting frame buffer to a cv2 window. Then, save final frame to a png when user closes program
    img_array = np.zeros((height, width, 4), dtype=np.uint8)

    j = 0
    for i in range(height):
        # Convert each row of the image buffer to a format suitable for opencv
        # Note that the reversal is because opencv works in BGR instead of RGB
        img_array[i] = np.frombuffer(image.buffer, dtype=np.uint8)[j:(j + width*4)][::-1].reshape(width, 4)
        j += width*4 + 1
    
    cv2.imshow('image', img_array)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
image.saveAsPNG("image1.png")
#endregion
#'''