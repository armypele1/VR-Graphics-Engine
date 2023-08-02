import pandas as pd
import math
import time
from vector import Vector
from quaternion import Quaternion
from transformations import getRotationMatrix

IMURotation = getRotationMatrix(90,0,0)

# Get the IMU data in a suitable format
def getCleanedCSV():
    df = pd.read_csv("./data/IMUData.csv")
    df.columns = df.columns.str.replace(' ', '')
    axes = ["X","Y","Z"]

    # Remove accelerometer readings that 

    # Normalize accelerometer and magnetometer readings
    df["accelerometer.X"],df["accelerometer.Y"],df["accelerometer.Z"] = normalizeColumns(df["accelerometer.X"],df["accelerometer.Y"],df["accelerometer.Z"])
    df["magnetometer.X"],df["magnetometer.Y"],df["magnetometer.Z"] = normalizeColumns(df["magnetometer.X"],df["magnetometer.Y"],df["magnetometer.Z"])

    # Convert gyroscope readings to radians/sec
    axes = ["X","Y","Z"]
    for axis in axes:
        df[("gyroscope."+axis)] = df[("gyroscope."+axis)].apply(lambda x: math.radians(x)) # Convert to radians/s
    
    # Finally perform rotation on gyro and acc to have the axis be the correct orientation
    df["accelerometer.X"],df["accelerometer.Y"],df["accelerometer.Z"] = correctAxisOrientation(df["accelerometer.X"],df["accelerometer.Y"],df["accelerometer.Z"])
    df["gyroscope.X"],df["gyroscope.Y"],df["gyroscope.Z"] = correctAxisOrientation(df["gyroscope.X"],df["gyroscope.Y"],df["gyroscope.Z"])

    df = removeLargeDiffs(df)

    return df

# Correct the orientation of the axis s.t. Y is the up vector instead of Z
def correctAxisOrientation(X,Y,Z):
    for i in range(len(X)):
        vector = Vector(X[i],Y[i],Z[i],0)
        vector = IMURotation * vector
        X[i], Y[i], Z[i] = vector.x, vector.y, vector.z
    return X,Y,Z

# Use to answer q2.2
def normalizeColumns(X,Y,Z):
    for i in range(len(X)):
        vector = Vector(X[i],Y[i],Z[i])
        normalized = vector.normalize() # this function handles the division by 0 edge case
        X[i], Y[i], Z[i] = normalized.x, normalized.y, normalized.z
    return X,Y,Z

def gyroIntegration(q, deltatime, w_hat):
    l = w_hat.norm()    
    v = w_hat * (1/l)
    angle = l * deltatime
    updater = Quaternion.fromAxisAngleRepresentation(v, angle)
    qNext = q * updater
    return qNext

# Remove any spikes in acceleration data likely due to object movement that would skew the estimated gravity vector
def removeLargeDiffs(df, thresholdAngle=0.1):
    previousAcc = Vector(df["accelerometer.X"][0],df["accelerometer.Y"][0],df["accelerometer.Z"][0])
    for i in range(1, len(df["accelerometer.X"])-1):
        currentAcc = Vector(df["accelerometer.X"][i],df["accelerometer.Y"][i],df["accelerometer.Z"][i])
        nextAcc = Vector(df["accelerometer.X"][i+1],df["accelerometer.Y"][i+1],df["accelerometer.Z"][i+1])

        angleDiffPrevious = currentAcc.angleBetweenVectors(previousAcc)
        angleDiffNext = nextAcc.angleBetweenVectors(currentAcc)

        tempX, tempY, tempZ = df["accelerometer.X"][i],df["accelerometer.Y"][i],df["accelerometer.Z"][i]

        if abs(angleDiffPrevious) > thresholdAngle and abs(angleDiffNext) > thresholdAngle:
           df["accelerometer.X"][i],df["accelerometer.Y"][i],df["accelerometer.Z"][i] = df["accelerometer.X"][i-1],df["accelerometer.Y"][i-1],df["accelerometer.Z"][i-1]
        
        previousAcc = Vector(tempX, tempY, tempZ)
    
    return df

# Find average of range of 2n values where index is at centre
def getAverageAcceleration(index, df, n):
    startFlag = max(0, index - n)
    endFlag = min(len(df.index), index + n)
    avgAccX = sum(df["accelerometer.X"][startFlag:endFlag]) / (endFlag - startFlag)
    avgAccY = sum(df["accelerometer.Y"][startFlag:endFlag]) / (endFlag - startFlag)
    avgAccZ = sum(df["accelerometer.Z"][startFlag:endFlag]) / (endFlag - startFlag)

    return Vector(avgAccX, avgAccY, avgAccZ)

def tiltCorrection(q, index, df, alpha, n=100):
    avgAcc = getAverageAcceleration(index, df, n)
    a_euler = getGlobalFrameAcceleration(q, avgAcc).getEulerAngles()
    
    a_hat = Vector(a_euler.x,a_euler.y,a_euler.z) 

    t = Vector(a_hat.z, 0, -a_hat.x)
    up = Vector(0,1,0)
    phi = a_hat.angleBetweenVectors(up)
    tiltCorrect = Quaternion.fromAxisAngleRepresentation(t, (-1*alpha*phi))
    q = q * tiltCorrect
    return q

# Perform gyro intergration and tilt correction to get next rotation of the headset from the IMU data
def getNextRotation(q, deltatime, index, df):
    w_hat = Vector(df[("gyroscope.X")][index],df[("gyroscope.Y")][index],df[("gyroscope.Z")][index])
    qNext = gyroIntegration(q, deltatime, w_hat)
    #qNext = tiltCorrection(qNext, index, df, 0.01)
    euler = q.getEulerAngles()
    return qNext, euler

def getGlobalFrameAcceleration(q, localEulerAngles):
    locala =  Quaternion.fromEulerAngles(Vector(*localEulerAngles.components))
    globala = q.getConjugate() * locala * q
    return globala


