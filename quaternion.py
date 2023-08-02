import math
from math import sin, cos, tan
import numbers
from vector import Vector
import cmath
from matrix import Matrix

class Quaternion(object):
    """
    A quaternion with useful operations for reading IMU data
    """
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x 
        self.y = y
        self.z = z
    
    def asList(self):
        return [self.w,self.x,self.y,self.z]
    
    def norm(self):
        """ Returns the norm (magnitude) of the quaternion"""
        return math.sqrt(sum(math.pow(num, 2) for num in self.asList()))
    
    def normalize(self):
        """ Returns a normalized unit quaternion from this quaternion."""
        magnitude = self.norm() if self.norm() != 0 else 1
        return Quaternion.fromList([num/magnitude for num in self.asList()])

    # (Part 3)
    def getConjugate(self):
        """ Returns the conjugate (inverse) of the quaternion without affecting the current quaternion."""
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    # (Part 2)
    def getEulerAngles(self):
        """ Returns a set of euler angles in the form Vector(X,Y,Z)"""
        q = self
        eulerAngles = Vector(0,0,0,0)
        # x rotation
        sinxcosy = 2 * (q.w * q.x + q.y * q.z)
        cosxcosy = 1 - 2 * (q.x * q.x + q.y * q.y)
        eulerAngles.x = math.atan2(sinxcosy, cosxcosy)

        # y rotation
        siny = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosy = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        eulerAngles.y = 2 * math.atan2(siny, cosy) - math.pi / 2

        # z rotation
        sinzcosy = 2 * (q.w * q.z + q.x * q.y)
        coszcosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        eulerAngles.z = math.atan2(sinzcosy, coszcosy)

        return eulerAngles
    
    def quaternionMult(self, other):
        assert type(other) == Quaternion, "other is not a quaternion"
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z

        wOut = (w1*w2)-(x1*x2)-(y1*y2)-(z1*z2)
        xOut = (w1*x2)+(x1*w2)+(y1*z2)-(z1*y2)
        yOut = (w1*y2)+(y1*w2)+(z1*x2)-(x1*z2)
        zOut = (w1*z2)+(z1*w2)+(x1*y2)-(y1*x2)

        return Quaternion(wOut, xOut, yOut, zOut)
    
    def getRotationMatrix(self):
        """ Returns the rotation matrix corresponding to this quaternion"""
        q = self
        r00 = 2 * (q.w**2 + q.x**2) - 1
        r01 = 2 * (q.x*q.y - q.w*q.z)
        r02 = 2 * (q.x * q.z + q.w * q.y)

        r10 = 2 * (q.x * q.y + q.w * q.z)
        r11 = 2 * (q.w * q.w + q.y * q.y) - 1
        r12 = 2 * (q.y * q.z - q.w * q.x)
        
        r20 = 2 * (q.x * q.z - q.w * q.y)
        r21 = 2 * (q.y * q.z + q.w * q.x)
        r22 = 2 * (q.w * q.w + q.z * q.z) - 1

        rMatrixList = [
            [r00,r01,r02,0],
            [r10,r11,r12,0],
            [r20,r21,r22,0],
            [0,0,0,1]
        ]

        rotationMatrix = Matrix.fromList(rMatrixList)
        return rotationMatrix
    # Overrides
    def __str__(self):
        return str(self.asList())
    
    def __eq__(self, other):
        return (self.asList() == other.asList())
    
    # (Part 4)
    def __mul__(self, other):
        """ multiply this quaternion with another (non-commutative)"""
        assert(type(other) == Quaternion), "Only quaternion by quaternion multiplication implemented"
        return self.quaternionMult(other)
    
    def __add__(self, other):
        assert(type(other) == Quaternion), "Only quaternion + quaternion implemented"
        return Quaternion(self.w+other.w, self.x+other.x, self.y+other.y, self.z+other.z)
    
    # Class Methods
    @classmethod
    def Identity(cls):
        """ Generate identity quaternion [1,0,0,0] """
        return Quaternion(1,0,0,0)

    @classmethod
    def fromList(cls, quaternionList):
        """ Create a quaternion by directly passing a list in the form [w,x,y,z] """
        return Quaternion(*quaternionList)

    @classmethod
    def fromRotationMatrix(cls, rotationMatrix):
        """ Create a quaternion by using a predefined rotation matrix """
        return Quaternion.Identity()
    
    # (Part 1)
    @classmethod
    def fromEulerAngles(cls, eulerAngles: Vector):
        """ Convert a set of euler angles in the form Vector(X,Y,Z) into a quaternion """
        assert len(eulerAngles) >= 3, "there must be 3 euler angles"
        angleX = eulerAngles.x
        angleY = eulerAngles.y
        angleZ = eulerAngles.z

        cx = cos(angleX * 0.5)
        sx = sin(angleX * 0.5)
        cy = cos(angleY * 0.5)
        sy = sin(angleY * 0.5)
        cz = cos(angleZ * 0.5)
        sz = sin(angleZ * 0.5)
        
        out = Quaternion.Identity()
        out.w = (cx * cy * cz) + (sx * sy * sz)
        out.x = (sx * cy * cz) - (cx * sy * sz)
        out.y = (cx * sy * cz) + (sx * cy * sz)
        out.z = (cx * cy * sz) - (sx * sy * cz)

        return out
    
    @classmethod
    def fromAxisAngleRepresentation(cls, axisVector: Vector, angle: float):
        """ Convert an axis-angle representation into a unit quaternion """
        w = cos(angle/2)
        x = axisVector.x * sin(angle/2)
        y = axisVector.y * sin(angle/2)
        z = axisVector.z * sin(angle/2)
        return Quaternion(w,x,y,z)


