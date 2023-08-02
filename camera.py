from vector import Vector
from matrix import Matrix
from shape import Point

class Camera(object):
    def __init__(self, pos: Point, lookAt: Vector, up: Vector, fov=100):
        self.pos  = pos
        self.lookAt = lookAt
        self.up = up
        self.fov = fov
            
    def lookAtPoint(self, p):
        # Use Formula 3.37 laValle to get vector pointing towards given point
        e = self.pos
        c = Vector((e.x-p.x),(e.y-p.y),(e.z-p.z))
        c = c.normalize()

        self.lookAt = Vector(c.x,c.y,c.z,0)
    
    def getViewMatrix(self):
        x,y,z = self.pos.x, self.pos.y, self.pos.z
        
        # Calculated using formula in LaValle coursebook
        zhat = self.lookAt * -1
        xhat = self.up.cross(zhat)
        yhat = zhat.cross(xhat)

        RList = [
            [xhat.x, xhat.y, xhat.z, 0],
            [yhat.x, yhat.y, yhat.z, 0],
            [zhat.x, zhat.y, zhat.z, 0],
            [0,0,0,1]
        ]
        eyeList = [
            [1,0,0,-x],
            [0,1,0,-y],
            [0,0,1,-z],
            [0,0,0,1]
        ]

        RMatrix = Matrix.fromList(RList)
        eyeMatrix = Matrix.fromList(eyeList)
        viewTransform = RMatrix * eyeMatrix

        return viewTransform

