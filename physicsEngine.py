import math
from vector import Vector

GRAVITY = -9.81
AIR_DENSITY = 1.3

class PhysicsObject(object):
    """
    The physics based counterpart to all models in the scene, allowing for gravity based movement and simple collision detection
    """
    def __init__(self, objectScale=1, initialPos=Vector(0,0,0), initialVelocity=Vector(0,0,0), mass=1, dragCoeff=0.5, area=0.2, useGravity=True):
        self.scale = objectScale
        self.radius = self.setCollisionRadius()
        self.pos = initialPos
        self.velocity = initialVelocity
        self.drag = Vector(0,0,0)
        self.mass = mass
        self.weight = self.getWeightVector(mass)
        self.dragCoeff = dragCoeff
        self.area = area
        self.useGravity = useGravity

    def getWeightVector(self, objectMass):
        return Vector(0,objectMass * GRAVITY,0)
    
    def setWeight(self):
        """Set weight appropriately depending on whether object uses gravity"""
        self.weight = self.getWeightVector(self.mass) if self.useGravity else Vector(0,0,0)
    
    def getDragForOneAxis(self, currentVelocityAlongOneAxis):
        # Ensure drag is always acting AGAINST current direction of travel
        dragCoeff = -self.dragCoeff if currentVelocityAlongOneAxis < 0 else self.dragCoeff
        return dragCoeff * ((AIR_DENSITY * currentVelocityAlongOneAxis**2) / 2) * self.area

    def setDrag(self):
        xDrag = self.getDragForOneAxis(self.velocity.x)
        yDrag = self.getDragForOneAxis(self.velocity.y)
        zDrag = self.getDragForOneAxis(self.velocity.z)
        self.drag = Vector(xDrag, yDrag, zDrag)
    
    def getAcceleration(self):
        return (self.weight - self.drag) / self.mass
    
    def applyGravity(self, dt):
        """ Apply net force to the object with respect to delatime. Call after all physics events have occurred"""
        self.setWeight()
        self.setDrag()
        a = self.getAcceleration()
        self.velocity = self.velocity + a * dt
    
    def applyImpulse(self, impulseForce: Vector):
        """ Apply a force instantaneously to an object."""
        self.velocity += impulseForce
    
    def setCollisionRadius(self):
        return self.scale * 1
        

    def checkCollision(self, other):
        """Check for a collision between this object and another"""
        if (abs(self.pos.x-other.pos.x)) < (self.radius + other.radius) and (abs(self.pos.y-other.pos.y)) < (self.radius + other.radius) and (abs(self.pos.z-other.pos.z)) < (self.radius + other.radius):
            print("collision!")
            self.handleCollision(other)
    
    def handleCollision(self, other):
        # Find normalized direction vector from self origin
        a = self.pos
        b = other.pos

        # Get line of collision and normalize
        collisionNormal = Vector((b.x-a.x),(b.y-a.y),(b.z-a.z)).normalize()

        # Get the point of "most" intersection for each bounding sphere
        intersectPointA = a + collisionNormal * self.radius
        intersectPointB = b - collisionNormal * other.radius

        # find the distance between these two points
        d = math.sqrt(math.pow(intersectPointB.x-intersectPointA.x, 2) + math.pow(intersectPointB.y-intersectPointA.y, 2) + math.pow(intersectPointB.z-intersectPointA.z, 2))

        # Determine the total velocity of the collision
        relativeVelocity = self.velocity - other.velocity

        e = 1
        totalCollisionVelocity = (relativeVelocity * -1 * (1+e)) * collisionNormal

        # Use this to define an impulse force
        impulse = totalCollisionVelocity / (1/self.mass + 1/other.mass)

        # Apply this impulse to the objects to determine their new velocities
        self.velocity = self.velocity + collisionNormal * impulse * 1/self.mass
        other.velocity = other.velocity - collisionNormal * impulse * 1/self.mass




