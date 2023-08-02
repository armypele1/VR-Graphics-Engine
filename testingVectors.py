from vector import Vector

myVector = Vector(1,2,3,4)
identityMatrix = [[1,1,1,1],[0,2,0,0],[0,0,2,0],[0,0,0,2]]
identityMatrix = [Vector(*x) for x in identityMatrix]
print(myVector.components)

myNewVector = myVector.matrixMult(identityMatrix)
print(myNewVector.components)