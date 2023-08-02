import math
import numbers
from vector import Vector
import numpy as np

flatten_list = lambda y:[x for a in y for x in flatten_list(a)] if type(y) is list else [y]

class Matrix(object):
    """ A matrix with useful matrix operations that work with the vector class
    """

    def __init__(self, m, n):
        # Automatically return a 4x4 identity matrix
        self.m = m
        self.n = n
        self.rows = [[0]*n for x in range(m)]
    
    def getRank(self):
        return (self.m, self.n)
    
    def getTranspose(self):
        """Return the transpose of matrix without affecting the current matrix"""
        m, n = self.n, self.m
        mat = Matrix(m, n)
        mat.rows =  [list(item) for item in zip(*self.rows)]
        return mat
    
    def matrixMult(self, other):
        other_m, other_n = other.getRank()
        assert self.n == other_m, "number of columns in self must be equal to number of rows in other"

        other_t = other.getTranspose() # Because we can only work in rows
        out = Matrix(self.m, other_n)

        for x in range(self.m):
            for y in range(other_t.m):
                out[x][y] = sum([item[0]*item[1] for item in zip(self.rows[x], other_t[y])])
        return out
    
    def getDet():
        return 1
    
    # Currently uses numpy to do this, not sure if I need to implement myself?
    def getInverse(self):
        mat_np = np.array(self.rows)
        inv_np = np.linalg.inv(mat_np)
        inv = Matrix.fromList(inv_np.tolist())
        return inv

    # Overrides 
    def __getitem__(self, idx):
        return self.rows[idx]
    
    def __setitem__(self, idx, item):
        self.rows[idx] = item
    
    def __eq__(self, other):
        return (self.rows == other.rows)

    def __mul__(self, other):
        if type(other) == type(self):
            # multiply directly by other matrix")
            return self.matrixMult(other)

        elif type(other) == Vector:
            # convert vector to matrix then perform multiplication
            vectorAsMatrix = Matrix(len(other.components),1)
            for i in range(vectorAsMatrix.m):
                vectorAsMatrix[i][0] = other.components[i]
            outMatrix = self.matrixMult(vectorAsMatrix)
            # Convert the result back to a vector for further calculations
            # This is very simplified since we only have to work with vectors length 3 or 4

            out = Vector(*flatten_list(outMatrix.rows))
            #print(out.components)
            #if len(self.rows) == 4:
            #    out = Vector(outMatrix[0][0],outMatrix[1][0],outMatrix[2][0],outMatrix[3][0])
            #elif len(self.rows) == 3:
            #    out = Vector(outMatrix[0][0],outMatrix[1][0],outMatrix[2][0])
            return out

        elif isinstance(other, numbers.Real):
            out = Matrix(self.m, self.n)
            for x in range(self.m):
                for y in range(self.n):
                    out[x][y] = out[x][y] * other
            return out
        
    def __str__(self):
        s='\n'.join([' '.join([str(item) for item in row]) for row in self.rows])
        return s + '\n'
        
    # Class Methods
    @classmethod
    def Identity(cls, m):
        """ Generate identity matrix of rank (mxm) """
        rows = [[0]*m for x in range(m)]
        idx = 0 
        for row in rows:
            row[idx] = 1
            idx += 1

        return cls.fromList(rows)
    
    @classmethod
    def _makeMatrix(cls, rows):
        m = len(rows)
        n = len(rows[0])
        # Validity check
        assert any([len(row) == n for row in rows[1:]]), "inconsistent row length"
        mat = Matrix(m,n)
        mat.rows = rows

        return mat

    @classmethod
    def fromList(cls, listoflists):
        """ Create a matrix by directly passing a list
        of lists """
        rows = listoflists[:]
        return cls._makeMatrix(rows)


'''
a = Matrix(4,4)
print(a[0][3], a[3])
print(a.getRank())
a = Matrix.Identity(4)
for i in range(a.m):
    a[i][i] = 1 
print(str(a))

avector = Vector(1,3,4,1)

b = a * avector
print(b.components)

c = a.getInverse()
print(c)
'''
        

