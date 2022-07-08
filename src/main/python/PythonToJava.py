from OnshapeToPython import OnshapeReaderMatrix, OnshapeReaderVector2D

class VectorToJava:
    def __init__(self, reader : OnshapeReaderVector2D):
        self.reader = reader
        self.className = "FingerVectorLookupTable"

    def makeJavaFile(self, filename):
        pass

class MatrixToJava:
    def __init__(self, reader : OnshapeReaderMatrix):
        self.reader = reader
        self.className = "FingerJacobianLookupTable"

    def makeJavaFile(self, filename):
        pass