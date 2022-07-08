from cmath import sqrt
import pandas as pd
import statistics as stat
import math
import numpy as np

class OnshapeReaderVector2D:
    def __init__(self, fileName, path):
        self.xname = "x"
        self.yname = "y"
        filePath = path + fileName
        self.csv = pd.read_csv(filePath)

        numData = len(self.csv)
        self.sideLength = int(np.sqrt(numData))

        self.processData()

    def processData(self):
        xData = self.csv[self.xname].values.tolist()
        yData = self.csv[self.yname].values.tolist()

        xData = np.array(xData).reshape(self.sideLength, self.sideLength)
        yData = np.array(yData).reshape(self.sideLength, self.sideLength)

        self.data = np.stack([xData, yData], axis=0)
        self.data = np.moveaxis(self.data, 0, -1)

    def getDataAt(self, x, y):
        return self.data[x, y, :]

class OnshapeReaderMatrix:
    def __init__(self, fileName, path):
        self.m00name = "m00"
        self.m10name = "m10"
        self.m01name = "m01"
        self.m11name = "m11"
        filePath = path + fileName
        self.csv = pd.read_csv(filePath)

        numData = len(self.csv)
        self.sideLength = int(np.sqrt(numData))

        self.processData()

    def processData(self):
        m00Data = self.csv[self.m00name].values.tolist()
        m10Data = self.csv[self.m10name].values.tolist()
        m01Data = self.csv[self.m01name].values.tolist()
        m11Data = self.csv[self.m11name].values.tolist()

        m00Data = np.array(m00Data).reshape(self.sideLength, self.sideLength)
        m10Data = np.array(m10Data).reshape(self.sideLength, self.sideLength)
        m01Data = np.array(m01Data).reshape(self.sideLength, self.sideLength)
        m11Data = np.array(m11Data).reshape(self.sideLength, self.sideLength)

        self.data = np.stack([m00Data, m10Data, m01Data, m11Data], axis=0)
        self.data = np.moveaxis(self.data, 0, -1)

    def getDataAt(self, x, y):
        vectorData = self.data[x, y, :] # swapped y, x. how?
        return vectorData.reshape(2,2)

if __name__ == "__main__":
    fileName = "FingerKinematicsOnshape.csv"
    path = "./"
    reader = OnshapeReaderVector2D(fileName, path)
    matrixReader = OnshapeReaderMatrix("FingerJacobianOnshape.csv", path)
    print(reader.getDataAt(0, 1))
    print(matrixReader.getDataAt(0,1))
