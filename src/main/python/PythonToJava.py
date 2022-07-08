from OnshapeToPython import OnshapeReaderMatrix, OnshapeReaderVector2D

# These just create a 2d java array. 
# dim1 is input1, dim2 is input2

class PrinterTools:
    def printLine(self, numTabs, message):
        out = ""
        out += self.printTab(numTabs)
        out += message
        out += "\n"
        return out

    def printTab(self, n):
        out = ""
        for i in range(n):
            out += "\t"
        return out

    def printWhiteSpace(self, n):
        out = ""
        for i in range(n):
            out += "\n"
        return out

class VectorToJava(PrinterTools):
    def __init__(self, path, csvName):
        self.reader = OnshapeReaderVector2D(csvName, path)
        self.path = path
        self.packageName = "yu.evan.finger.kinematics"
        self.className = "DiscreteLookupTable2DKinematicsV1"
        self.storageClassName = "ArithmeticVector2D"
        self.maxIndex = self.reader.sideLength

    def makeJavaFile(self):
        with open(self.path + self.className + ".java", "w") as outputFile:
            outputFile.write(self.printLine(0, "package " + self.packageName + ";"))
            outputFile.write("\n")
            outputFile.write(self.printLine(0, "public class " + self.className + " implements DiscreteLookupTable2D<" + self.storageClassName + ">"))
            outputFile.write(self.printLine(0, "{"))
            outputFile.write(self.printConstructor())
            outputFile.write("\n")
            outputFile.write(self.printSizeFields())
            outputFile.write("\n")
            outputFile.write(self.printAccessorMethod())
            outputFile.write("\n")
            outputFile.write(self.printDataTable())
            outputFile.write(self.printLine(0, "}"))
    
    def printConstructor(self):
        out = ""
        out += self.printLine(1, "public " + self.className + "() {}")
        return out;

    def printSizeFields(self):
        out = ""
        out += self.printLine(1, "private int numDivisions = " + str(self.maxIndex - 1) + ";")
        out += self.printLine(1, "private int maxIndex = " + str(self.maxIndex) + ";")
        out += "\n"
        out += self.printLine(1, "public int getNumDivisions()")
        out += self.printLine(1, "{")
        out += self.printLine(2, "return numDivisions;")
        out += self.printLine(1, "}")
        return out
        
    def printAccessorMethod(self):
        out = ""
        out += self.printLine(1, "public " + self.storageClassName + " getValueAtIndex(int i0, int i1)")
        out += self.printLine(1, "{")
        out += self.printLine(2, "return dataTable[i0][i1];")
        out += self.printLine(1, "}")
        return out

    def printElement(self, vector):
        return "new " + self.storageClassName + "(" + "{:.19f}".format(vector[0]) + ", " + "{:.19f}".format(vector[1]) + ")"

    def printDataTable(self):
        out = ""
        out += self.printLine(1, "private " + self.storageClassName + "[][] dataTable =")
        out += self.printLine(2, "{")
        for i1 in range(self.maxIndex):
            line = ""
            line += "{"
            for i2 in range(self.maxIndex):
                line += self.printElement(self.reader.getDataAt(i1, i2))
                if i2 != self.maxIndex - 1:
                    line += ", "
            line += "}"
            if i1 != self.maxIndex - 1:
                line += ","
            out += self.printLine(3, line)
        out += self.printLine(2, "};")
        return out

class MatrixToJava:
    def __init__(self, reader : OnshapeReaderMatrix):
        self.reader = reader
        self.className = "FingerJacobianLookupTable"

    def makeJavaFile(self, filename):
        pass

if __name__ == "__main__":
    csvName = "FingerKinematicsOnshape.csv"
    path = "./"
    writer = VectorToJava(path=path, csvName=csvName)
    writer.makeJavaFile()

