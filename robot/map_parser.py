import xml.etree.ElementTree as ET

CELLROWS = 7
CELLCOLS = 14


class LabMap:
    def __init__(self, filename: str):
        tree = ET.parse(filename)
        root = tree.getroot()

        # Same map layout used by ciberRatoTools/pClient/mainRob.py.
        self.lab_map = [[" "] * (CELLCOLS * 2 - 1) for _ in range(CELLROWS * 2 - 1)]
        for child in root.iter("Row"):
            line = child.attrib["Pattern"]
            row = int(child.attrib["Pos"])

            if row % 2 == 0:
                for col, value in enumerate(line):
                    if (col + 1) % 3 == 0 and value == "|":
                        self.lab_map[row][(col + 1) // 3 * 2 - 1] = "|"
            else:
                for col, value in enumerate(line):
                    if col % 3 == 0 and value == "-":
                        self.lab_map[row][col // 3 * 2] = "-"

    def print(self):
        for row in reversed(self.lab_map):
            print("".join(row))
