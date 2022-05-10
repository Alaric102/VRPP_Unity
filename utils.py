import numpy as np

def ExtractVector(words) -> np.ndarray((3,1)):
    vector = []
    for word in words:
        vector.append(float(word.replace(",", ".", 1)))
    return np.array(vector)

def ExtractVectorInt(words) -> np.ndarray((3,1), dtype=int):
    vector = []
    for word in words:
        vector.append(int(word))
    return np.array(vector)

def ExtractFloat(word: str) -> float:
    return float(word.replace(",", ".", 1))
    
def ExtractInt(word: str) -> float:
    return int(word)

def LoadLocalPlan(prefix: str, folder: str = "D:/catkin_ws/src/VRPP_ROS/launch/") -> bool:
    positions = []
    rotations = []
    full_path = folder + prefix + "_local.txt"
    try:
        f = open(full_path)
        # Get replaning requests
        words = f.readline().split(maxsplit=2)
        replanningCounts = ExtractInt(words[1])
        # Get time in ms
        words = f.readline().split(maxsplit=2)
        time_ms = ExtractFloat(words[1])
        
        line = f.readline()
        while line:
            data = line.split(maxsplit=2, sep=";")
            positions_word = data[0].split(maxsplit=3, sep=" ")
            position = ExtractVector(positions_word)
                
            rotations_word = data[1].split(maxsplit=3, sep=" ")
            rotation = ExtractVector(rotations_word)
            
            positions.append(position)
            rotations.append(rotation)
            line = f.readline()
        f.close()
        return True, positions, rotations, time_ms, replanningCounts
    except FileNotFoundError:
        print("No such file:" , full_path)
        return False, positions, rotations, np.inf, 0

def LoadVoxelMap(folder: str = "D:/catkin_ws/src/VRPP_ROS/launch/") -> bool:
    full_path = folder + "map.txt"
    try:
        f = open(full_path)
        # Get voxelMap size
        mapSize_words = f.readline().split(maxsplit=3)
        mapSize = ExtractVectorInt(mapSize_words)
        mapArray = np.full(mapSize, False, dtype=bool)

        # # Get voxelMap grid size
        gridSize_words = f.readline().split(maxsplit=3)
        gridSize = ExtractVector(gridSize_words)

        # # Get voxelMap min corner
        minCorner_words = f.readline().split(maxsplit=3)
        minCorner = ExtractVector(minCorner_words)

        line = f.readline()
        # self.__voxelMapData = np.full(self.__voxelMapSize, False, dtype=bool)
        while line:
            data = line.split(maxsplit=2, sep=";")
            dPosition_words = data[0].split(maxsplit=3, sep=" ")
            dPosition = ExtractVectorInt(dPosition_words)
            # cPosition_words = data[1].split(maxsplit=3, sep=" ")
            # cPosition = ExtractVector(dPosition_words)
            mapArray[dPosition[0], dPosition[1], dPosition[2]] = True
                
            line = f.readline()
        f.close()
        return True, mapArray
    except FileNotFoundError:
        print("No such file:" , full_path)
        return False

def LoadWeightPlan(prefix: str, folder: str = "D:/catkin_ws/src/VRPP_ROS/launch/") -> bool:
    full_path = folder + prefix + "_weight.txt"
    try:
        f = open(full_path)
        # Get voxelMap size
        mapSize_words = f.readline().split(maxsplit=3)
        mapSize = ExtractVectorInt(mapSize_words)
        mapArray = np.full(mapSize, 0.0, dtype=float)
        
        line = f.readline()
        # self.__voxelMapData = np.full(self.__voxelMapSize, False, dtype=bool)
        while line:
            data = line.split(maxsplit=2, sep=";")
            dPosition_words = data[0].split(maxsplit=3, sep=" ")
            dPosition = ExtractVectorInt(dPosition_words)
            mapArray[dPosition[0], dPosition[1], dPosition[2]] = ExtractFloat(data[1])
                
            line = f.readline()
        f.close()
        return True, mapArray
    except FileNotFoundError:
        print("No such file:" , full_path)
        return False, mapArray

def LoadGlobalPlan(prefix: str, folder: str = "D:/catkin_ws/src/VRPP_ROS/launch/") -> bool:
    positions = []
    full_path = folder + prefix + "_global.txt"
    try:
        f = open(full_path)
        # Get time in ms
        words = f.readline().split(maxsplit=2)
        time_ms = words[1]
        
        line = f.readline()
        while line:
            data = line.split(maxsplit=2, sep=";")
            positions_word = data[0].split(maxsplit=3, sep=" ")
            position = ExtractVectorInt(positions_word)
            
            positions.append(position)
            line = f.readline()
        f.close()
        return True, positions, time_ms
    except FileNotFoundError:
        print("No such file:" , full_path)
        return False, positions, None