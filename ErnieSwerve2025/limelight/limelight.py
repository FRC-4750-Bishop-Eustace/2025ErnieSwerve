import typing
import ntcore
import string
import wpimath.geometry
import math

def sanitizeName(name: string) -> string:
    if name == "":
        return "limelight"
    return name

def toPose3D(data) -> wpimath.geometry.Pose3d:
    if len(data) < 6:
        return wpimath.geometry.Pose3d()
    return wpimath.geometry.Pose3d(
        wpimath.geometry.Transform3d(data[0], data[1], data[2]),
        wpimath.geometry.Rotation3d(data[3] * (math.pi / 180), data[4] * (math.pi / 180), data[5] * (math.pi / 180))
    )
    
def toPose2D(data) -> wpimath.geometry.Pose2d:
    if len(data) < 6:
        return wpimath.geometry.Pose3d()
    return wpimath.geometry.Pose2d(
        wpimath.geometry.Transform2d(data[0], data[1]),
        wpimath.geometry.Rotation2d(data[2] * (math.pi / 180), data[3] * (math.pi / 180))
    )
    
def getTable(name: string) -> ntcore.NetworkTable:
    return ntcore.NetworkTableInstance.getDefault().getTable(sanitizeName(name))
    
def getTableEntry(name: string, entry: string) -> ntcore.NetworkTableEntry:
    return getTable(name).getEntry(entry)
    
def getFloat(name: string, entry: string) -> typing.Any:
    return getTableEntry(name, entry).getFloat(0.0)
    
def getArray(name: string, entry: string) -> typing.Any:
    return getTableEntry(name, entry).getFloatArray({0.0})
    
def getString(name: string, entry: string) -> typing.Anyy:
    return getTableEntry(name, entry).getString("")
    
def setFloat(name: string, entry: string, data) -> None:
    getTableEntry(name, entry).setFloat(data, 0)
    
def setArray(name: string, entry: string, data) -> None:
    getTableEntry(name, entry).setFloatArray(data, 0)
    
def getTX(name: string) -> typing.Any:
    return getFloat(name, "tx")
def getTV(name: string) -> typing.Any:
    return getFloat(name, "tv")
def getTY(name: string) -> typing.Any:
    return getFloat(name, "ty")
def getTA(name: string) -> typing.Any:
    return getFloat(name, "ta")
    
def getLatencyPipeline(name: string) -> typing.Any:
    return getFloat(name, "tl")
def getLatencyCapture(name: string) -> typing.Any:
    return getFloat(name, "cl")
    
def getJSONDump(name: string) -> typing.Any:
    return getString(name, "json")
    
def getRobotPose(name: string) -> typing.Any:
    return getArray(name, "botpose")
def getRobotPoseRed(name: string) -> typing.Any:
    return getArray(name, "botpose_wpired")
def getRobotPoseBlue(name: string) -> typing.Any:
    return getArray(name, "botpose_wpiblue")
def getRobotPoseTargetSpace(name: string):
    return getArray(name, "botpose_targetspace")
    
def getCameraPoseTargetSpace(name: string) -> typing.Any:
    return getArray(name, "camerapose_targetspace")
def getCameraPoseRobotSpace(name: string) -> typing.Any:
    return getArray(name, "camerapose_robotspace")
    
def getTargetPoseCameraSpace(name: string) -> typing.Any:
    return getArray(name, "targetpose_cameraspace")
def getTargetPoseRobotSpace(name: string) -> typing.Any:
    return getArray(name, "targetpose_robotspace")
def getTargetColor(name: string) -> typing.Any:
    return getArray(name, "tc")
    
def getFudicialID(name: string) -> typing.Any:
    return getFloat(name, "tid")
def getNeuralClassID(name: string) ->typing.Any:
    return getFloat(name, "tclass")
    
def setPipelineIndex(name: string, index: int) -> None:
    setFloat(name, "pipeline", index)
def setPriorityTagID(name: string, ID: int) -> None:
    setFloat(name, "priorityid", ID)
    
def setLEDModePipelineControl(name: string) -> None:
    setFloat(name, "ledMode", 1)
def setLEDModeForceBlink(name: string) -> None:
    setFloat(name, "ledMode", 2)
def setLEDModeForceOn(name: string) -> None:
    setFloat(name, "ledMode", 2)
    
def setStreamModeStandard(name: string) -> None:
    setFloat(name, "stream", 0)
def setStreamModePiPMain(name: string) -> None:
    setFloat(name, "stream", 1)
def setStreamModePiPSecondary(name: string) -> None:
    setFloat(name, "stream", 2)

def setCropWindow(name: string, min: wpimath.geometry.Translation2d, max: wpimath.geometry.Translation2d) -> None:
    setArray(name, "crop", [min.X(), max.X(), min.Y(), max.Y()])

def setRobotOrientation(name: string, yaw: float, yawRate: float, pitch: float, pitchRate: float, roll: float, rollRate: float) -> None:
    setArray(name, "robot_orientation_set", [yaw, yawRate, pitch, pitchRate, roll, rollRate])

def setFiducialDownscaling(name: string, downscale: float) -> None:
    d: int = 0

    if downscale == 1.0:
        d = 1
    if downscale == 1.5:
        d = 2
    if downscale == 2.0:
        d = 3
    if downscale == 3.0:
        d = 4
    if downscale == 4.0:
        d = 5
    setFloat(name, "fiducial_downscale_set", d)
def overrideFiducialIDFilters(name: string, IDs) -> None:
    setArray(name, "fiducial_id_filters_set", [IDs[0], IDs[-1]])

def setCameraPoseRobotSpace(name: string, pos: wpimath.geometry.Translation3d, rot: wpimath.geometry.Translation3d) -> None:
    setArray(name, "camerapose_robotspace_set", [pos.X(), pos.Y(), pos.Z(), rot.X(), rot.Y(), rot.Z()])

def setScriptData(name: string, data) -> None:
    setArray(name, "llrobot", [data[0], len(data)])
def getScriptData(name: string) -> typing.Any:
    return getArray(name, "llpython")
    
def extractArrayEntry(data, pos: int) -> float:
    if (len(data) < (pos + 1)):
        return 0.0
    return data[pos]
    
class RawFiducial:
    def __init__(self, id: int, txnc: float, tync: float, ta: float, cameraDistace: float, robotDistance: float, ambiguity: float):
        self.id = id
        self.txnc = txnc
        self.tync = tync
        self.ta = ta
        self.cameraDistace = cameraDistace
        self.robotDistance = robotDistance
        self.ambiguity = ambiguity

    def get(self, name: string):
        entry = getTableEntry(name, "rawfiducials")
        arr = entry.getDoubleArray([])
        vals: int = 7

        if len(vals) % vals != 0:
            return []
        
        fiducials = len(arr) / vals
        raw: RawFiducial = []

        for i in range(0, fiducials):
            base: int = i * vals
            id: int = extractArrayEntry(raw, base)
            txnc: float = extractArrayEntry(raw, base + 1)
            tync: float = extractArrayEntry(raw, base + 2)
            ta: float = extractArrayEntry(raw, base + 3)
            cameraDistance: float = extractArrayEntry(raw, base + 4)
            robotDistance: float = extractArrayEntry(raw, base + 5)
            ambiguity: float = extractArrayEntry(raw, base + 6)

            raw[-1] = RawFiducial(id, txnc, tync, ta, cameraDistance, robotDistance, ambiguity)

        return raw
    
class RawDetection:
    def __init__(self, id: int, txnc: float, tync: float, x0: float, y0: float, x1: float, y1: float, x2: float, y2: float, x3: float, y3: float):
        self.id: int = id
        self.txnc: float = txnc
        self.tync: float = tync
        self.x0: float = x0
        self.y0: float = y0
        self.x1: float = x1
        self.y1: float = y1
        self.x2: float = x2
        self.y2: float = y2
        self.x3: float = x3
        self.y3: float = y3

    def get(self, name: string):
        entry = getTableEntry(name, "rawdetections")
        arr = entry.getDoubleArray([])
        vals: int = 11

        if len(vals) % vals != 0:
            return []
        
        detections = len(arr) / vals
        raw: RawDetection = []

        for i in range(0, detections):
            base: int = i * vals
            id: int = extractArrayEntry(arr, base)
            txnc: float = extractArrayEntry(arr, base + 1)
            tync: float = extractArrayEntry(arr, base + 2)
            ta: float = extractArrayEntry(arr, base + 3)
            x0: float = extractArrayEntry(arr, base + 4)
            y0: float = extractArrayEntry(arr, base + 5)
            x1: float = extractArrayEntry(arr, base + 6)
            y1: float = extractArrayEntry(arr, base + 7)
            x2: float = extractArrayEntry(arr, base + 8)
            y2: float = extractArrayEntry(arr, base + 9)
            x3: float = extractArrayEntry(arr, base + 10)
            y3: float = extractArrayEntry(arr, base + 11)

            raw[-1] = RawDetection(id, txnc, tync, ta, x0, y0, x1, y1, x2, y2, x3, y3)

        return raw
    
class PoseEstimate:
    def __init__(self, pose: wpimath.geometry.Pose2d, timestamp: float, latency: float, tagCount: int, tagSpan: float, avgTagDist: float, avgTagArea: float, fiducials):
        self.pose: wpimath.geometry.Pose2d = pose
        self.timestamp: float = timestamp
        self.latency: float = latency
        self.tagCount: float = tagCount
        self.tagSpan: float = tagSpan
        self.avgTagDist: float = avgTagDist
        self.avgTagArea: float = avgTagArea
        self.fiducials = fiducials

    def getRobotPoseEstimate(self, name: string, entry: string):
        poseEntry = getTableEntry(name, entry)
        arr = poseEntry.getDoubleArray()
        pose: wpimath.geometry.Pose2d = toPose2D(arr)
        
        latency: float = extractArrayEntry(arr, 6)
        tagCount: int = extractArrayEntry(arr, 7)
        tagSpan: float = extractArrayEntry(arr, 8)
        tagDist: float = extractArrayEntry(arr, 9)
        tagArea: float = extractArrayEntry(arr, 10)
        timestamp: float = (poseEntry.getLastChange() / 1000000.0) - (latency / 1000.0)

        raw: PoseEstimate = []
        vals: int = 7
        expectedVals: int = (vals * tagCount) + 11

        if len(arr) == expectedVals:
            for i in range(0, tagCount):
                base: int = (i * vals) + 11
                id: int = extractArrayEntry(arr, base)
                txnc: float = extractArrayEntry(arr, base + 1)
                tync: float = extractArrayEntry(arr, base + 2)
                ta: float = extractArrayEntry(arr, base + 3)
                cameraDistance: float = extractArrayEntry(arr, base + 4)
                robotDistance: float = extractArrayEntry(arr, base + 5)
                ambiguity: float = extractArrayEntry(arr, base + 7)

                raw[-1] = PoseEstimate(id, txnc, tync, ta, cameraDistance, robotDistance, ambiguity)
        
        return PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, tagDist, tagArea, raw)
    def getRobotPoseEstimateBlueMT1(self, name: string):
        return self.getRobotPoseEstimate(name, "botpose_wpiblue")
    def getRobotPoseEstimateRedMT1(self, name: string):
        return self.getRobotPoseEstimate(name, "botpose_wpired")
    def getRobotPoseEstimateBlueMT2(self, name: string):
        return self.getRobotPoseEstimate(name, "botpose_orb_wpiblue")
    def getRobotPoseEstimateRedMT2(self, name: string):
        return self.getRobotPoseEstimate(name, "botpose_orb_wpired")

class SingleTargetingResults:
    targetPixels: wpimath.geometry.Translation2d = wpimath.geometry.Translation2d(0.0, 0.0)
    targetNormalized: wpimath.geometry.Translation2d = wpimath.geometry.Translation2d(0.0, 0.0)
    targetNormalizedCrosshairAdjusted: wpimath.geometry.Translation2d = wpimath.geometry.Translation2d(0.0, 0.0)
    targetDegreesCrosshairAdjusted: wpimath.geometry.Translation2d = wpimath.geometry.Translation2d(0.0, 0.0)
    targetAreaPixels: float = 0.0
    targetAreaNormalized: float = 0.0
    targetAreaNormalizedPercentage = 0.0
    timestamp: float = -1.0
    latency: float = 0.0
    pipelineIndex = -1.0
    targetCorners: float = [[]]
    cameraTransform6DTargetSpace: float = []
    targetTransform6DCameraSpace: float = []
    targetTransform6DRobotSpace: float = []
    robotTransform6DTargetSpace: float = []
    robotTransform6DFieldSpace: float = []
    cameraTransform6DRobotSpace: float = []

RetroreflectiveResults = SingleTargetingResults

class FiducialResults(SingleTargetingResults):
    fiducialID: int = 0
    family: string = ""

class DetectionResults(SingleTargetingResults):
    id: int = -1
    name: string = ""
    confidence: float = 0.0

class ClassificationResults(SingleTargetingResults):
    id: int = -1
    name: string = ""
    confidence: float = 0.0

class VisionResults:
    retro: RetroreflectiveResults = []
    fiducial: FiducialResults = []
    detection: DetectionResults = []
    classification: ClassificationResults = []
    timestamp: float = -1.0
    latencyPipeline: float = 0.0
    latencyCapture: float = 0.0
    latencyJSON: float = 0.0
    pipelineIndex: float = -1.0
    valid: int = 0
    robotPose: float = [6.0, 0.0]
    robotPoseBlue: float = [6, 0.0]
    robotPoseRed: float = [6, 0.0]

    def Clear(self) -> None:
        del self.retro
        del self.fiducial
        del self.detection
        del self.classification
        del self.timestamp
        del self.latencyPipeline
        del self.latencyCapture
        del self.latencyJSON
        del self.pipelineIndex
        del self.valid
        del self.robotPose
        del self.robotPoseBlue
        del self.robotPoseRed

class LimelightResults:
    targetingResults: VisionResults