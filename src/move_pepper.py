import qi
import sys
import time
import almath
import numpy as np
import math

class Pepper():
    def __init__(self):
        self.connection_url = "tcp://128.237.226.149:9559"
        self.app = qi.Application(["PredictionDemo", "--qi-url=" + self.connection_url])
        self.app.start()
        self.session = self.app.session

        self.motion = self.session.service("ALMotion")
        self.names = ["HeadYaw", "HeadPitch", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        self.angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.fractionMaxSpeed = 0.1

        self.centerPelvisIndex = 0;
        self.midBodyIndex = 12;
        self.neckBaseIndex = 13;
        self.noseIndex = 14;
        self.headIndex = 15;
        self.rightShoulderIndex = 17;
        self.rightElbowIndex = 18;
        self.rightHandIndex = 19;
        self.rightWristIndex = 20;
        self.rightThumbIndex = 21;
        self.rightFingerTipIndex = 22;
        self.leftShoulderIndex = 25;
        self.leftElbowIndex = 26;
        self.leftHandIndex = 27;
        self.leftWristIndex = 28;
        self.leftThumbIndex = 29;
        self.leftFingerTipIndex = 30;

    def movePepper(self, vals):

        self.centerPelvisX = vals[self.centerPelvisIndex,0];
        self.centerPelvisY = vals[self.centerPelvisIndex,1];
        self.centerPelvisZ = vals[self.centerPelvisIndex,2];

        self.midBodyX = vals[self.midBodyIndex,0];
        self.midBodyY = vals[self.midBodyIndex,1];
        self.midBodyZ = vals[self.midBodyIndex,2];

        self.neckBaseX = vals[self.neckBaseIndex,0];
        self.neckBaseY = vals[self.neckBaseIndex,1];
        self.neckBaseZ = vals[self.neckBaseIndex,2];

        self.noseX = vals[self.noseIndex,0];
        self.noseY = vals[self.noseIndex,1];
        self.noseZ = vals[self.noseIndex,2];

        self.headX = vals[self.headIndex,0];
        self.headY = vals[self.headIndex,1];
        self.headZ = vals[self.headIndex,2];

        self.rightShoulderX = vals[self.rightShoulderIndex,0];
        self.rightShoulderY = vals[self.rightShoulderIndex,1];
        self.rightShoulderZ = vals[self.rightShoulderIndex,2];

        self.rightElbowX = vals[self.rightElbowIndex,0];
        self.rightElbowY = vals[self.rightElbowIndex,1];
        self.rightElbowZ = vals[self.rightElbowIndex,2];

        self.rightHandX = vals[self.rightHandIndex,0];
        self.rightHandY = vals[self.rightHandIndex,1];
        self.rightHandZ = vals[self.rightHandIndex,2];

        self.rightWristX = vals[self.rightWristIndex,0];
        self.rightWristY = vals[self.rightWristIndex,1];
        self.rightWristZ = vals[self.rightWristIndex,2];

        self.rightThumbX = vals[self.rightThumbIndex,0];
        self.rightThumbY = vals[self.rightThumbIndex,1];
        self.rightThumbZ = vals[self.rightThumbIndex,2];

        self.rightFingerTipX = vals[self.rightFingerTipIndex,0];
        self.rightFingerTipY = vals[self.rightFingerTipIndex,1];
        self.rightFingerTipZ = vals[self.rightFingerTipIndex,2];

        self.leftShoulderX = vals[self.leftShoulderIndex,0];
        self.leftShoulderY = vals[self.leftShoulderIndex,1];
        self.leftShoulderZ = vals[self.leftShoulderIndex,2];

        self.leftElbowX = vals[self.leftElbowIndex,0];
        self.leftElbowY = vals[self.leftElbowIndex,1];
        self.leftElbowZ = vals[self.leftElbowIndex,2];

        self.leftHandX = vals[self.leftHandIndex,0];
        self.leftHandY = vals[self.leftHandIndex,1];
        self.leftHandZ = vals[self.leftHandIndex,2];

        self.leftWristX = vals[self.leftWristIndex,0];
        self.leftWristY = vals[self.leftWristIndex,1];
        self.leftWristZ = vals[self.leftWristIndex,2];

        self.leftThumbX = vals[self.leftThumbIndex,0];
        self.leftThumbY = vals[self.leftThumbIndex,1];
        self.leftThumbZ = vals[self.leftThumbIndex,2];

        self.leftFingerTipX = vals[self.leftFingerTipIndex,0];
        self.leftFingerTipY = vals[self.leftFingerTipIndex,1];
        self.leftFingerTipZ = vals[self.leftFingerTipIndex,2];

        # From kitchingroup.cheme.cmu.edu/blog/2015/01/18/Equation-of-a-plane-through-three-points/
        headPlanePoint1 = np.array([self.neckBaseX, self.neckBaseY, self.neckBaseZ])
        headPlanePoint2 = np.array([self.noseX, self.noseY, self.noseZ])
        headPlanePoint3 = np.array([self.headX, self.headY, self.headZ])
        headPlaneVector1 = headPlanePoint3 - headPlanePoint2
        headPlaneVector2 = headPlanePoint1 - headPlanePoint2
        headPlaneCrossProduct = np.cross(headPlaneVector1, headPlaneVector2)
        headPlaneEquationA, headPlaneEquationB, headPlaneEquationC = headPlaneCrossProduct
        headPlaneEquationD = np.dot(headPlaneCrossProduct, headPlanePoint3)

        # From kitchingroup.cheme.cmu.edu/blog/2015/01/18/Equation-of-a-plane-through-three-points/
        bodyPlanePoint1 = np.array([self.rightShoulderX, self.rightShoulderY, self.rightShoulderZ])
        bodyPlanePoint2 = np.array([self.leftShoulderX, self.leftShoulderY, self.leftShoulderZ])
        bodyPlanePoint3 = np.array([self.midBodyX, self.midBodyY, self.midBodyZ])
        bodyPlaneVector1 = bodyPlanePoint3 - bodyPlanePoint1
        bodyPlaneVector2 = bodyPlanePoint2 - bodyPlanePoint1
        bodyPlaneCrossProduct = np.cross(bodyPlaneVector1, bodyPlaneVector2)
        bodyPlaneEquationA, bodyPlaneEquationB, bodyPlaneEquationC = bodyPlaneCrossProduct
        bodyPlaneEquationD = np.dot(bodyPlaneCrossProduct, bodyPlanePoint3)

        # Angle between 2 planes from https://math.tutorvista.com/geometry/angle-between-two-planes.html
        headYaw = math.acos((bodyPlaneEquationA * headPlaneEquationA + bodyPlaneEquationB * headPlaneEquationB + bodyPlaneEquationC * headPlaneEquationC) / (math.sqrt(bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2) * math.sqrt(headPlaneEquationA ** 2 + headPlaneEquationB ** 2 + headPlaneEquationC ** 2)))
        pepperHeadYaw = max(min(headYaw - math.pi/2, math.radians(90)), math.radians(-90)) 

        # Angle between a line and a plane from https://www.vitutor.com/geometry/distance/line_plane.html
        headVector = headPlanePoint3 - headPlanePoint1
        pepperHeadPitch = math.asin((bodyPlaneEquationA * headVector[0] + bodyPlaneEquationB * headVector[1] + bodyPlaneEquationC * headVector[2]) / (math.sqrt(bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2) * math.sqrt(headVector[0] ** 2 + headVector[1] ** 2 + headVector[2] ** 2)))

        lengthOfRightUpperArm = math.sqrt((self.rightShoulderX - self.rightElbowX) ** 2 + (self.rightShoulderY - self.rightElbowY) ** 2 + (self.rightShoulderZ - self.rightElbowZ) ** 2)
        deltaZRightUpperArm = self.rightShoulderZ - self.rightElbowZ
        # From https://mathinsight.org/distance_point_plane
        distanceRightElbowPointFromBodyPlane = (bodyPlaneEquationA * self.rightElbowX + bodyPlaneEquationB * self.rightElbowY + bodyPlaneEquationC * self.rightElbowZ + bodyPlaneEquationD) / math.sqrt(bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2)
        
        pepperRightShoulderPitch = min(max(math.atan2(deltaZRightUpperArm, distanceRightElbowPointFromBodyPlane), math.radians(-119.5)), math.radians(119.5))
        pepperRightShoulderRoll = max(-math.acos(abs(deltaZRightUpperArm) / lengthOfRightUpperArm), math.radians(-89.5))

        # From https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_plane
        rightElbowTranslatedD = (bodyPlaneEquationD - bodyPlaneEquationA * self.rightElbowX - bodyPlaneEquationB * self.rightElbowY - bodyPlaneEquationC * self.rightElbowZ) 
        closestPointOnBodyPlaneToRightElbowPointX = (bodyPlaneEquationA * rightElbowTranslatedD) / (bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2) + self.rightElbowX
        closestPointOnBodyPlaneToRightElbowPointY = (bodyPlaneEquationB * rightElbowTranslatedD) / (bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2) + self.rightElbowY
        closestPointOnBodyPlaneToRightElbowPointZ = (bodyPlaneEquationC * rightElbowTranslatedD) / (bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2) + self.rightElbowZ
        closestPointOnBodyPlaneToRightElbowPoint = np.array([closestPointOnBodyPlaneToRightElbowPointX, closestPointOnBodyPlaneToRightElbowPointY, closestPointOnBodyPlaneToRightElbowPointZ])

        rightUpperArmPlanePoint1 = np.array([self.rightShoulderX, self.rightShoulderY, self.rightShoulderZ])
        rightUpperArmPlanePoint2 = np.array([closestPointOnBodyPlaneToRightElbowPoint[0], closestPointOnBodyPlaneToRightElbowPoint[1], closestPointOnBodyPlaneToRightElbowPoint[2] + deltaZRightUpperArm])
        rightUpperArmPlanePoint3 = np.array([self.rightElbowX, self.rightElbowY, self.rightElbowZ])
        rightUpperArmPlaneVector1 = rightUpperArmPlanePoint3 - rightUpperArmPlanePoint1
        rightUpperArmPlaneVector2 = rightUpperArmPlanePoint2 - rightUpperArmPlanePoint1
        rightUpperArmPlaneCrossProduct = np.cross(rightUpperArmPlaneVector1, rightUpperArmPlaneVector2)
        rightUpperArmPlaneEquationA, rightUpperArmPlaneEquationB, rightUpperArmPlaneEquationC = rightUpperArmPlaneCrossProduct
        
        rightArmPlanePoint1 = np.array([self.rightShoulderX, self.rightShoulderY, self.rightShoulderZ])
        rightArmPlanePoint2 = np.array([self.rightElbowX, self.rightElbowY, self.rightElbowZ])
        rightArmPlanePoint3 = np.array([self.rightHandX, self.rightHandY, self.rightHandZ])
        rightArmPlaneVector1 = rightArmPlanePoint1 - rightArmPlanePoint2
        rightArmPlaneVector2 = rightArmPlanePoint3 - rightArmPlanePoint2
        rightArmPlaneCrossProduct = np.cross(rightArmPlaneVector1, rightArmPlaneVector2)
        rightArmPlaneEquationA, rightArmPlaneEquationB, rightArmPlaneEquationC = rightArmPlaneCrossProduct

        # Angle between 2 planes from https://math.tutorvista.com/geometry/angle-between-two-planes.html
        pepperRightElbowYaw = math.acos((rightUpperArmPlaneEquationA * rightArmPlaneEquationA + rightUpperArmPlaneEquationB * rightArmPlaneEquationB + rightUpperArmPlaneEquationC * rightArmPlaneEquationC) / (math.sqrt(rightUpperArmPlaneEquationA ** 2 + rightUpperArmPlaneEquationB ** 2 + rightUpperArmPlaneEquationC ** 2) * math.sqrt(rightArmPlaneEquationA ** 2 + rightArmPlaneEquationB ** 2 + rightArmPlaneEquationC ** 2)))

        # Angle between 3 points from https://stackoverflow.com/questions/19729831/angle-between-3-points-in-3d-space
        rightArmPlaneVector1Magnitude = math.sqrt(rightArmPlaneVector1[0] ** 2 + rightArmPlaneVector1[1] ** 2 + rightArmPlaneVector1[2] ** 2)
        rightArmPlaneVector2Magnitude = math.sqrt(rightArmPlaneVector2[0] ** 2 + rightArmPlaneVector2[1] ** 2 + rightArmPlaneVector2[2] ** 2)
        rightArmPlaneVector1Normalized = rightArmPlaneVector1 / rightArmPlaneVector1Magnitude
        rightArmPlaneVector2Normalized = rightArmPlaneVector2 / rightArmPlaneVector2Magnitude
        pepperRightElbowRoll = min(math.radians(180) - math.acos(np.dot(rightArmPlaneVector1Normalized, rightArmPlaneVector2Normalized)), math.radians(89.5))

        rightForearmPlanePoint1 = np.array([self.rightElbowX, self.rightElbowY, self.rightElbowZ])
        rightForearmPlanePoint2 = np.array([self.rightWristX, self.rightWristY, self.rightWristZ])
        rightForearmPlanePoint3 = np.array([self.rightThumbX, self.rightThumbY, self.rightThumbZ])
        rightForearmPlaneVector1 = rightForearmPlanePoint1 - rightForearmPlanePoint2
        rightForearmPlaneVector2 = rightForearmPlanePoint3 - rightForearmPlanePoint2
        rightForearmPlaneCrossProduct = np.cross(rightForearmPlaneVector1, rightForearmPlaneVector2)
        rightForearmPlaneEquationA, rightForearmPlaneEquationB, rightForearmPlaneEquationC = rightForearmPlaneCrossProduct

        # Angle between 2 planes from https://math.tutorvista.com/geometry/angle-between-two-planes.html
        pepperRightWristYaw = math.acos((rightArmPlaneEquationA * rightForearmPlaneEquationA + rightArmPlaneEquationB * rightForearmPlaneEquationB + rightArmPlaneEquationC * rightForearmPlaneEquationC) / (math.sqrt(rightArmPlaneEquationA ** 2 + rightArmPlaneEquationB ** 2 + rightArmPlaneEquationC ** 2) * math.sqrt(rightForearmPlaneEquationA ** 2 + rightForearmPlaneEquationB ** 2 + rightForearmPlaneEquationC ** 2)))

        lengthOfLeftUpperArm = math.sqrt((self.leftShoulderX - self.leftElbowX) ** 2 + (self.leftShoulderY - self.leftElbowY) ** 2 + (self.leftShoulderZ - self.leftElbowZ) ** 2)
        deltaZLeftUpperArm = self.leftShoulderZ - self.leftElbowZ
        # From https://mathinsight.org/distance_point_plane
        distanceLeftElbowPointFromBodyPlane = (bodyPlaneEquationA * self.leftElbowX + bodyPlaneEquationB * self.leftElbowY + bodyPlaneEquationC * self.leftElbowZ + bodyPlaneEquationD) / math.sqrt(bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2)
        
        pepperLeftShoulderPitch = min(max(math.atan2(deltaZLeftUpperArm, distanceLeftElbowPointFromBodyPlane), math.radians(-119.5)), math.radians(119.5))
        pepperLeftShoulderRoll = min(math.acos(abs(deltaZLeftUpperArm) / lengthOfLeftUpperArm), math.radians(89.5))

        # From https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_plane
        leftElbowTranslatedD = (bodyPlaneEquationD - bodyPlaneEquationA * self.leftElbowX - bodyPlaneEquationB * self.leftElbowY - bodyPlaneEquationC * self.leftElbowZ) 
        closestPointOnBodyPlaneToLeftElbowPointX = (bodyPlaneEquationA * leftElbowTranslatedD) / (bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2) + self.leftElbowX
        closestPointOnBodyPlaneToLeftElbowPointY = (bodyPlaneEquationB * leftElbowTranslatedD) / (bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2) + self.leftElbowY
        closestPointOnBodyPlaneToLeftElbowPointZ = (bodyPlaneEquationC * leftElbowTranslatedD) / (bodyPlaneEquationA ** 2 + bodyPlaneEquationB ** 2 + bodyPlaneEquationC ** 2) + self.leftElbowZ
        closestPointOnBodyPlaneToLeftElbowPoint = np.array([closestPointOnBodyPlaneToLeftElbowPointX, closestPointOnBodyPlaneToLeftElbowPointY, closestPointOnBodyPlaneToLeftElbowPointZ])

        leftUpperArmPlanePoint1 = np.array([self.leftShoulderX, self.leftShoulderY, self.leftShoulderZ])
        leftUpperArmPlanePoint2 = np.array([closestPointOnBodyPlaneToLeftElbowPoint[0], closestPointOnBodyPlaneToLeftElbowPoint[1], closestPointOnBodyPlaneToLeftElbowPoint[2] + deltaZLeftUpperArm])
        leftUpperArmPlanePoint3 = np.array([self.leftElbowX, self.leftElbowY, self.leftElbowZ])
        leftUpperArmPlaneVector1 = leftUpperArmPlanePoint3 - leftUpperArmPlanePoint1
        leftUpperArmPlaneVector2 = leftUpperArmPlanePoint2 - leftUpperArmPlanePoint1
        leftUpperArmPlaneCrossProduct = np.cross(leftUpperArmPlaneVector1, leftUpperArmPlaneVector2)
        leftUpperArmPlaneEquationA, leftUpperArmPlaneEquationB, leftUpperArmPlaneEquationC = leftUpperArmPlaneCrossProduct
        
        leftArmPlanePoint1 = np.array([self.leftShoulderX, self.leftShoulderY, self.leftShoulderZ])
        leftArmPlanePoint2 = np.array([self.leftElbowX, self.leftElbowY, self.leftElbowZ])
        leftArmPlanePoint3 = np.array([self.leftHandX, self.leftHandY, self.leftHandZ])
        leftArmPlaneVector1 = leftArmPlanePoint1 - leftArmPlanePoint2
        leftArmPlaneVector2 = leftArmPlanePoint3 - leftArmPlanePoint2
        leftArmPlaneCrossProduct = np.cross(leftArmPlaneVector1, leftArmPlaneVector2)
        leftArmPlaneEquationA, leftArmPlaneEquationB, leftArmPlaneEquationC = leftArmPlaneCrossProduct

        # Angle between 2 planes from https://math.tutorvista.com/geometry/angle-between-two-planes.html
        pepperLeftElbowYaw = -math.acos((leftUpperArmPlaneEquationA * leftArmPlaneEquationA + leftUpperArmPlaneEquationB * leftArmPlaneEquationB + leftUpperArmPlaneEquationC * leftArmPlaneEquationC) / (math.sqrt(leftUpperArmPlaneEquationA ** 2 + leftUpperArmPlaneEquationB ** 2 + leftUpperArmPlaneEquationC ** 2) * math.sqrt(leftArmPlaneEquationA ** 2 + leftArmPlaneEquationB ** 2 + leftArmPlaneEquationC ** 2)))

        # Angle between 3 points from https://stackoverflow.com/questions/19729831/angle-between-3-points-in-3d-space
        leftArmPlaneVector1Magnitude = math.sqrt(leftArmPlaneVector1[0] ** 2 + leftArmPlaneVector1[1] ** 2 + leftArmPlaneVector1[2] ** 2)
        leftArmPlaneVector2Magnitude = math.sqrt(leftArmPlaneVector2[0] ** 2 + leftArmPlaneVector2[1] ** 2 + leftArmPlaneVector2[2] ** 2)
        leftArmPlaneVector1Normalized = leftArmPlaneVector1 / leftArmPlaneVector1Magnitude
        leftArmPlaneVector2Normalized = leftArmPlaneVector2 / leftArmPlaneVector2Magnitude
        pepperLeftElbowRoll = -min(math.radians(180) - math.acos(np.dot(leftArmPlaneVector1Normalized, leftArmPlaneVector2Normalized)), math.radians(89.5))

        leftForearmPlanePoint1 = np.array([self.leftElbowX, self.leftElbowY, self.leftElbowZ])
        leftForearmPlanePoint2 = np.array([self.leftWristX, self.leftWristY, self.leftWristZ])
        leftForearmPlanePoint3 = np.array([self.leftThumbX, self.leftThumbY, self.leftThumbZ])
        leftForearmPlaneVector1 = leftForearmPlanePoint1 - leftForearmPlanePoint2
        leftForearmPlaneVector2 = leftForearmPlanePoint3 - leftForearmPlanePoint2
        leftForearmPlaneCrossProduct = np.cross(leftForearmPlaneVector1, leftForearmPlaneVector2)
        leftForearmPlaneEquationA, leftForearmPlaneEquationB, leftForearmPlaneEquationC = leftForearmPlaneCrossProduct

        # Angle between 2 planes from https://math.tutorvista.com/geometry/angle-between-two-planes.html
        pepperLeftWristYaw = -math.acos((leftArmPlaneEquationA * leftForearmPlaneEquationA + leftArmPlaneEquationB * leftForearmPlaneEquationB + leftArmPlaneEquationC * leftForearmPlaneEquationC) / (math.sqrt(leftArmPlaneEquationA ** 2 + leftArmPlaneEquationB ** 2 + leftArmPlaneEquationC ** 2) * math.sqrt(leftForearmPlaneEquationA ** 2 + leftForearmPlaneEquationB ** 2 + leftForearmPlaneEquationC ** 2)))

        self.angles = [pepperHeadYaw, pepperHeadPitch, pepperRightShoulderPitch, pepperRightShoulderRoll, pepperRightElbowYaw, pepperRightElbowRoll, pepperRightWristYaw, pepperLeftShoulderPitch, pepperLeftShoulderRoll, pepperLeftElbowYaw, pepperLeftElbowRoll, pepperLeftWristYaw]

        self.motion.setAngles(self.names,self.angles,self.fractionMaxSpeed)