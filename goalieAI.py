from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys
import random
import math
import _thread as thread

# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]

HIGHESTSCORES = []


class BodyGameRuntime(object):
    def __init__(self, soccerGoalie=False, soccerKicker=False, tennisAI=False):
        pygame.init()   # gets pygame started (starts game)

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        # set_mode(resolution = (width, height), flags= collection of additional options, depth = num of bits for color)
        # the flags controls type of display wanted
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                                       # hardware accelerated   #used with HWSURFACE    # display window is resizeable

        pygame.display.set_caption("Kinect for Windows v2 Body Game")   # title of the window

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        self.soccerGoalie = soccerGoalie
        self.soccerKicker = soccerKicker
        self.tennisAI = tennisAI

        if self.soccerGoalie or self.soccerKicker or self.tennisAI:
            self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
        else:
            self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data 
        self._bodies = None

        # getting all coordinates of right hand 
        self.rightHandCoor = (0,0)
        self.rightTipCoor = (0,0)
        self.rightWristCoor = (0,0)
        self.rightElbowCoor = (0,0)
        self.collideRight = False
        self.rightHandOpen = True

        # getting all coordinates of left hand
        self.leftHandCoor = (0,0)
        self.leftTipCoor = (0,0)
        self.leftWristCoor = (0,0)
        self.leftElbowCoor = (0,0)
        self.collideLeft = False
        self.leftHandOpen = True

        # getting coordinates of right foot
        self.rightAnkleCoor = (0,0)
        self.rightFootCoor = (0,0)
        self.collideFoot = False

        # getting coordinates of left foot
        self.leftAnkleCoor = (0,0)
        self.leftFootCoor = (0,0)

        # coordinates of middle of spine
        self.spineMidCoor = (0,0)

        # characteristics of soccer ball
        self.balls = pygame.image.load('soccer.png').convert_alpha()
        ballsRect = self.balls.get_rect()
        self.ballx = (self._infoObject.current_w) // 2
        self.bally = (self._infoObject.current_h) // 2
        self.ballr = ballsRect.height / 2
        self.soccerList = []
        self.rightKick = False
        self.leftKick = False
        self.soccer = False   # check if in soccer mode
        self.soccerTarget = False

        self.soccerGoalFront = pygame.image.load('goal.png')
        self.goalieCoor = [self._infoObject.current_w / 2, self._infoObject.current_h / 2]
        self.goalieList = []
        self.drawG = ''

        # characteristics of tennis racket and balll
        self.tennisRacket = [pygame.image.load("rightTennis.png"), pygame.image.load("leftTennis.png")]
        self.tennisGrip = ''   # location of the grip to hold
        self.rightSide = True    # check if racket on right side
        self.leftSide = False    # check if racket on left side
        self.tennisBall = pygame.image.load("tennis.png").convert_alpha()
        self.tennisBallRect = self.tennisBall.get_rect()
        self.tennisBallr = self.tennisBallRect.height / 2   # original radius of the ball
        self.tennisBallList = []   # list of all tennis balls
        self.tennis = False   # check if in tennis mode
        self.tennisServe = False
        self.tennisTarget = False

        self.background = ""

        self.target = pygame.image.load('target.png').convert_alpha()
        self.targets = []

        self.timerCount = 0   # 70 counts a second

        self.depthOfPlayer = 0   # the depth/z coordinate of the player position
        self.depthOfRightHand = 0
        self.depthOfLeftHand = 0
        self.depthRightFoot = 0
        self.depthLeftFoot = 0
        self.depthRightHip = 0
        self.depthLeftHip = 0
        self.depthRightShoulder = 0
        self.depthLeftShoulder = 0

        self.originalRightHand = (0,0)
        self.orginalLeftHand = (0,0)
        self.orginalRightFoot = (0,0)
        self.orginalLeftFoot = (0,0)

        self.functionCount = 0
        self.score = 0
        self.timer = 30   # 30 seconds for each game

        self.starter = False
        self.gameOver = False


        # Kinect runtime object, we want only color and body frames

    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        
        # setting the coordinates of each joint
        if joint0 == PyKinectV2.JointType_HandRight:
            self.rightHandCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
            self.depthOfRightHand = joints[getattr(PyKinectV2, "JointType_HandRight")].Position.z
        elif joint0 == PyKinectV2.JointType_HandTipRight:
            self.rightTipCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        elif joint0 == PyKinectV2.JointType_WristRight:
            self.rightWristCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        elif joint0 == PyKinectV2.JointType_ElbowRight:
            self.rightElbowCoor = (jointPoints[joint0].x, jointPoints[joint0].y)

        if joint0 == PyKinectV2.JointType_HandLeft:
            self.leftHandCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
            self.depthOfLeftHand = joints[getattr(PyKinectV2, "JointType_HandLeft")].Position.z
        elif joint0 == PyKinectV2.JointType_HandTipLeft:
            self.leftTipCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        elif joint0 == PyKinectV2.JointType_WristLeft:
            self.leftWristCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        elif joint0 == PyKinectV2.JointType_ElbowLeft:
            self.leftElbowCoor = (jointPoints[joint0].x, jointPoints[joint0].y)

        if joint0 == PyKinectV2.JointType_AnkleRight:
            self.rightAnkleCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
            self.depthRightFoot = joints[getattr(PyKinectV2, "JointType_AnkleRight")].Position.z
        if joint0 == PyKinectV2.JointType_FootRight:
            self.rightFootCoor = (jointPoints[joint0].x, jointPoints[joint0].y)

        if joint0 == PyKinectV2.JointType_AnkleLeft:
            self.leftAnkleCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
            self.depthLeftFoot = joints[getattr(PyKinectV2, "JointType_AnkleLeft")].Position.z
        if joint0 == PyKinectV2.JointType_FootLeft:
            self.leftFootCoor = (jointPoints[joint0].x, jointPoints[joint0].y)

        if joint0 == PyKinectV2.JointType_SpineMid:
            self.depthOfPlayer = joints[getattr(PyKinectV2, "JointType_SpineMid")].Position.z
            self.spineMidCoor = (jointPoints[joint0].x, jointPoints[joint0].y)


        if joint0 == PyKinectV2.JointType_HipRight:
            self.depthRightHip = joints[getattr(PyKinectV2, "JointType_HipRight")].Position.z

        if joint0 == PyKinectV2.JointType_HipRight:
            self.depthLeftHip = joints[getattr(PyKinectV2, "JointType_HipLeft")].Position.z

        if joint0 == PyKinectV2.JointType_ShoulderRight:
            self.depthRightShoulder = joints[getattr(PyKinectV2, "JointType_ShoulderRight")].Position.z

        if joint0 == PyKinectV2.JointType_ShoulderLeft:
            self.depthLeftShoulder = joints[getattr(PyKinectV2, "JointType_ShoulderLeft")].Position.z

        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);

        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);

    
    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()


###########################################################################
   # STARTER MODE
###########################################################################
    def starterMode(self):
        if self.starter == True:
            tennisRect = pygame.Rect(0,100,500,200)
            soccerRect = pygame.Rect(0, 700, 500, 200)
            highScore = pygame.Rect(self._infoObject.current_w - 500,100,500,200)

            font = pygame.font.SysFont('Courier', 60, bold=True)

            # drawing tennis mode box
            instruct = pygame.font.SysFont('Courier', 40, bold=True)
            pygame.draw.rect(self._frame_surface, (153, 255, 51), tennisRect)
            tennisText = "Tennis Mode"
            tennisDraw = font.render(tennisText, True, (0, 0, 0) )
            self._frame_surface.blit(tennisDraw, (50, 150))
            instructionT = "touch w/ left hand"
            tennisDraw = instruct.render(instructionT, True, (0, 0, 0) )
            self._frame_surface.blit(tennisDraw, (50, 100))

            # drawing soccer mode box
            pygame.draw.rect(self._frame_surface, (0, 0, 0), soccerRect)
            soccerText = "Soccer Mode"
            soccerDraw = font.render(soccerText, True, (255, 255, 255) )
            self._frame_surface.blit(soccerDraw, (50, 750))
            instructionS = "kick w/ left foot"
            soccerDraw = instruct.render(instructionS, True, (255, 255, 255) )
            self._frame_surface.blit(soccerDraw, (50, 700))

            # drawing highest score box
            pygame.draw.rect(self._frame_surface, (255, 51, 51), highScore)
            highText = "High Scores"
            highDraw = font.render(highText, True, (255, 255, 255) )
            self._frame_surface.blit(highDraw, (self._infoObject.current_w - 500, 150))
            instructionH = "touch w/ right hand"
            highDraw = instruct.render(instructionH, True, (255, 255, 255) )
            self._frame_surface.blit(highDraw, (self._infoObject.current_w - 500, 100))
            

            # checking the mode player chose
            if tennisRect.collidepoint(self.leftHandCoor):
                self.tennis = True
                self.starter = False
                self.soccer = False
            if soccerRect.collidepoint(self.leftAnkleCoor):
                self.soccer = True
                self.starter = False
                self.tennis = False
            if highScore.collidepoint(self.rightHandCoor):
                pass

###########################################################################
   # SOCCER MODE
###########################################################################
    def soccerMode(self):
        if self.soccer == True:
            self.gameOver = False
            self.score = 0
            self.timer = 30
            targetRect = pygame.Rect(20,100,500,200)
            homeRect = pygame.Rect(20, 700, 500, 200)
            goalieRect = pygame.Rect(self._infoObject.current_w - 520,100,500,200)
            kickerRect = pygame.Rect(self._infoObject.current_w - 520,700,500,200)

            font = pygame.font.SysFont('Courier', 60, bold=True)
            
            pygame.draw.rect(self._frame_surface, (153, 255, 51), targetRect)
            targetText = "Target Mode"
            targetDraw = font.render(targetText, True, (0, 0, 0) )
            self._frame_surface.blit(targetDraw, (50, 150))

            pygame.draw.rect(self._frame_surface, (0, 0, 0), homeRect)
            homeText = "Home"
            homeDraw = font.render(homeText, True, (255, 255, 255) )
            self._frame_surface.blit(homeDraw, (50, 750))

            pygame.draw.rect(self._frame_surface, (255, 51, 51), goalieRect)
            goalieText = "Goalie"
            goalieDraw = font.render(goalieText, True, (255, 255, 255) )
            self._frame_surface.blit(goalieDraw, (self._infoObject.current_w - 500, 150))

            pygame.draw.rect(self._frame_surface, (255, 51, 51), kickerRect)
            kickerText = "Kicker"
            kickerDraw = font.render(kickerText, True, (255, 255, 255) )
            self._frame_surface.blit(kickerDraw, (self._infoObject.current_w - 500, 750))

            text = "Use Right Hand To Select"
            ren = font.render(text, True, (127, 6, 119) )
            self._frame_surface.blit(ren, (400, 300))
            
            if targetRect.collidepoint(self.rightHandCoor):
                self.soccer = False   # check if in soccer mode
                self.soccerTarget = True
                self.soccerGoalie = False
                self.soccerKicker = False
                self.starter = False
            if homeRect.collidepoint(self.rightHandCoor):
                self.soccer = False   # check if in soccer mode
                self.soccerTarget = False
                self.soccerGoalie = False
                self.soccerKicker = False
                self.starter = True
            if goalieRect.collidepoint(self.rightHandCoor):
                self.soccer = False   # check if in soccer mode
                self.soccerTarget = False
                self.soccerGoalie = True
                self.soccerKicker = False
                self.starter = False
            if kickerRect.collidepoint(self.rightHandCoor):
                self.soccer = False   # check if in soccer mode
                self.soccerTarget = False
                self.soccerGoalie = False
                self.soccerKicker = True
                self.starter = True
    
    # check foot collision 
    def checkFeetCollison(self, elem):
        rightCoorx = self.rightAnkleCoor[0]
        rightCoory = self.rightAnkleCoor[1]
        leftCoorx = self.leftAnkleCoor[0]
        leftCoory = self.leftAnkleCoor[1]

        distanceRight = ((rightCoorx - elem[0].centerx) ** 2 + (rightCoory - elem[0].centery) ** 2) ** 0.5
        distanceLeft = ((leftCoorx - elem[0].centerx) ** 2 + (leftCoory - elem[0].centery) ** 2) ** 0.5
        r = elem[0].height / 2
        if distanceRight <= r or distanceLeft <= r:
            elem[3] = True
            if distanceRight < distanceLeft:
                self.rightKick = True
                self.leftKick = False
            else:
                self.rightKick = False
                self.leftKick = True
    
    def directionS(self):
        distance = self.depthRightHip - self.depthLeftHip
        return distance

###########################################################################
   # SOCCER TARGET MODE
###########################################################################
    # run the soccer mode
    def soccerTargetMode(self):
        if self.soccerTarget == True:
            self.drawScoreTime()
            for elem in self.soccerList:
                self.checkFeetCollison(elem)
                self.moveFootBall(elem)
                self.collideTarget(elem)
            self.drawBall()
            self.drawTarget()
            if self.timerFunc():
                self.soccerTarget = False

    def drawBall(self):
        if len(self.soccerList) == 0:
            soccerx = random.uniform(self.tennisBallr, (self._infoObject.current_w - self.tennisBallr))
            soccery = self._infoObject.current_h - self.ballr
            scale = 150
            ball = pygame.image.load("soccer.png")
            drawBall = ball.get_rect(center=(soccerx, soccery))
            speed = 100
            hit = False
            angle = 0
            self.soccerList.append([drawBall, ball, scale, hit, speed, angle])
        for elem in self.soccerList:
            if elem[3] == False:
                if elem[2] <= 50:
                    self.soccerList.remove(elem)
                if self.timerCount % 2 == 0:
                    elem[2] -= 3
                self._frame_surface.blit(pygame.transform.smoothscale(elem[1], [elem[2], elem[2]]), elem[0])
            else:
                if elem[2] >= 150:
                    self.soccerList.remove(elem)
                if self.timerCount % 2 == 0:
                    elem[2] += 3
                self._frame_surface.blit(pygame.transform.smoothscale(elem[1], [elem[2], elem[2]]), elem[0])
            pygame.display.flip()
       

    # soccer ball moved with foot
    def moveFootBall(self, elem):
        if elem[3] == True:
            elem[5] = self.directionS() * 100
            if elem[5] < 0:
                elem[5] *= 2
            if elem[4] > 0:
                elem[0].centery -= 20
                elem[4] -= 1
            elif elem[4] == 0:
                elem[4] -= 1
            else:
                elem[0].centery += 10
            elem[0].centerx += elem[5]


    def drawTarget(self):
        if self.timerCount % 140 == 0:
            targetx = random.uniform(100, (self._infoObject.current_w - 100))
            targety = random.uniform(100, (self._infoObject.current_h / 2))
            target = pygame.image.load("target.png")
            drawtar = target.get_rect(center=(targetx, targety))
            self.targets.append([drawtar, target])
        for elem in self.targets:
            self._frame_surface.blit(elem[1], elem[0])

    def collideTarget(self, elem):
        for target in self.targets:
            distance = ((target[0].centerx - elem[0].centerx) ** 2 + (target[0].centery - elem[0].centery) ** 2) ** 0.5
            if distance <= target[0].height / 2 and elem[3] == True:
                self.targets.remove(target)
                if self.soccerTarget:
                    self.soccerList.remove(elem)
                elif self.tennisTarget:
                    self.tennisBallList.remove(elem)
                self.score += 1

###########################################################################
   # SOCCER KICKER MODE
###########################################################################

    def soccerKickerMode(self):
        if self.soccerKicker:
            self.functionCount += 1
            goal = pygame.Rect(300,200,1300,360)
            self.drawScoreTime()
            self.background = self.soccerGoalFront
            for elem in self.goalieList:
                self.checkFeetCollison(elem)
                self.moveScoreBall(elem)
                ballRect = pygame.Rect(elem[0].x, elem[0].y, 35, 35)
                if self.timerCount % 5 == 0:
                    self.moveGoalie(elem)
                if self.collideGoalie(elem) == False and elem[2] <= 70 and ballRect.colliderect(goal):
                    self.score += 1
            self.drawKickerBall()
            self.drawGoalie()
            if self.timerFunc():
                self.soccerKicker = False

    def drawGoalie(self):
        goalie = pygame.image.load("goalie.png")
        self.drawG = goalie.get_rect(center=(self.goalieCoor))
        self._frame_surface.blit(goalie, self.drawG)

    def moveGoalie(self, ball):
        if ball[3] == True:
            self.goalieCoor[0] = random.uniform(ball[0].centerx - 300, ball[0].centerx + 300)
            self.goalieCoor[1] = random.uniform(ball[0].centery - 100, ball[0].centery + 100)
        else:
            self.goalieCoor[0] = ball[0].centerx
            self.goalieCoor[1] = ball[0].centery
        if self.goalieCoor[0] <= 300:
            self.goalieCoor[0] = 300
        elif self.goalieCoor[0] >= 1600:
            self.goalieCoor[0] = 1600
        if self.goalieCoor[1] <= 300:
            self.goalieCoor[1] = 300
        elif self.goalieCoor[1] >= self._infoObject.current_h / 2:
            self.goalieCoor[1] = self._infoObject.current_h / 2

    def collideGoalie(self, ball):
        bottom = self.drawG.bottom + 35
        top = self.drawG.top - 35
        left = self.drawG.left - 35
        right = self.drawG.right + 35
        if left <= ball[0].centerx <= right and top <= ball[0].centery <= bottom:
            return True
        return False
        
    def drawKickerBall(self):
        if len(self.goalieList) == 0:
            soccerx = self._infoObject.current_w / 2
            soccery = self._infoObject.current_h / 2
            scale = 150
            ball = pygame.image.load("soccer.png")
            drawBall = ball.get_rect(center=(soccerx, soccery))
            speed = 100
            hit = False
            self.goalieList.append([drawBall, ball, scale, hit, speed])
        for elem in self.goalieList:
            if elem[3] == True:
                if elem[2] <= 70 or elem[0].x < 0 or elem[0].x + elem[0].width > self._infoObject.current_w:
                    self.goalieList.remove(elem)
                if self.timerCount % 2 == 0:
                    elem[2] -= 3
            self._frame_surface.blit(pygame.transform.smoothscale(elem[1], [elem[2], elem[2]]), elem[0])
            pygame.display.flip()

    def moveScoreBall(self, ball):
        if ball[3] == True:
            direction = self.directionS() * 800
            if direction < 0:
                direction *= 1.5
            if ball[0].centery < ball[0].height:
                ball[4] = 0
            if ball[4] > 0:
                ball[0].centery -= 40
                ball[4] -= 1
            else:
                ball[0].centery += 10
            ball[0].centerx += direction
        else:
            if self.rightHandOpen == False:
                ball[0].centerx = self.rightHandCoor[0]
                ball[0].centery = self.rightAnkleCoor[1]


###########################################################################
   # SOCCER GOALIE MODE
###########################################################################
    def soccerGoalieMode(self):
        pass



###########################################################################
   # TENNIS MODE
###########################################################################
    def tennisMode(self):
        if self.tennis == True:
            self.score = 0
            self.timer = 30
            serveRect = pygame.Rect(20,100,500,200)
            targetRect = pygame.Rect(20, 700, 500, 200)
            AIRect = pygame.Rect(self._infoObject.current_w - 520,100,500,200)
            homeRect = pygame.Rect(self._infoObject.current_w - 520,700,500,200)
            leftRect = pygame.Rect(520, 700, 300, 200)
            rightRect = pygame.Rect(1020, 700, 300, 200)

            font = pygame.font.SysFont('Courier', 60, bold=True)
            
            pygame.draw.rect(self._frame_surface, (153, 255, 51), serveRect)
            serveText = "Serve Mode"
            serveDraw = font.render(serveText, True, (0, 0, 0) )
            self._frame_surface.blit(serveDraw, (50, 150))

            pygame.draw.rect(self._frame_surface, (0, 0, 0), targetRect)
            targetText = "Target Mode"
            targetDraw = font.render(targetText, True, (255, 255, 255) )
            self._frame_surface.blit(targetDraw, (50, 750))

            pygame.draw.rect(self._frame_surface, (255, 51, 51), AIRect)
            AIText = "AI Mode"
            AIDraw = font.render(AIText, True, (255, 255, 255) )
            self._frame_surface.blit(AIDraw, (self._infoObject.current_w - 500, 150))

            pygame.draw.rect(self._frame_surface, (255, 51, 51), homeRect)
            homeText = "Home"
            homeDraw = font.render(homeText, True, (255, 255, 255) )
            self._frame_surface.blit(homeDraw, (self._infoObject.current_w - 500, 750))
            
            pygame.draw.rect(self._frame_surface, (255, 51, 51), leftRect)
            leftText = "Lefty"
            leftDraw = font.render(leftText, True, (255, 255, 255) )
            self._frame_surface.blit(leftDraw, (570, 750))

            pygame.draw.rect(self._frame_surface, (255, 51, 51), rightRect)
            rightText = "Righty"
            rightDraw = font.render(rightText, True, (255, 255, 255) )
            self._frame_surface.blit(rightDraw, (1070, 750))
            
            text = "Use Right Hand To Select"
            ren = font.render(text, True, (127, 6, 119) )
            self._frame_surface.blit(ren, (400, 300))
            
            if serveRect.collidepoint(self.rightHandCoor):
                self.tennis = False
                self.tennisServe = True
                self.tennisTarget = False
                self.tennisAI = False
                self.starter = False
            if targetRect.collidepoint(self.rightHandCoor):
                self.tennis = False
                self.tennisServe = False
                self.tennisTarget = True
                self.tennisAI = False
                self.starter = False
            if AIRect.collidepoint(self.rightHandCoor):
                self.tennis = False
                self.tennisServe = False
                self.tennisTarget = False
                self.tennisAI = True
                self.starter = False

            if homeRect.collidepoint(self.rightHandCoor):
                self.tennis = False
                self.tennisServe = False
                self.tennisTarget = False
                self.tennisAI = False
                self.starter = True

            if leftRect.collidepoint(self.rightHandCoor):
                self.rightSide = False
                self.leftSide = True

            if rightRect.collidepoint(self.rightHandCoor):
                self.rightSide = True
                self.leftSide = False

###########################################################################
   # TENNIS TARGET MODE
###########################################################################
    # play tennis target mode
    def tennisTargetMode(self):
        if self.tennisTarget == True:
            self.drawScoreTime()
            self.drawRacket()
            self.drawTarget()
            for elem in self.tennisBallList:
                self.collideRacket(elem)
                self.moveTennisBall(elem)
                self.collideTarget(elem)
            self.drawTennisBall()
            self.timerFunc()


    # drawing the racket depending on the position of hand
    def drawRacket(self):
        #angle = self.getAngleWrist()
        # checks if player is righty or lefty
        if self.rightSide:
            wristX = self.rightWristCoor[0]
            wristY = self.rightWristCoor[1]
        elif self.leftSide:
            wristX = self.leftWristCoor[0]
            wristY = self.leftWristCoor[1]

        # checks which side racket is on according to their dominant hand
        # when hand is on the left side of body
        if self.rightSide and wristX < self.spineMidCoor[0] or self.leftSide and wristX <= self.spineMidCoor[0]:
            #rotated = pygame.transform.rotate(self.tennisRacket[1], 45)
            #self.tennisGrip = rotated.get_rect(midright = (wristX,wristY))
            #self._frame_surface.blit(rotated, self.tennisGrip)
            self.tennisGrip = self.tennisRacket[1].get_rect(bottomright=(wristX, wristY))
            self._frame_surface.blit(self.tennisRacket[1], self.tennisGrip)
        # when hand is on the right side of body
        elif self.rightSide and wristX >= self.spineMidCoor[0] or self.leftSide and wristX > self.spineMidCoor[0]:
            self.tennisGrip = self.tennisRacket[0].get_rect(bottomleft=(wristX, wristY))
            self._frame_surface.blit(self.tennisRacket[0], self.tennisGrip)


    def drawTennisBall(self):
        if len(self.tennisBallList) == 0:
            tennisBallx = random.uniform(self.tennisBallr, (self._infoObject.current_w - self.tennisBallr))
            tennisBally = random.uniform(self.tennisBallr, (self._infoObject.current_h - self.tennisBallr))
            tennisBallSpeed = random.randint(100, 200)
            scale = 150
            angle = 0
            hit = False
            ball = pygame.image.load("tennis.png")
            drawBall = ball.get_rect(center=(tennisBallx, tennisBally))
            self.tennisBallList.append([drawBall, ball, scale, hit, tennisBallSpeed, angle])
        for elem in self.tennisBallList:
            if elem[3] == False:
                if elem[2] <= 50:
                    self.tennisBallList.remove(elem)
                if self.timerCount % 2 == 0:
                    elem[2] -= 1
                self._frame_surface.blit(pygame.transform.smoothscale(elem[1], [elem[2], elem[2]]), elem[0])
            else:
                elem[5] = self.depthContact() / 5
                if elem[2] >= 150:
                    self.tennisBallList.remove(elem)
                if self.timerCount % 2 == 0:
                    elem[2] += 5
                elem[0].centerx += elem[5]
                self._frame_surface.blit(pygame.transform.smoothscale(elem[1], [elem[2], elem[2]]), elem[0])
            pygame.display.flip()


    def moveTennisBall(self, elem):
        if elem[3] == False:
            if elem[4] > 0:
                elem[0].centery -= 1
                elem[4] -= 1
            elif elem[4] == 0:
                elem[4] -= 1
            else:
                elem[0].centery += 5
                if elem[0].centery >= self._infoObject.current_h - elem[0].height / 2:
                    elem[0].centery = self._infoObject.current_h - elem[0].height / 2


    def collideRacket(self, elem):
        centerx = self.tennisGrip.center[0]
        centery = self.tennisGrip.center[1]
        width = self.tennisGrip.width / 2
        height = self.tennisGrip.height / 2
        bottom = centery + self.tennisBallr
        top = centery - height
        left = centerx - self.tennisBallr
        right = centerx + width + self.tennisBallr
        if left <= elem[0].centerx <= right and top <= elem[0].centery <= bottom:
            elem[3] = True


    def depthContact(self):
        if self.rightSide:
            distance = self.rightHandCoor[0] - self.spineMidCoor[0]
        return distance


###########################################################################
   # TENNIS SERVE MODE
###########################################################################

    def serveMode(self):
        if self.tennisServe:
            self.drawScoreTime()
            self.serveRacket()
            for elem in self.tennisBallList:
                if self.collideRacketServe(elem):
                    self.score += 1
                self.throwBall(elem)
            self.drawServeBall()
            if self.timerFunc():
                self.tennisServe = False


    def drawServeBall(self):
        if len(self.tennisBallList) == 0:
            tennisBallx = 0
            tennisBally = 0
            tennisBallSpeed = random.randint(100, 200)
            scale = 150
            hit = False
            ball = pygame.image.load("tennis.png")
            drawBall = ball.get_rect(center=(tennisBallx, tennisBally))
            self.tennisBallList.append([drawBall, tennisBallSpeed, ball, scale, hit])
        for elem in self.tennisBallList:
            if elem[4] == False:
                 elem[3] = 50
            self._frame_surface.blit(pygame.transform.smoothscale(elem[2], [elem[3], elem[3]]), elem[0])

    def collideRacketServe(self, elem):
        centerx = self.tennisGrip.center[0]
        centery = self.tennisGrip.center[1]
        width = self.tennisGrip.width / 2
        height = self.tennisGrip.height / 2
        bottom = centery + self.tennisBallr
        top = centery - height
        left = centerx - self.tennisBallr
        right = centerx + width + self.tennisBallr
        if left <= elem[0].centerx <= right and top <= elem[0].centery <= bottom:
            elem[4] = True
            self.tennisBallList.remove(elem)
            return True

    def serveRacket(self):
        #angle = self.getAngleWrist()
        # checks if player is righty or lefty
        if self.rightSide:
            wristX = self.rightWristCoor[0]
            wristY = self.rightWristCoor[1]
        elif self.leftSide:
            wristX = self.leftWristCoor[0]
            wristY = self.leftWristCoor[1]

        # checks which side racket is on according to their dominant hand
        # when hand is on the left side of body
        if self.leftSide:
            rotated = pygame.transform.rotate(self.tennisRacket[1], -45)
            self.tennisGrip = rotated.get_rect(midbottom = (wristX,wristY))
            self._frame_surface.blit(rotated, self.tennisGrip)
        # when hand is on the right side of body
        elif self.rightSide:
            rotated = pygame.transform.rotate(self.tennisRacket[1], -45)
            self.tennisGrip = rotated.get_rect(midbottom=(wristX, wristY))
            self._frame_surface.blit(rotated, self.tennisGrip)

    def throwBall(self, elem):
        if self.rightSide and self.leftHandOpen or self.leftSide and self.rightHandOpen:
            elem[0].centery -= 10
            if elem[0].centery <= 50:
                elem[0].centery += 15
        elif self.rightSide and self.leftHandOpen == False:
            elem[0].centerx = self.leftHandCoor[0] + elem[0].width / 2
            elem[0].centery = self.leftHandCoor[1] + elem[0].height / 2
        elif self.leftSide and self.rightHandOpen == False:
            elem[0].centerx = self.rightHandCoor[0] + elem[0].width / 2
            elem[0].centery = self.rightHandCoor[1] + elem[0].height / 2


###########################################################################
   # TENNIS AI MODE
###########################################################################



###########################################################################
   # COMMON FUNCTIONS
###########################################################################

        # check hand and circle collison
    def checkHandCollison(self):
        # find distance of line from center
        handRightx = self.rightHandCoor[0]
        handRighty = self.rightHandCoor[1]
        handLeftx = self.leftHandCoor[0]
        handLefty = self.leftHandCoor[1]
        
        # distance between left/right hand and the center of ball
        distanceRight = ((handRightx - self.ballx) ** 2 + (handRighty - self.bally) ** 2) ** 0.5
        distanceLeft = ((handLeftx - self.ballx) ** 2 + (handLefty - self.bally) ** 2) ** 0.5
        
        # checking distance less than circle radius
        if distanceRight <= self.ballr and distanceLeft <= self.ballr:   # when both hands collide
            self.collideRight = True
            self.collideLeft = True
        elif distanceRight <= self.ballr:   # only when right hand collides
            self.collideRight = True
            self.collideLeft = False
        elif distanceLeft <= self.ballr:   # only when left hand collides
            self.collideRight = False
            self.collideLeft = True
        else:                             # none collides
            self.collideRight = False
            self.collideLeft = False

    # checking hand open or close
    def checkHandOpen(self, jointPoints, body):
        if body.hand_right_state == HandState_Open:
            self.rightHandOpen = True
        else:
            self.rightHandOpen = False

        if body.hand_left_state == HandState_Open:
            self.leftHandOpen = True
        else:
            self.leftHandOpen = False
            self.leftHandOpen = False

    # moves the ball according to hand motions
    def moveBall(self, elem):
        # moving the ball around with hand motion
        self.checkHandCollison()
        if self.rightHandOpen == False and self.collideRight == True:
                self.ballx = int(self.rightHandCoor[0])
                self.bally = int(self.rightHandCoor[1])
        elif self.leftHandOpen == False and self.collideLeft == True:
                self.ballx = int(self.leftHandCoor[0])
                self.bally = int(self.leftHandCoor[1])
        elif self.rightHandOpen == False and self.leftHandOpen == False and self.collideRight == True and self.collideLeft == True:
                self.ballx = (int(self.rightHandCoor[0]) + int(self.leftHandCoor[0])) / 2
                self.bally = (int(self.rightHandCoor[1]) + int(self.leftHandCoor[1])) / 2
        else:
            self.bally += 1
            if self.bally + self.ballr >= self._infoObject.current_h:
                self.bally = self._infoObject.current_h - self.ballr
    

    def timerFunc(self):
            self.timerCount += 1
            if self.timerCount % 40 == 0:
                self.timer -= 1
                if self.timer <= 0:
                    self.timer = 0
                    self.gameOver = True
                    return True

    def drawScoreTime(self):
        font = pygame.font.SysFont('Courier', 60, bold=True)
        text = "Score: %d" %(self.score)
        ren = font.render(text, True, (127, 6, 119) )
        self._frame_surface.blit(ren, (self._infoObject.current_w - 400, 50))

        text = "Timer: %d" %(self.timer)
        ren = font.render(text, True, (127, 6, 119) )
        self._frame_surface.blit(ren, (self._infoObject.current_w - 400, 100))



    def gameOverMode(self):
        if self.gameOver:
            font = pygame.font.SysFont('Courier', 100, bold=True)
            text = "Score: %d" %(self.score)
            ren = font.render(text, True, (127, 6, 119) )
            self._frame_surface.blit(ren, (self._infoObject.current_w / 2, 50))

            homeRect =  pygame.Rect(self._infoObject.current_w / 2 - 250, 700, 500, 200)
            pygame.draw.rect(self._frame_surface, (0, 0, 0), homeRect)
            homeText = "Home"
            homeDraw = font.render(homeText, True, (255, 255, 255) )
            self._frame_surface.blit(homeDraw, (self._infoObject.current_w / 2 - 250, 750))

            if homeRect.collidepoint(self.rightHandCoor):
                self.gameOver = False
                self.__init__()    # newly added part to initialize everything
                self.starter = True




###########################################################################
   # DEPTH
###########################################################################
    # checks player's depth and add reminders
    def checkingDepth(self):
        if self.depthOfPlayer < 1.7:
            font = pygame.font.SysFont('Courier', 100, bold=True)
            text = "Back Up"
            draw = font.render(text, True, (132, 186, 91) )
            self._frame_surface.blit(draw, (100, 950))

        elif self.depthOfPlayer > 2.2:
            font = pygame.font.SysFont('Courier', 150, bold=True)
            text = "Too Far"
            draw = font.render(text, True, (132, 186, 91) )
            self._frame_surface.blit(draw, (100, 950))


        #color = pygame.color.THECOLORS["red"] 
        #pygame.draw.circle(self._frame_surface, color, (int(self.rightHandCoor[0]), int(self.rightHandCoor[1])), 100)

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                    
            # --- Game logic should go here


            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)   # this line puts in the camera
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue 
                    
                    joints = body.joints 
                    # convert joint coordinates to color space 
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i])
                    self.checkHandOpen(joint_points, body)   # checking handstates
            
            self.checkingDepth()

            # starter screen 
            self.starterMode()

            # all soccer modes
            self.soccerMode()
            self.soccerTargetMode()
            if self.soccerKicker and self.functionCount == 0:
                self.init(soccerKicker=True)
            self.soccerKickerMode()
            if self.soccerGoalie and self.functionCount == 0:
                self.init(soccerGoalie=True)
            self.soccerGoalieMode()

            # all tennis modes
            self.tennisMode()
            self.serveMode()
            self.tennisTargetMode()
            
            # end screen
            self.gameOverMode()


            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None

            # --- Go ahead and update the screen with what we've drawn.
            if self.soccerGoalie or self.soccerKicker or self.tennisAI:
                self._frame_surface.blit(self.background, (0,0))
            pygame.display.flip()   # simply updates the screen
            
            # --- Limit to 60 frames per second
            self._clock.tick(100)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()   # unintializes all of pygame modules (end and clean game)

__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime();
game.run();
