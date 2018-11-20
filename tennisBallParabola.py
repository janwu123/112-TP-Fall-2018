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


class BodyGameRuntime(object):
    def __init__(self):
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

        # Kinect runtime object, we want only color and body frames 
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
        self.ballx = (self._infoObject.current_w)//2
        self.bally = (self._infoObject.current_h)//2
        self.ballr = ballsRect.height / 2
        self.soccer = False

        # characteristics of tennis racket and balll
        self.tennisRacket = [pygame.image.load("rightTennis.png"), pygame.image.load("leftTennis.png")]
        self.tennisGrip = ''   # location of the grip to hold
        self.rightSide = False  
        self.leftSide = True
        self.tennisBall = pygame.image.load("tennis.png").convert_alpha()
        self.tennisBallRect = self.tennisBall.get_rect()
        self.tennisBallr = self.tennisBallRect.height / 2   # original radius of the ball
        self.tennisBallList = []   # list of all tennis balls
        self.tennis = True

        self.timerCount = 0

        self.depthOfPlayer = 0
        


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
        elif joint0 == PyKinectV2.JointType_HandTipRight:
            self.rightTipCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        elif joint0 == PyKinectV2.JointType_WristRight:
            self.rightWristCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        elif joint0 == PyKinectV2.JointType_ElbowRight:
            self.rightElbowCoor = (jointPoints[joint0].x, jointPoints[joint0].y)

        if joint0 == PyKinectV2.JointType_HandLeft:
            self.leftHandCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        elif joint0 == PyKinectV2.JointType_HandTipLeft:
            self.leftTipCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        elif joint0 == PyKinectV2.JointType_WristLeft:
            self.leftWristCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        elif joint0 == PyKinectV2.JointType_ElbowLeft:
            self.leftElbowCoor = (jointPoints[joint0].x, jointPoints[joint0].y)

        if joint0 == PyKinectV2.JointType_AnkleRight:
            self.rightAnkleCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        if joint0 == PyKinectV2.JointType_FootRight:
            self.rightFootCoor = (jointPoints[joint0].x, jointPoints[joint0].y)

        if joint0 == PyKinectV2.JointType_AnkleLeft:
            self.leftAnkleCoor = (jointPoints[joint0].x, jointPoints[joint0].y)
        if joint0 == PyKinectV2.JointType_FootLeft:
            self.leftFootCoor = (jointPoints[joint0].x, jointPoints[joint0].y)

        if joint0 == PyKinectV2.JointType_SpineMid:
            self.depthOfPlayer = joints[getattr(PyKinectV2, "JointType_SpineMid")].Position.z
            self.spineMidCoor = (jointPoints[joint0].x, jointPoints[joint0].y)


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
   # SOCCER MODE
###########################################################################
    # run the soccer mode
    def soccerMode(self):
        if self.soccer == True:
            self.moveBall()
            self.moveFootBall()
            self.drawBall()

    # drawing the ball (graphics) it will later be a picture
    def drawBall(self):
        ballsCenter = self.balls.get_rect(center=(self.ballx, self.bally))
        self._frame_surface.blit(self.balls, ballsCenter)

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
    def moveBall(self):
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
                
    # check foot collision 
    def checkFeetCollison(self):
        rightCoorx = self.rightAnkleCoor[0]
        rightCoory = self.rightAnkleCoor[1]
        leftCoorx = self.leftAnkleCoor[0]
        leftCoory = self.leftAnkleCoor[1]

        distanceRight = ((rightCoorx - self.ballx) ** 2 + (rightCoory - self.bally) ** 2) ** 0.5
        distanceLeft = ((leftCoorx - self.ballx) ** 2 + (leftCoory - self.bally) ** 2) ** 0.5

        if distanceRight <= self.ballr or distanceLeft <= self.ballr:
            self.collideFoot = True
        else:
            self.collideFoot = False

    # soccer ball moved with foot
    def moveFootBall(self):
        self.checkFeetCollison()
        # rewrite shrinking part (shrink the picture)
        if self.collideFoot == True:
           #self.ballr += 5
            if self.ballr >= 100:
                self.ballr = 100
        else:
            #self.ballr -= 1
            if self.ballr == 0:
                self.ballr = 100

###########################################################################
   # TENNIS MODE
###########################################################################
    # play tennis mode
    def tennisMode(self):
        if self.tennis == True:
            self.timerCount += 1
            self.drawRacket()
            for elem in self.tennisBallList:
                if self.collideRacket(elem):
                    self.tennisBallList.remove(elem)
                self.moveTennisBall(elem)
            self.drawTennisBall()
            pygame.display.update()

    def getAngleWrist(self):
        angle = 0
        if rightSide == True:
            smallY = self.rightWristCoor[1] - self.rightHandCoor[1]
            smallX = self.rightWristCoor[0] - self.rightHandCoor[0]
            smallAngle = math.atan(smallY / smallX)
            longY = self.rightWristCoor[1] - self.rightElbowCoor[1]
            longX = self.rightWristCoor[0] - self.rightElbowCoor[0]
            longAngle = math.atan(longY / longX)
        else:
            smallY = self.leftWristCoor[1] - self.leftHandCoor[1]
            smallX = self.leftWristCoor[0] - self.leftHandCoor[0]
            smallAngle = math.atan(smallY / smallX)
            longY = self.leftWristCoor[1] - self.leftElbowCoor[1]
            longX = self.leftWristCoor[0] - self.leftElbowCoor[0]
            longAngle = math.atan(longY / longX)

        if smallAngle > longAngle:
            angle = smallAngle - longAngle
        else:
            angle = longAngle - smallAngle

        return angle



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
        if self.timerCount % 70 == 0:
            tennisBallx = random.uniform(self.tennisBallr, (self._infoObject.current_w - self.tennisBallr))
            tennisBally = random.uniform(self.tennisBallr, (self._infoObject.current_h - self.tennisBallr))
            tennisBallSpeed = random.randint(100, 200)
            scale = 150
            ball = pygame.image.load("tennis.png")
            drawBall = ball.get_rect(center=(tennisBallx, tennisBally))
            self.tennisBallList.append([drawBall, tennisBallSpeed, ball, scale])
        for elem in self.tennisBallList:
            if elem[3] <= 50:
                self.tennisBallList.remove(elem)
            if self.timerCount % 2 == 0:
                elem[3] -= 1
            self._frame_surface.blit(pygame.transform.smoothscale(elem[2], [elem[3], elem[3]]), elem[0])
            pygame.display.flip()

    def moveTennisBall(self, elem):
        if elem[1] > 0:
            elem[0].centery -= 1
            elem[1] -= 1
        elif elem[1] == 0:
            elem[1] -= 1
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
            return True
        return False




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
                self.draw_color_frame(frame, self._frame_surface)
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

            # playing different modes
            self.soccerMode()
            self.tennisMode()


            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()   # simply updates the screen
            
            # --- Limit to 60 frames per second
            self._clock.tick(70)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()   # unintializes all of pygame modules (end and clean game)

__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime();
game.run();
