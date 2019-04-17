'''
* Team Id :         1873
* Author List :     PALLAV BHALLA, MOHAMMAD AZHARUDDIN, VINIT NARAYAN JHA, SAURAV KUMAR
* Filename:         Task_final.py
* Theme:            Thirsty Crow(TC)
* Functions:        getCameraMatrix(), main(), init_gl(), resize(), drawGLScene(), detect_markers(), draw_background(), 
*                    overlay(), serial_send_data(), serial_receive_data(), crow_flying_with_stone(), crow_flying_without_stone(), 
*                    crow_picking_stone(), crow_dropping_stone(), transmit_data() and internal functions of 'serial', 'time' 
*                    and 'openGL' library.
* Global Variables: texture_object, texture_background, crow, stone3, stone3_removed, stone5, stone5_removed,
*                    stone6, stone6_removed, pitcher_low, pitcher_mid, pitcher_high, pitcher_full, crow, crow_full_up_wings,
*                    crow_full_down_wings, crow_with_wings_slightly_downwards, crow_with_wings_slightly_up, crow_stone, 
*                    crow_full_up_wings_stone, crow_full_down_wings_stone, crow_with_wings_slightly_downwards_stone, 
*                    crow_with_wings_slightly_up_stone, crow_down_000001, crow_down_000002, crow_down_000003, crow_down_000004,
*                    crow_down_000005, crow_down_000006, crow_down_000007, crow_down_000008, crow_down_000009, crow_down_000010, 
*                    crow_up_stone_000001, crow_up_stone_000002, crow_up_stone_000003, crow_up_stone_000004, crow_up_stone_000005, 
*                    crow_up_stone_000006, crow_up_stone_000007, crow_up_stone_000008, crow_up_stone_000009, crow_up_stone_000010, 
*                    value_returned, pitcher, pebble, transmit_count, counter. 
*  
'''

import serial
import time
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL import Image
import pygame
from objloader import * # For opening the OBJ files of blender models to be projected.

texture_object = None
texture_background = None
camera_matrix = None
dist_coeff = None
pitcher_low = None
pitcher_mid = None
pitcher_high = None
pitcher_full = None
stone3 = None
stone3_removed = None
stone5 = None
stone5_removed = None
stone6 = None
stone6_removed = None
crow = None
crow_full_up_wings = None
crow_full_down_wings = None
crow_with_wings_slightly_downwards = None
crow_with_wings_slightly_up = None
crow_stone = None
crow_full_up_wings_stone = None
crow_full_down_wings_stone = None
crow_with_wings_slightly_downwards_stone = None
crow_with_wings_slightly_up_stone = None
crow_down_000001 = None
crow_down_000002 = None
crow_down_000003 = None
crow_down_000004 = None
crow_down_000005 = None
crow_down_000006 = None
crow_down_000007 = None
crow_down_000008 = None
crow_down_000009 = None
crow_down_000010 = None
crow_up_stone_000001 = None
crow_up_stone_000002 = None
crow_up_stone_000003 = None
crow_up_stone_000004 = None
crow_up_stone_000005 = None
crow_up_stone_000006 = None
crow_up_stone_000007 = None
crow_up_stone_000008 = None
crow_up_stone_000009 = None
crow_up_stone_000010 = None
counter = 0

value_returned = None
pitcher = None
pebble1 = None
pebble2 = None
pebble3 = None
thirsty_crow = None

cap = cv2.VideoCapture(1) # '0' - for using the system inbuilt camera
                          # '1' - for using the external camera

ser = serial.Serial("COM13", 9600, timeout=0.005) #object that helps to access functions related to serial library


INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [ 1.0, 1.0, 1.0, 1.0]])

############################### ARENA CONFIGURATION DICTIONARY ######################################
Robot_start = "START-1" # Defining the robot start position
arena_config = {3: ("Water Pitcher", 7, "3-3"), 
                5:("Pebble", 17, "2-2"), 
                0:("Pebble", 2, "1-1"), 
                2:("Pebble", 15, "3-3")}    
                # Dictionary containing the arena configuration.
                
###################################################################################################

# Generalising the data to be sent to the bot containing the information about the objects
# position using serial communication with X-Bee.

#dict_keys = arena_config.keys()                     # Extracting the keys from the arena config dictionary in the form of a list
#print(dict_keys)

dict_keys = []
pairs = list(arena_config.items())
#print(pairs)
for i in pairs:
    if i[1][0]=="Water Pitcher":
        dict_keys.append(i[0])
for i in pairs:
    if i[1][0]=="Pebble":
        dict_keys.append(i[0])
#print(dict_keys)

"""
* Collecting the required data that is necessary for arena traversal into separate variables.
* Converting the data from int to string for transmitting it to the bot through serial communication using XBee.
* Wherever required, we have also converted the data from a single digit number to a 2-digit number for generalising
  the code for any configuration as required by the corresponding bot traversal code written in C. 
"""
flag_variable = len(dict_keys)

if flag_variable == 4:
    
    Water_Pitcher = arena_config[dict_keys[0]][0]
    Water_Pitcher_Cell = str(arena_config[dict_keys[0]][1]) # type conversion of the data containing the cell no. from int to string.
    if len(Water_Pitcher_Cell) == 1:
        Water_Pitcher_Cell = '0'+ Water_Pitcher_Cell        # Converting the cell no. data into a 2 digit number(for e.g. "9" is converted to "09")
    Water_Pitcher_Axes = arena_config[dict_keys[0]][2]

    Stone1 = arena_config[dict_keys[1]][0] + "1"
    Stone1_Cell = str(arena_config[dict_keys[1]][1])         # type conversion of the data containing the cell no. from int to string.
    if len(Stone1_Cell) == 1:
        Stone1_Cell = '0'+ Stone1_Cell
    Stone1_Axes = arena_config[dict_keys[1]][2]

    Stone2 = arena_config[dict_keys[2]][0] + "2"
    Stone2_Cell = str(arena_config[dict_keys[2]][1])        # type conversion of the data containing the cell no. from int to string.
    if len(Stone2_Cell) == 1:
        Stone2_Cell = '0'+ Stone2_Cell
    Stone2_Axes = arena_config[dict_keys[2]][2]
    Stone3 = arena_config[dict_keys[3]][0] + "3"

    Stone3_Cell = str(arena_config[dict_keys[3]][1])        # type conversion of the data containing the cell no. from int to string.
    if len(Stone3_Cell) == 1:
        Stone3_Cell = '0'+ Stone3_Cell
    Stone3_Axes = arena_config[dict_keys[3]][2]

    #data_to_transmit = Robot_start + Water_Pitcher + Water_Pitcher_Cell + Water_Pitcher_Axes + Stone1 + Stone1_Cell + Stone1_Axes + Stone2 + Stone2_Cell + Stone2_Axes + Stone3 + Stone3_Cell + Stone3_Axes

    transmit_count = 0

elif flag_variable ==3:
    Water_Pitcher = arena_config[dict_keys[0]][0]
    Water_Pitcher_Cell = str(arena_config[dict_keys[0]][1]) # type conversion of the data containing the cell no. from int to string.
    if len(Water_Pitcher_Cell) == 1:
        Water_Pitcher_Cell = '0'+ Water_Pitcher_Cell        # Converting the cell no. data into a 2 digit number(for e.g. "9" is converted to "09")
    Water_Pitcher_Axes = arena_config[dict_keys[0]][2]

    Stone1 = arena_config[dict_keys[1]][0] + "1"
    Stone1_Cell = str(arena_config[dict_keys[1]][1])         # type conversion of the data containing the cell no. from int to string.
    if len(Stone1_Cell) == 1:
        Stone1_Cell = '0'+ Stone1_Cell
    Stone1_Axes = arena_config[dict_keys[1]][2]

    Stone2 = arena_config[dict_keys[2]][0] + "2"
    Stone2_Cell = str(arena_config[dict_keys[2]][1])        # type conversion of the data containing the cell no. from int to string.
    if len(Stone2_Cell) == 1:
        Stone2_Cell = '0'+ Stone2_Cell
    Stone2_Axes = arena_config[dict_keys[2]][2]

    Stone3 = "#######"
    Stone3_Cell = "#"        # type conversion of the data containing the cell no. from int to string.
    if len(Stone3_Cell) == 1:
        Stone3_Cell = '#'+ Stone3_Cell
    Stone3_Axes = "###"

    #data_to_transmit = Robot_start + Water_Pitcher + Water_Pitcher_Cell + Water_Pitcher_Axes + Stone1 + Stone1_Cell + Stone1_Axes + Stone2 + Stone2_Cell + Stone2_Axes + Stone3 + Stone3_Cell + Stone3_Axes

    transmit_count = 0

elif flag_variable == 2:
   
    Water_Pitcher = arena_config[dict_keys[0]][0]
    Water_Pitcher_Cell = str(arena_config[dict_keys[0]][1]) # type conversion of the data containing the cell no. from int to string.
    if len(Water_Pitcher_Cell) == 1:
        Water_Pitcher_Cell = '0'+ Water_Pitcher_Cell        # Converting the cell no. data into a 2 digit number(for e.g. "9" is converted to "09")
    Water_Pitcher_Axes = arena_config[dict_keys[0]][2]

    Stone1 = arena_config[dict_keys[1]][0] + "1"
    Stone1_Cell = str(arena_config[dict_keys[1]][1])         # type conversion of the data containing the cell no. from int to string.
    if len(Stone1_Cell) == 1:
        Stone1_Cell = '0'+ Stone1_Cell
    Stone1_Axes = arena_config[dict_keys[1]][2]

    Stone2 = "#######"
    Stone2_Cell = "#"        # type conversion of the data containing the cell no. from int to string.
    if len(Stone2_Cell) == 1:
        Stone2_Cell = '#'+ Stone2_Cell
    Stone2_Axes = "###"

    Stone3 = "#######"
    Stone3_Cell = "#"        # type conversion of the data containing the cell no. from int to string.
    if len(Stone3_Cell) == 1:
        Stone3_Cell = '#'+ Stone3_Cell
    Stone3_Axes = "###"

    #data_to_transmit = Robot_start + Water_Pitcher + Water_Pitcher_Cell + Water_Pitcher_Axes + Stone1 + Stone1_Cell + Stone1_Axes + Stone2 + Stone2_Cell + Stone2_Axes + Stone3 + Stone3_Cell + Stone3_Axes

    transmit_count = 0


#print(Robot_start, Water_Pitcher, Water_Pitcher_Cell, Water_Pitcher_Axes, Stone1, Stone1_Cell, Stone1_Axes, Stone2, Stone2_Cell, Stone2_Axes, Stone3, Stone3_Cell, Stone3_Axes)

    

'''
*Function Name : getCameraMatrix()
*Input:          None
*Output:         camera_matrix, dist_coeff
*Logic:          Loads the camera calibration file provided and returns the camera and
*                distortion matrix saved in the calibration file.
*Example Call:     getCameraMatrix()
*
'''
def getCameraMatrix():
        global camera_matrix, dist_coeff
        with np.load('System.npz') as X:
                camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

'''
*Function Name : transmit_data()
*Input:          None
*Output:         transmit_count
*Logic:          For calibrating the start of bot traversal and the animation in openGL window.
*                 This had to be done since the objloader(object loader) script takes a lot of time in 
*                 importing the required .mtl and .obj files. Using the normal procedure, the bot traversal 
*                 starts first and then after some delay, the openGL window pops up. In order to make a better 
*                 animation, we had to calibrate the start of the bot traversal and OpenGL animation.
*                 The data containing the arena configuration is transmitted only once using this function throughout 
*                 the execution of python script.
*                 The normal animation is projected until the character '@' received from the bot through serial 
*                 communication using X-Bee.
*                 On receiving '@', the python script sends the arena config data until transmit_count is not 1.
*                 This data is not sent again, since "transmit_count = 1" and (transmit_count == 0) = 0, i.e.,False.
*                 We call this function multiple times inside openGL main loop, which repeats forever, so as to minimize 
*                 the probability of not receiving the "@". The arena config data is sent to the bot only after receiving "@".
*                 Hence it becomes important, to call this. 
*Example Call:   transmit_data()
'''
def transmit_data():

        global transmit_count    
        if transmit_count == 0:
            x = ser.read(1).decode()
            print(x) 
            """ Send the arena config information/data only after receiving "@" from the bot. The bot sends "@", the moment it 
                is switched "ON"."""  
            if x == '@':
                #ser.write("#".encode())
                a = ser.read(1).decode()
                while a == '@':
                    a = ser.read(1).decode()
                    # do nothing
                """Sending the data serially to the bot using X-Bee serial communication. """                             
                ser.write(Robot_start.encode()) 
                ser.write(Water_Pitcher.encode())
                ser.write(Water_Pitcher_Cell.encode())
                ser.write(Water_Pitcher_Axes.encode())
                ser.write(Stone1.encode())
                ser.write(Stone1_Cell.encode())
                ser.write(Stone1_Axes.encode())
                ser.write(Stone2.encode())
                ser.write(Stone2_Cell.encode())
                ser.write(Stone2_Axes.encode())
                ser.write(Stone3.encode())
                ser.write(Stone3_Cell.encode())
                ser.write(Stone3_Axes.encode())
                
                transmit_count = 1
                print(transmit_count)


############# Main Function and Initialisations ########################
'''
*Function Name : main()
*Input:          None
*Output:         None
*Logic:          Run the "serial_receive_data" function until a specific character (here '@') is received.
*                Send the data regarding the arena configuration using the function "serial_send_data". 
*                This portion of the code upto this point runs only once.Initialises OpenGL window and 
*                callback functions. Then starts the event processing loop.
**Example Call:  main()
'''       
def main():

        glutInit()
        getCameraMatrix()
        glutInitWindowSize(640, 480)
        glutInitWindowPosition(671, 88)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)
        window_id = glutCreateWindow("OpenGL")
        init_gl()
        glutDisplayFunc(drawGLScene)
        glutIdleFunc(drawGLScene)
        glutReshapeFunc(resize)
        glutMainLoop()


########################################################################       
       
'''
*Function Name : init_gl()
*Input:          None
*Output:         None
*Logic:          Initialises various parameters related to OpenGL scene.
*Example Call:   init_gl()
*
''' 
def init_gl():
        global texture_object, texture_background
        global stone3
        global stone3_removed
        global stone5
        global stone5_removed
        global stone6
        global stone6_removed
        global pitcher_low
        global pitcher_mid
        global pitcher_high
        global pitcher_full
        global crow
        global crow_full_up_wings
        global crow_full_down_wings
        global crow_with_wings_slightly_downwards
        global crow_with_wings_slightly_up
        global crow_stone
        global crow_full_up_wings_stone
        global crow_full_down_wings_stone
        global crow_with_wings_slightly_downwards_stone
        global crow_with_wings_slightly_up_stone
        global crow_down_000001
        global crow_down_000002
        global crow_down_000003
        global crow_down_000004
        global crow_down_000005
        global crow_down_000006
        global crow_down_000007
        global crow_down_000008
        global crow_down_000009
        global crow_down_000010
        global crow_up_stone_000001
        global crow_up_stone_000002
        global crow_up_stone_000003
        global crow_up_stone_000004
        global crow_up_stone_000005
        global crow_up_stone_000006
        global crow_up_stone_000007
        global crow_up_stone_000008
        global crow_up_stone_000009
        global crow_up_stone_000010


        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)  
        glMatrixMode(GL_MODELVIEW)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_TEXTURE_2D)                        
        texture_background = glGenTextures(1)
        texture_object = glGenTextures(1)
        crow = OBJ('crow.obj', swapyz=True)  # for loading the 'crow.obj', Wavefront OBJ file. 
        pitcher_low = OBJ('pitcher_low.obj', swapyz=True)  # for loading the 'pitcher_low.obj', Wavefront OBJ file.
        pitcher_mid = OBJ('pitcher_mid.obj', swapyz=True)  # for loading the 'pitcher_mid.obj', Wavefront OBJ file.
        pitcher_high = OBJ('pitcher_high.obj', swapyz=True)  # for loading the 'pitcher_high.obj', Wavefront OBJ file. 
        pitcher_full = OBJ('pitcher_full.obj', swapyz=True)  # for loading the 'pitcher_full.obj', Wavefront OBJ file.
        stone3 = OBJ('stone3.obj', swapyz=True)
        stone3_removed = OBJ('stone3_removed.obj', swapyz=True)
        stone5 = OBJ('stone5.obj', swapyz=True)
        stone5_removed = OBJ('stone5_removed.obj', swapyz=True)
        stone6 = OBJ('stone6.obj', swapyz=True)
        stone6_removed = OBJ('stone6_removed.obj', swapyz=True)
        crow_stone = OBJ('crow_stone.obj', swapyz=True)
        crow_full_up_wings = OBJ('crow_full_up_wings.obj', swapyz=True)
        crow_full_up_wings_stone = OBJ('crow_full_up_wings_stone.obj', swapyz=True)
        crow_full_down_wings = OBJ('crow_full_down_wings.obj', swapyz=True)
        crow_full_down_wings_stone = OBJ('crow_full_down_wings_stone.obj', swapyz=True)
        crow_with_wings_slightly_up = OBJ('crow_with_wings_slightly_up.obj', swapyz=True)
        crow_with_wings_slightly_up_stone = OBJ('crow_with_wings_slightly_up_stone.obj', swapyz=True)
        crow_with_wings_slightly_downwards = OBJ('crow_with_wings_slightly_downwards.obj', swapyz=True)
        crow_with_wings_slightly_downwards_stone = OBJ('crow_with_wings_slightly_downwards_stone.obj', swapyz=True)
        crow_down_000001= OBJ('crow_down_000001.obj', swapyz=True)
        crow_down_000002= OBJ('crow_down_000002.obj', swapyz=True)
        crow_down_000003= OBJ('crow_down_000003.obj', swapyz=True)
        crow_down_000004= OBJ('crow_down_000004.obj', swapyz=True)
        crow_down_000005= OBJ('crow_down_000005.obj', swapyz=True)
        crow_down_000006= OBJ('crow_down_000006.obj', swapyz=True)
        crow_down_000007= OBJ('crow_down_000007.obj', swapyz=True)
        crow_down_000008= OBJ('crow_down_000008.obj', swapyz=True)
        crow_down_000009= OBJ('crow_down_000009.obj', swapyz=True)
        crow_down_000010= OBJ('crow_down_000010.obj', swapyz=True)
        crow_up_stone_000001= OBJ('crow_up_stone_000001.obj', swapyz=True)
        crow_up_stone_000002= OBJ('crow_up_stone_000002.obj', swapyz=True)
        crow_up_stone_000003= OBJ('crow_up_stone_000003.obj', swapyz=True)
        crow_up_stone_000004= OBJ('crow_up_stone_000004.obj', swapyz=True)
        crow_up_stone_000005= OBJ('crow_up_stone_000005.obj', swapyz=True)
        crow_up_stone_000006= OBJ('crow_up_stone_000006.obj', swapyz=True)
        crow_up_stone_000007= OBJ('crow_up_stone_000007.obj', swapyz=True)
        crow_up_stone_000008= OBJ('crow_up_stone_000008.obj', swapyz=True)
        crow_up_stone_000009= OBJ('crow_up_stone_000009.obj', swapyz=True)
        crow_up_stone_000010= OBJ('crow_up_stone_000010.obj', swapyz=True)

'''
*Function Name : resize()
*Input:          w->width
                 h->height
*Output:         None
*Logic:          Initialises the projection matrix of OpenGL scene
*Example Call:     resize(w,h)
*
'''
def resize(w,h):
        ratio = 1.0* w / h
        glMatrixMode(GL_PROJECTION)
        glViewport(0,0,w,h)
        gluPerspective(45, ratio, 0.1, 100.0)


'''
*Function Name : crow_flying_without_stone()
*Input:          None
*Output:         Returns the respective gl_list for a particular blender model, that is to be projected.
*Logic:              This function is used for the projection and animation of
*                 crow without stone in it's mouth. In this a global variable with the name of
*                 counter is initialized and 8 frames are projected one after the other in the 
*                 form of animation and every time the counter is increazed by one.
*Example Call:   crow_flying_without_stone()
*   
'''
def crow_flying_without_stone():
        global counter
        if counter%8 == 0:
                time.sleep(0.05)
                counter = counter + 1
                return crow_full_down_wings.gl_list
        elif counter%8 == 1:
                time.sleep(0.05)
                counter = counter + 1
                return crow_with_wings_slightly_downwards.gl_list
        elif counter%8 == 2:
                time.sleep(0.05)
                counter = counter + 1
                return crow.gl_list
        elif counter%8 == 3:
                time.sleep(0.05)
                counter = counter + 1
                return crow_with_wings_slightly_up.gl_list
        elif counter%8 == 4:
                time.sleep(0.05)
                counter = counter + 1
                return crow_full_up_wings.gl_list
        elif counter%8 == 5:
                time.sleep(0.05)
                counter = counter + 1
                return crow_with_wings_slightly_up.gl_list
        elif counter%8 == 6:
                time.sleep(0.05)
                counter = counter + 1
                return crow.gl_list
        elif counter%8 == 7:
                time.sleep(0.05)
                counter = counter + 1
                return crow_with_wings_slightly_downwards.gl_list
   
'''
*Function Name : crow_flying_with_stone()
*Input:          None
*Output:          Returns the respective gl_list for a particular blender model, that is to be projected.
*Logic:            This function is used for the projection and animation of
*                  crow with stone in it's mouth.In this a global variable with the name of
*                  counter is initialized and 8 frames are projected one after the other in the 
*                  form of animation and every time the counter is increazed by one.
*Example Call:   crow_flying_with_stone()
*
'''
def crow_flying_with_stone():
        global counter
        if counter%8 == 0:
                time.sleep(0.05)
                counter = counter + 1
                return crow_full_down_wings_stone.gl_list
        elif counter%8 == 1:
                time.sleep(0.05)
                counter = counter + 1
                return crow_with_wings_slightly_downwards_stone.gl_list
        elif counter%8 == 2:
                time.sleep(0.05)
                counter = counter + 1
                return crow_stone.gl_list
        elif counter%8 == 3:
                time.sleep(0.05)
                counter = counter + 1
                return crow_with_wings_slightly_up_stone.gl_list
        elif counter%8 == 4:
                time.sleep(0.05)
                counter = counter + 1
                return crow_full_up_wings_stone.gl_list
        elif counter%8 == 5:
                time.sleep(0.05)
                counter = counter + 1
                return crow_with_wings_slightly_up_stone.gl_list
        elif counter%8 == 6:
                time.sleep(0.05)
                counter = counter + 1
                return crow_stone.gl_list
        elif counter%8 == 7:
                time.sleep(0.05)
                counter = counter + 1
                return crow_with_wings_slightly_downwards_stone.gl_list

"""
*Function Name : crow_picking_stone()
*Input:          None
*Output:          Returns the respective gl_list for a particular blender model, that is to be projected.
*Purpose:          This function is used for the animation of crow, when the crow is picking
*                  the stone. In this a global variable with the name of counter is initialized
*                  and 20 frames are projected one after the other, in which first 10 frames describes
*                  crow going down to pick the stone and the next 10 frames contains crow coming up with
*                  the stone. After each frame the value of counter is increased by one.
*Example Call:   crow_picking_stone()
*"""
def crow_picking_stone():
        global counter
        if counter%20 == 0:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000001.gl_list
        elif counter%20 == 1:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000002.gl_list
        elif counter%20 == 2:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000003.gl_list
        elif counter%20 == 3:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000004.gl_list
        elif counter%20 == 4:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000005.gl_list
        elif counter%20 == 5:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000006.gl_list
        elif counter%20 == 6:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000007.gl_list
        elif counter%20 == 7:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000008.gl_list
        elif counter%20 == 8:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000009.gl_list
        elif counter%20 == 9:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000010.gl_list
        elif counter%20 == 10:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000010.gl_list
        elif counter%20 == 11:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000009.gl_list
        elif counter%20 == 12:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000008.gl_list
        elif counter%20 == 13:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000007.gl_list
        elif counter%20 == 14:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000006.gl_list
        elif counter%20 == 15:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000005.gl_list
        elif counter%20 == 16:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000004.gl_list
        elif counter%20 == 17:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000003.gl_list
        elif counter%20 == 18:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000002.gl_list
        elif counter%20 == 19:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000001.gl_list

"""
*Function Name : crow_dropping_stone()
*Input:          None
*Output:         Returns the respective gl_list for a particular blender model, that is to be projected.
*Logic:          This function is used for the animation of crow, when the crow is dropping
*                  the stone. In this a global variable with the name of counter is initialized
*                  and 20 frames are projected one after the other, in which first 10 frames describes
*                  crow going down to drop the stone and the next 10 frames contains crow coming up without
*                  the stone. After each frame the value of counter is increased by one.
*Example Call:   crow_dropping_stone()
*
"""
def crow_dropping_stone():
        global counter
        if counter%20 == 0:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000001.gl_list
        elif counter%20 == 1:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000002.gl_list
        elif counter%20 == 2:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000003.gl_list
        elif counter%20 == 3:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000004.gl_list
        elif counter%20 == 4:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000005.gl_list
        elif counter%20 == 5:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000006.gl_list
        elif counter%20 == 6:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000007.gl_list
        elif counter%20 == 7:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000008.gl_list
        elif counter%20 == 8:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000009.gl_list
        elif counter%20 == 9:
                time.sleep(0.05)
                counter = counter + 1
                return crow_up_stone_000010.gl_list
        elif counter%20 == 10:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000010.gl_list
        elif counter%20 == 11:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000009.gl_list
        elif counter%20 == 12:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000008.gl_list
        elif counter%20 == 13:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000007.gl_list
        elif counter%20 == 14:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000006.gl_list
        elif counter%20 == 15:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000005.gl_list
        elif counter%20 == 16:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000004.gl_list
        elif counter%20 == 17:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000003.gl_list
        elif counter%20 == 18:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000002.gl_list
        elif counter%20 == 19:
                time.sleep(0.05)
                counter = counter + 1
                return crow_down_000001.gl_list


'''
*Function Name : drawGLScene()
*Input:          None
*Output:         None
*Logic:          It is the main callback function which is called again and
*                again by the event processing loop. In this loop, the webcam frame
*                is received and set as background for OpenGL scene. ArUco marker is
*                detected in the webcam frame and 3D model is overlayed on the marker
*                by calling the overlay() function. The respective 3D model is projected
*                 upon receiving a particular character through serial communication using XBee.
*                 The serial_receive_data() function solves this purpose of receiving the data 
*                 through serial communication from the bot using X-Bee modules.
*Example Call:   drawGLScene()
*
'''
def drawGLScene():
        transmit_data() # checking whether the variable "transmit_count==0" equals "True" or "False"
                        # 1.) if True, then check if the value received serially is"@" or not. If "@" is received, 
                        #      then send the arena config data and assign transmit_count=1, so that, this data is not sent again.
                        # 2.) if False, do nothing and continue with the OpenGL animation.
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        ar_list = []
        ret, frame = cap.read()
        if ret == True:
                draw_background(frame)
                glMatrixMode(GL_MODELVIEW)
                glLoadIdentity()
                ar_list, frame = detect_markers(frame)
                serial_receive_data()  # receiving the data serially using the "serial_receive_data" function.
                                       # This uses the serial module to communicate serially between two X-Bees.

                for i in ar_list:
                        if i[0] == dict_keys[0]:
                            if pitcher == 'm':                                        #project this pitcher after the bot drops the first pebble into it.
                                overlay(frame, ar_list, i[0], pitcher_low.gl_list)    # (when 'm' is received through serial communication)

                            elif pitcher == 'h':                                    #project this pitcher after the bot drops the second pebble into it. 
                                overlay(frame, ar_list, i[0], pitcher_high.gl_list) # (when 'h' is received through serial communication)

                            elif pitcher == 'f':                                    #project this pitcher after the bot drops the third pebble into it.
                                overlay(frame, ar_list, i[0], pitcher_full.gl_list) # (when 'f' is received through serial communication)

                            else:                                                    #project this pitcher before the bot drops any pebble into it.
                                overlay(frame, ar_list, i[0], pitcher_low.gl_list)

                        if i[0] == dict_keys[1]:
                            if pebble1 == 'p':                                          # project this pebble after the bot picks the first pebble. 
                                overlay(frame, ar_list, i[0], stone3_removed.gl_list)    # (when 'p' is received through serial communication)
                            else:
                                overlay(frame, ar_list, i[0], stone3.gl_list)             #project this pebble before the bot picks the first pebble. 

                        if i[0] == dict_keys[2]:
                            if pebble2 == 'q':                                          #project this pebble after the bot picks the second pebble. 
                                overlay(frame, ar_list, i[0], stone5_removed.gl_list)    # (when 'q' is received through serial communication)
                            else:                                                         
                                overlay(frame, ar_list, i[0], stone5.gl_list)            #project this pebble before the bot picks the second pebble.

                        if i[0] == dict_keys[3]:
                            if pebble3 == 'r':                                          #project this pebble after the bot picks the third pebble. 
                                overlay(frame, ar_list, i[0], stone6_removed.gl_list)    # (when 'r' is received through serial communication)
                            else:                                                         
                                overlay(frame, ar_list, i[0], stone6.gl_list)            #project this pebble before the bot picks the third pebble.

                        if i[0] == 10:
                            if thirsty_crow == 'w':                                         # project this crow-animation while the bot travels from pebble
                                overlay(frame, ar_list, i[0], crow_flying_with_stone())    # to pitcher.(when 'w' is received through serial communication)

                            elif thirsty_crow == 'u':                                        # project this crow-animation while the bot picks up the pebble.
                                overlay(frame, ar_list, i[0], crow_picking_stone())           #(when 'u' is received through serial communication)

                            elif thirsty_crow == 'd':                                        # project this crow-animation while the bot drops up the pebble.
                                overlay(frame, ar_list, i[0], crow_dropping_stone())       #(when 'd' is received through serial communication)

                            elif thirsty_crow == 'n':                                        # project this crow-animation while the bot traverses without a pebble.
                                overlay(frame, ar_list, i[0], crow_flying_without_stone()) #(when 'n' is received through serial communication)

                            elif thirsty_crow == 'z':                                        # project this crow model after the bot completes it traversal.
                                overlay(frame, ar_list, i[0], crow.gl_list)                   #(when 'z' is received through serial communication)
                            else:                                                    
                                overlay(frame, ar_list, i[0], crow_flying_without_stone())

                transmit_data() # checking whether the variable "transmit_count==0" equals "True" or "False"
                                # 1.) if True, then check if the value received serially is"@" or not. If "@" is received, 
                                #      then send the arena config data and assign transmit_count=1, so that, this data is not sent again.
                                # 2.) if False, do nothing and continue with the OpenGL animation.
                cv2.imshow('frame', frame)
                cv2.waitKey(1)
        glutSwapBuffers()
       
########################################################################
######################## Aruco Detection Function ######################
'''
*Function Name : detect_markers()
*Input:          img (numpy array)
*Output:         aruco list in the form [(aruco_id_1, centre_1, rvec_1, tvec_1),(aruco_id_2,
*                centre_2, rvec_2, tvec_2), ()....] and image.
*Logic:          This function takes the image in form of a numpy array, camera_matrix and
*                distortion matrix as input and detects ArUco markers in the image. For each
*                ArUco marker detected in image, paramters such as ID, centre coord, rvec
*                and tvec are calculated and stored in a list in a prescribed format. The list
*                is returned as output for the function
*Example Call:   detect_markers(img)
*
'''
def detect_markers(img):
        transmit_data() # checking whether the variable "transmit_count==0" equals "True" or "False"
                        # 1.) if True, then check if the value received serially is"@" or not. If "@" is received, 
                        #      then send the arena config data and assign transmit_count=1, so that, this data is not sent again.
                        # 2.) if False, do nothing and continue with the OpenGL animation.
        markerLength = 100
        aruco_list = []
        xaxis = []
        yaxis = []
        centre = []
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        img = aruco.drawDetectedMarkers(img, corners, ids)  # detect the sruco markers and display its aruco id.
        k=len(corners)
        for j in range(k):

            xaxis.append(((corners[j][0][0][0])+(corners[j][0][2][0]))/2)
            yaxis.append(((corners[j][0][0][1])+(corners[j][0][2][1]))/2)
            centre.append([xaxis[j],yaxis[j]])

        rvec, tvec, _= aruco.estimatePoseSingleMarkers(corners,  markerLength, camera_matrix, dist_coeff)
        for b in range(k):
            aruco_list.append((ids[b] , centre[b] , rvec[b] , tvec[b]))   
        return aruco_list, img

##################################################################
    
 
'''
*Function Name : draw_background()
*Input:          img (numpy array)
*Output:         None
*Logic:          Takes image as input and converts it into an OpenGL texture. That
*                OpenGL texture is then set as background of the OpenGL scene
*Example Call:   draw_background()
*
'''
def draw_background(image):                          #covert into gl texture and put it on quadilatedral and set as background

        transmit_data() # checking whether the variable "transmit_count==0" equals "True" or "False"
                        # 1.) if True, then check if the value received serially is"@" or not. If "@" is received, 
                        #      then send the arena config data and assign transmit_count=1, so that, this data is not sent again.
                        # 2.) if False, do nothing and continue with the OpenGL animation.

        # convert into opengl texture
        bg_image = cv2.flip(image, 0)
        bg_image = Image.fromarray(bg_image)    
        ix = bg_image.size[0]
        iy = bg_image.size[1]
        bg_image = bg_image.tobytes('raw', 'BGRX', 0, -1)

        # create background texture, OpenGL texture is then set as background of the OpenGL scene
        glBindTexture(GL_TEXTURE_2D,texture_background)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)

        # draw background
        glBindTexture(GL_TEXTURE_2D,texture_background)
        glPushMatrix()
        glTranslatef(0.0,0.0,-7.0)
        #glutSolidTeapot(0.5)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0); glVertex3f(-4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 4.0,  3.0, 0.0)
        glTexCoord2f(0.0, 0.0); glVertex3f(-4.0,  3.0, 0.0)
        glEnd( )
        glPopMatrix()

        return None


'''
*Function Name : overlay()
*Input:          img (numpy array), aruco_list, aruco_id, model_file (gl_list of the required model file)
*Output:         None
*Logic:          Receives the ArUco information as input and overlays the 3D Model of the blender objects
*                on the ArUco marker. That ArUco information is used to calculate the rotation matrix and
*                subsequently the view matrix. Then that view matrix is loaded as current matrix and the 
*                3D model is rendered. The "model_file" is related to the gl_list parameter, necessary to 
*                 call the corresponding blender model to be projected.
*Example Call:   overlay(img, ar_list, ar_id, model_file)
*
'''
def overlay(img, ar_list, ar_id, model_file):
        transmit_data() # checking whether the variable "transmit_count==0" equals "True" or "False"
                        # 1.) if True, then check if the value received serially is"@" or not. If "@" is received, 
                        #      then send the arena config data and assign transmit_count=1, so that, this data is not sent again.
                        # 2.) if False, do nothing and continue with the OpenGL animation.
        for x in ar_list:
                if ar_id == x[0]:
                        centre, rvec, tvec = x[1], x[2], x[3]
        rmtx = cv2.Rodrigues(rvec)[0]
        y_axis_translation = (tvec[0][1]/(210))        # Here the tvec value of Y-axis is scaled down so as to project the blender model correctly over the aruco marker, placed on the arena  
        x_axis_translation = tvec[0][0]/210            # Here the tvec value of X-axis is scaled down so as to project the blender model correctly over the aruco marker, placed on the arena 
        error = y_axis_translation - 1.11            # This calibrates the error which occurs while projecting the model on the arena in Y-axis.
        if tvec[0][1] > 90:                              # The behaviour of the projected model in Y-axis is slightly different since the value of error is different below 90 
                y_axis_translation = error      # and above 90(this is the value of tvec[0][1]) and hence different condition is applied while calibrating and making the error negligble.
                
        elif tvec[0][1] <= 90:
                y_axis_translation = error - (((tvec[0][1] - 23) * 0.0012) - 0.12)    # For values of tvec[0][1] below 90, this equation gives the corresponding value of error at particulat tvec value in 
                                                                                         #Y-axis, this error is reduced to zero with the help of this equation.
                

        view_matrix = np.array([[rmtx[0][0],rmtx[0][1],rmtx[0][2],x_axis_translation-(((tvec[0][0])/976.62)-0.19)],      #For values of tvec[0][0] in X-axis, thisequation gives the corresponding value of error at                                                                                              #X-axis,and then that error is made zero with the help of this equation.
                                [rmtx[1][0],rmtx[1][1],rmtx[1][2],y_axis_translation],                                   #particular tvec value in X-axis,and then that error is reduced to zero with the help of this equation.
                                [rmtx[2][0],rmtx[2][1],rmtx[2][2],6], 
                                [0.0       ,0.0       ,0.0       ,1.0    ]])

        view_matrix = view_matrix * INVERSE_MATRIX
        view_matrix = np.transpose(view_matrix)
        #glBindTexture(GL_TEXTURE_2D,texture_object)
        #init_object_texture(texture_file)
        glPushMatrix()
        glLoadMatrixd(view_matrix)
        glCallList(model_file)    #  The model_file parameter corresponds to the gl_list for displaying a 
                                  #     particular blender model.
        glPopMatrix()

'''
*Function Name : serial_send_data()
*Input:          data
*Output:         None
*Logic:          Takes the data to be transmitted as input and serially writes it using 
*                inbuilt functions of 'serial' library.
*Example Call:   serial_send_data() 
*
'''
def serial_send_data(data):
        ser.write(data.encode())
    
'''
*Function Name : serial_receive_data()
*Input:          None
*Output:         value_returned, pitcher, pebble
*Logic:          Receives the data serially using the inbuilt function of 'serial' library.
*Example Call:   serial_receive_data()
*
'''
def serial_receive_data():
    global value_returned
    global pitcher
    global pebble1, pebble2, pebble3
    global thirsty_crow
    #ser = serial.Serial("COM9", 9600, timeout=0.005)  #object that helps to access functions related to serial library 
    value_returned = (ser.read(1)).decode()
                      # value_returned : Store the value returned by Atmega microcontroller.
                      # 'ser.read' reads/recieves the value transmitted by Atmega microcontroller.
    if ((value_returned == 'h') or (value_returned == 'f') or (value_returned == 'm')):
        pitcher = value_returned    # "pitcher" variable stores the data that is received serially and is related to the pitcher 
                                    # models(as defined in "drawGLScene()" function)

    if (value_returned == 'p'):
        pebble1 = value_returned    # "pebble1" variable stores the data that is received serially and is related to pebble1 
                                    # models(as defined in "drawGLScene()" function)

    if (value_returned == 'q'):
        pebble2 = value_returned    # "pebble2" variable stores the data that is received serially and is related to pebble2 
                                    # models(as defined in "drawGLScene()" function)


    if (value_returned == 'r'):
        pebble3 = value_returned    # "pebble3" variable stores the data that is received serially and is related to pebble3 
                                    # models(as defined in "drawGLScene()" function)


    if ((value_returned == 'w') or (value_returned == 'u') or (value_returned == 'd') or (value_returned == 'n') or (value_returned == 'z')):  
        thirsty_crow = value_returned           # w -> crow flying with stone
                                                # u -> crow picking up stone
                                                # d -> crow dropping stone
                                                # n -> crow flying without stone
                                                # z -> crow settled



########################################################################
if __name__ == "__main__":
        main()