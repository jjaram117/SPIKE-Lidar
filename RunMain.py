import serial, time, struct
import csv
import math
import matplotlib.pyplot as plt
from operator import itemgetter 
import statistics
import os
    
Start_Scan = b"\xA5\x20" #Begins scanning
Force_Scan = b"\xA5\x21" #Overrides anything preventing a scan
Health = b"\xA5\x52" #Returns the state of the Lidar
Stop_Scan = b"\xA5\x25" #Stops the scan
RESET = b"\xA5\x40" #Resets the device
Motor = b"\xA5\xF0" #Sets motor speed
MotorFast = b'\xa5\xf0\x02\x94\x02\xc1'
Motor0 = b"\xA5\xF0\x02\x00\x00\x57" #A5F0 0200 0057
Rate = b'\xA5\x59'


def RunSPIKE(DesAng, DesDist): #Function communicates movement commands to the SPIKE. "rotAng" is rotation angle for the robot, "distMove" is the distance to move
    command = 'python3 /home/pi/Desktop/Lidar/SPIKE-Lidar/RunSPIKE.py ' + str(DesAng) + ' ' + str(DesDist) #Extra space string needed to distinguish between arguments
    os.system(command)

def chksm(payload):
    cksm = 0
    for elem in payload:
        cksm ^= elem
    return struct.pack('=B',cksm)

def serialComm(cmd, num= 0):
    payload = cmd + chksm(cmd)
    ser.write(payload)
    print(payload)
    reply = b''
    for i in range(num):
        reply += ser.readline()
    if num > 0:
        print(reply)
    return reply

def translate(payload):
    quality = payload[0]>>2
    S = payload[0] & 0b1
    notS = payload[0] & 0b10
    C = payload[1] & 0b1
    angle = struct.unpack('<H',payload[1:3])[0]/2/64  # divide by 2 gets rid of C bit
    dist = struct.unpack('<H',payload[3:5])[0]/4000
    return quality,S,notS,C,angle,dist

def GrabPts(num):
    run = 0

    #Preallocation for our 4 variables
    x = [0]*num
    y = [0]*num
    a = [0]*num
    d = [0]*num

    #Wiping the buffer
    ser.flushInput()
    ser.flushOutput()

    #Run Lidar and save desired number of angle and distance points to a file
    while run < num:
        reading = ser.read(5)   
        try: 
            (quality, S, notS, C, angle, dist) = translate(reading)
        except Exception as e:
            print(e) 
            continue 

        if quality > 10:
            #Eliminating noise calculated to be at distance (d) = 0
            if dist == 0:
                continue
            else:
                #Append polar coordinates to their running variables
                a[run] = (angle)
                d[run] = (dist)
                
                #Converting from polar to cartesian coordinates
                ang = angle*3.14/180 #radians to degrees
                x_calc = dist*math.cos(ang) 
                y_calc = dist*math.sin(ang)

                #Append cartesian coordinates to their running variables
                x[run] = (x_calc)
                y[run] = (y_calc)

                #Update Run variable
                run += 1
    return (x,y,a,d)

def TransformData(x,y, DesAng, DesDist): #Implements matrix and linear transforms when detecting physical movement
    #Preallocation for the transformed values
    x_transf = [0]*len(x)
    y_transf = [0]*len(y)

    if DesAng != 0: #Rotational transforms for all points based on the Desired Angle to rotate input
        global lidar_ang
        lidar_ang = lidar_ang + DesAng #Should update the global variable tracking rotation relative to starting point
        
        #convert angle to radians
        DesAng_R = lidar_ang*0.0174533

        for i in range(len(x)): #Calculate all of the transforms 
            x_transf[i] = x[i]*math.cos(DesAng_R) + y[i]*math.sin(DesAng_R)*-1
            y_transf[i] = x[i]*math.sin(DesAng_R) + y[i]*math.cos(DesAng_R)

        if lidar_ang > 360: #Reducing the lidar_ang to within [-360, 360] to calculate with smaller values
            lidar_ang = lidar_ang - 360

        elif lidar_ang < 360: 
            lidar_ang = lidar_ang + 360

        else:
            pass 

        #print(lidar_ang)
    else:
        pass

    #if DesDist != 0: #Linear transforms for all points based on the Desired Distance input

    return x_transf, y_transf


def TransformAutoData(x,y, DesAng, DesDist): #Implements matrix and linear transforms when detecting physical movement
    #Preallocation for rotational transformed values
    x_transf = [0]*len(x)
    y_transf = [0]*len(y)

    global lidar_ang, x_move, y_move #Call in the global variables
    lidar_ang = lidar_ang + DesAng #Should update the global variable tracking rotation relative to starting point
    
    #Reducing the lidar_ang to within [-360, 360] to calculate with smaller values
    if lidar_ang >= 360: 
        lidar_ang = lidar_ang - 360

    elif lidar_ang <= -360: #Shouldn't actually enter this path unless a more efficient turning system us put in place
        lidar_ang = lidar_ang + 360

    else:
        pass

    #Convert current global angle to radians
    DesAng_R = lidar_ang*0.0174533

    #Update how much movement has occurred in x and y relative to the starting point            
    x_move = x_move + math.cos(DesAng_R)*DesDist
    y_move = y_move + math.sin(DesAng_R)*DesDist

    for i in range(len(x)): #Apply first rotational then linear transforms to all values 
        x_transf[i] = x[i]*math.cos(DesAng_R) + y[i]*math.sin(DesAng_R)*-1
        x_transf[i] = x_transf[i] + x_move
        
        y_transf[i] = x[i]*math.sin(DesAng_R) + y[i]*math.cos(DesAng_R)
        y_transf[i] = y_transf[i] + y_move

    
    return x_transf, y_transf

def plotter(x,y):
#Creating circular outline to represent lidar shape and direction being faced     
    
    circle1 = plt.Circle((0, 0), 0.05, color='r', fill=False) 
    ax = plt.gca()
    ax.add_artist(circle1)
    plt.arrow(0,0,0.07,0, shape='full', lw=.5, length_includes_head=True, head_width=.015, color='r') #Arrow to show lidar direction 
    
    #Plotting data
    plt.scatter(x,y)
    #plt.plot(x,y)
    plt.draw()

    #Formatting plot
    plt.title('Lidar Mapping Data')
    plt.ylabel('Y-axis')
    plt.xlabel('X-axis')
    plt.axis('equal') #Equalizing the axis ratios
    
#    axisScale = 1.3 #variable to change the "zoom" of the plot that's graphed
    
#    xmin = min(x)
#    xmax = max(x)
#    ymin = min(y)
#    ymax =max(y)
#    xrange = xmax-xmin
#    yrange = ymax-ymin
#    
    #ax.set_xlim([-xmax*axisScale,xmax*axisScale]) #Setting axis limits based on data. Keeps plot consistent after inverting axis
    #ax.set_ylim([-ymax*axisScale,ymax*axisScale])

    #ax.set_xlim([-xrange/2*axisScale,xrange/2*axisScale]) #Setting axis limits based on data. Keeps plot consistent after inverting axis
    #ax.set_ylim([-yrange/2*axisScale,yrange/2*axisScale])
    
    plt.gca().invert_yaxis() #Flips y axis to match physical orientation to the lidar

    #plt.show()
    print("Your points have been plotted\n")
    
    plt.show()
    #plt.ioff()
    
    #plt.show()
    #plt.show()
    
    #plt.ion()
#    plt.pause(0.001)
    time.sleep(.5)

def LidarComm(): #Compact method of starting the communication with the Lidar
    serialComm(Start_Scan)
    success = False

    while not success:  
        while ser.read() != b'\xA5':
            time.sleep(0.01)
        reply = ser.read(6)
        if (reply == b'\x5A\x05\x00\x00\x40\x81'):
            print('starting...')
            success = True
        else:
            success = False
            print('incorrect reply')



def MedianDist(a,d,tol):
    ##-90 degree distance calculations-##
    Ind_90 = [i for i, x in enumerate(a) if x >= 90-tol and x <= 90+tol] #Gives indices of all values near 90 degrees within the tolerance "tol". ie if tol=2 the range is from 88 to 92
    
    medDis_90 = statistics.median(itemgetter(*Ind_90)(d)) #"itemgetter" gives list of values in "a" for the calculated indices in "ind". "statistics.median()"" calculates the median of those values and gives the median distance
    time.sleep(.1)

    ##-0 degree distance calculations-##
    Ind_0 = [i for i, x in enumerate(a) if x >= 360-tol] #or x <= 0+tol]

    medDis_0 = statistics.median(itemgetter(*Ind_0)(d))
    time.sleep(.1)

    return (medDis_90, medDis_0)


def SaveData():
    name = str(input('What would you like to name your file? (Do not inlude the file type, just the file name)\n')) #
    
    #Saving all data to the named .csv file
    filename = "/home/pi/Desktop/Lidar/SPIKE-Lidar" + name + ".csv" #Rename folder address as desired 
    with open(filename, 'a', newline='') as f:
            writer = csv.writer(f, delimiter = ',')
            writer.writerows(zip(x,y,a,d))
    time.sleep(.1)

    print('Your file has been saved as', name,'\n')
    time.sleep(2)

def CalcMove(a, d): #Uses the data from GrabPts to determine what angle to rotate to and distance to travel
    DistThresh = .5 #Minimum distance threshold for function to base calculations. Distance in meters  
    AngStep = 30 #Step size for determining which measured distances are farther away

    #Attempt to calculate indices. Otherwise, return a "False" Flag 
    try:
        indx = [idx for idx, val in enumerate(d) if val > DistThresh] #Gives indices of list "d" where values are above DistThresh
        AngThresh = list(itemgetter(*indx)(a)) #Zips elements of "a" with indices in "indx" to give list of all angles above DistThresh. Sorts in ascending order as well

    except:
        print("SUPREME FAILURE")
        success = False
        DesAng = 0 
        DesDist = 0

        return success, DesAng, DesDist

    
    
    ##----Iterate to find where most, farthest clusters are
    i = 0 #Increments based on the value of AngStep
    LoopCount = 0 #Counter variable indicating how many times we've iterated through the loop  
    NumMaxElements = 0 #Tracks what the highest number of elements was between all loo iterations
    MaxLoop = 0 #Variable updated with the version of the loop determined to have the highest number of values

    while i < 360: #While loop will analyze AngThresh values by chunks to determine where the largest cluster of distances are
        CurrElements = len(list(x for x in AngThresh if i < x <= i+AngStep)) #Determines how many values in the list fall within the angle range of AngStep
        LoopCount += 1

        if CurrElements >= NumMaxElements: #If greater than or equal to, update NumMaxElements, keep track of loop iteration
            NumMaxElements = CurrElements #Update max elements
            MaxLoop = LoopCount #Update which version of the loop we were counted in
            
            i += AngStep

        else: #If not greater than, just move on to next step size
            i += AngStep

    #print("MaxLoop =", MaxLoop)
    #print("LoopCount =", LoopCount)

    DesAng = round((MaxLoop*AngStep)-(AngStep/2)) #Calculates what angle we want to rotate to

    IndxDist = [i for i, x in enumerate(a) if x <= DesAng+(AngStep/2) and x >= DesAng-(AngStep/2)] #Use AngStep to determine the upper and lower half indices we're looking for
    MedDist = statistics.median(itemgetter(*IndxDist)(d)) #Calculates the median distance from the index values

    DesDist = round(0.75*MedDist,1) #Distance we want SPIKE to travel. Scaled down to avoid crashing

    print("DesAng =", DesAng)
    print("DesDist =", DesDist)

    success = True
    return success, DesAng, DesDist


    
def opt1(): #Request movement information, then Grab Data
    global x,y,a,d

    print('    \nPositive angles rotate clockwise, negatives counterclockwise. Positive values move forwards, negatives move backwards.     \nUnits are in degrees and meters, respectively.     \nThe SPIKE will rotate first, then move.')
    time.sleep(1)
    
    DesAng = float(input('\n    How much would you like to rotate? (90, 80, 167, etc.)\n'))
    DesDist = float(input('\n    How much would you like to move? (1, -3, 0.5, etc.)\n'))
    num = int(input("\n    How many points do you want?\n"))
    
    time.sleep(0.1)
    RunSPIKE(DesAng, DesDist) #Send movement command via terminal to the SPIKE
    time.sleep(0.2)
    
    LidarComm()

    (x,y,a,d) = GrabPts(num)

    serialComm(Stop_Scan)
    ser.read(ser.inWaiting()) #Flush the buffer

    #Determine if data needs to be transformed prior to plotting 
    if DesAng == 0 and DesDist ==0: #No need to go through Calculations
        plotter(x,y)

        #print("Your points have been plotted\n")
    
    else:
        x_transf,y_transf = TransformData(x,y,DesAng, DesDist)
        plotter(x_transf,y_transf)
        print("Your points have been plotted\n")


def opt2(): #Close Current figure
    print('Are you sure you want to clear the current figure? y/n\n')
    act = input()

    if act.lower() == 'y':
        plt.close()
    
    elif act.lower() == 'n':
        print('\nPlot will not be cleared')
        time.sleep(0.5)

    else:
        print("Not an available option\n")

def opt3(): #Save current data to a .csv file
    SaveData()


def opt4(): #Calculate distance after move
    num = int(input("How many points do you want? The more points (i.e. 1500+) the better\n"))

    tol = int(input("What degree tolerance do you want? (The higher the tolerance the better. i.e: 7)\n"))

    ##--Collecting the first set of points--##
    print('Now collecting your first set of points\n\n')
    LidarComm()
    
    (x,y,a,d) = GrabPts(num) #Grab all the points. x,y used for the plots, a,d used for the median distance calculations
    
    serialComm(Stop_Scan)
    ser.read(ser.inWaiting())
    time.sleep(.5)

    plotter(x,y) #Plotting the data

    (med1_90,med1_0) = MedianDist(a,d,tol) #Assigns first pair of calculated medians to med1_90 and med1_0
    time.sleep(.5)

    print('Your first distance at 90 has been calculated to be =', "%.3f" % med1_90, '\nYour first distance at 0 has been calculated to be =', "%.3f" % med1_0)
    time.sleep(2)


    ##--Moving the SPIKE
    DesAng = float(input('\n    How many degrees to rotate?\n'))
    DesDist = float(input('\n    How much to move linearly?\n'))
    
    time.sleep(0.1)
    RunSPIKE(DesAng, DesDist) #Send movement command via terminal to the SPIKE
    print("\n    Done moving\n")


    ##--Collecting the Second set of points--##
    print('\nPlease move your sensor to the new location. Press "Enter/Return" when ready to continue\n')
    time.sleep(2.5)
    input("Press Enter to continue...\n")

    #Collecting the second set of points:
    print('Now collecting your second set of points')
    LidarComm()

    (x,y,a,d) = GrabPts(num)

    serialComm(Stop_Scan)
    ser.read(ser.inWaiting())
    time.sleep(.5)

    plotter(x,y)

    (med2_90,med2_0) = MedianDist(a,d,tol) #Assigns second pair of calculated medians to med2_90 and med2_0
    
    #Calculating the differences between the calculated distances
    distMoved_90 = med1_90-med2_90
    distMoved_0 = med1_0-med2_0  

    print('Your second distance at 90 has been calculated to be =', "%.3f" % med2_90, '\nYour second distance at 0 has been calculated to be =', "%.3f" % med2_0)
    time.sleep(2)
    
    #Printing out the final results of the estimated moves
    print('\n   Your estimated total distance moved in the 90 direction is =', "%.3f" % distMoved_90, 'meters')
    print('\n   Your estimated total distance moved in the 0 direction is =', "%.3f" % distMoved_0, 'meters')
    time.sleep(4)

def opt5(): #End program and stop lidarx
    print('\nAre you sure you want to end the program? y/n\n')
    act = input()

    if act.lower() == 'y':
        print("\nNow exiting program...")
        serialComm(Motor0)
        exit()
    
    elif act.lower() == 'n':
        pass

    else:
        print("Not an available option\n")

def opt6(): #Automated Mode
    cycles = int(input("How many cycles would you like to go through? (Please only enter integers)\n"))

    global lidar_ang, x_move, y_move #used to track how much we've rotated and moved relative to the starting point
    lidar_ang = 0 
    x_move = 0 
    y_move = 0

    x_globalCoord = [] #Running variables where that we're appending the data from each cycle to and will ultimately filter through and plot
    y_globalCoord = []



    for i in range(cycles):
        success = False

        while not success:
            #Lidar intialization and data grab
            LidarComm()
            (x,y,a,d) = GrabPts(2000)
            serialComm(Stop_Scan)
            ser.read(ser.inWaiting()) #Flush the buffer

            success, DesAng, DesDist = CalcMove(a, d) #How much to rotate and move, based on the 
            
            if not success: #"CalcMove" function calculates a failure, will reattempt 
                print('Reattempting Calculations')
                continue

            print("\nWill rotate %f Degrees" % DesAng)
            print("Will travel %f meters forward" % DesDist)

            (x_transf, y_transf) = TransformAutoData(x,y, DesAng, DesDist) #After each successful GrabPts instance, tranform all points accordingly

            #Updating the global coordinate lists for final plotting (After transformed correctly)
            x_globalCoord = x_globalCoord + a
            y_globalCoord = y_globalCoord + d


            RunSPIKE(DesAng, DesDist)
            time.sleep(6)

    print("\nNow generating your global plot...\n")
    plotter(x_globalCoord, y_globalCoord)


##----------------------------SETUPS----------------------------##
port = "/dev/ttyUSB0" #Likely COM Port when plugged into a Pi

ser = serial.Serial(port, 115200, timeout = 1)
ser.setDTR(False)

serialComm(RESET,3)
serialComm(Health,1)
serialComm(Rate,1)

#Set Motor speed
speed = 800  # between 1 and 1023
motorSpeed = Motor + b'\x02'+struct.pack('<H',speed)  # 2 byte payload, little endian of the desired speed
serialComm(motorSpeed)
time.sleep(2) #Motor spin up

#Initializing localization variables to keep track of lidar location relative to starting point of 0,0. Will globalize
global lidar_ang
lidar_ang = 0

inp = 0 #setting up the variable for the input




##----------------------------MAIN LOOP----------------------------##
while True:
    inp = input('    \nOPTIONS TABLE: What would you like to do? \n1 = Move SPIKE and Grab Data \n2 = Close Current Figure \n3 = Save current (x,y,a,d) data to named file \n4 = For Testing: Calculate distance after move \n5 = Stop Lidar and end program \n6 = Run Automated Mode \n\n')
    time.sleep(.5)

    if inp == '1': #Request lidar to collect data
        opt1()        

    if inp == '2': #Close Current figure
        opt2()

    elif inp == '3': #Copy current "currData" file to a new local .csv file 
        opt3()

    elif inp == '4': #Calibration purposes: Lidar point calculations after move
        opt4()
        
    elif inp == '5': #Stop lidar and end the program 
        opt5()
    
    elif inp == '6': #Automated Mode
        opt6()

    else:
        print("Returning you to Options Table\n")
        time.sleep(1)
