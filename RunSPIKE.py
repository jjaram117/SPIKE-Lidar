import serial, time, math
import sys

#Functions having to do with serial communication
def startSerial(): #Creates serial connection to the port the SPIKE is plugged into I'm thinking that others will call this function
    global ser
    ser = serial.Serial('/dev/ttyACM0', 9600)
    ser.write(b'\x03') #Stops the previous program that may have been running. #Necessary elsewhere?
    
    while ser.inWaiting(): #Clear out the buffer
        ser.read(ser.inWaiting())
        time.sleep(.1)
        
    ser.write(b'import hub\r\n') #May as well import here the only library we'll be using
    ser.write(b'p = hub.port.A.motor.pair(hub.port.B.motor)\r\n')

def endSerial(): #End the communciation with the SPIKE. Likely only when the program is turned off
    ser.close()

def NecSetups(): #Sends commands to the SPIKE REPL necessary setups: libraries, motorpair setup
    ser.write(b'import hub\r\n') 
    ser.write(b'p = hub.port.A.motor.pair(hub.port.B.motor)') #Switch out the 'A' and 'B' depending on the ports motors are attached to

#Physical build functions
def Convert(DesDist, diam, calibVal): #Conversion from Desired Distance (DesDist) to number of rotations (NumRot) to number of degrees(NumDegrees) motors need to spin
                                      #Also passes through a calibVal to more easily adjust LinearSPIKE
    
    NumRot = DesDist/(math.pi*diam) #Divide DesDist by circumference of the wheels to determine necessary rotations to travel distance
    NumDegrees = round(NumRot*360*calibVal) #How many degrees motors will rotate for to travel desired distance. Only takes in whole Numbers (positive or negative), no decimals allowed
    return NumDegrees

def RotVari(diam, WheelDist): #Calculates number of rotations to achieve 360 EV3 spin. Based on wheel diameter and distance between the wheels
    Circumf = math.pi*diam #Calculate circumference of the wheels
    TurnCircumf = WheelDist*math.pi #Circumference of the circle the EV3 wheels travel upon when rotating in place 
    NeccRot = TurnCircumf/Circumf #Rotations required for to achieve full 360 rotation of circle in "TurnCircumf"  

    return NeccRot

def DistCommand(DesDist, NumDegrees): #Used to detect if the desired distance is positive, negative, or 0. Returns string to be executed
    degVal = str(NumDegrees) #Converting our calculated degrees into a string value
    
    if DesDist > 0: 
        pwmSend = str(-1*pwmVal) #Set PWM value so both motors move forward
        
        command = 'p.run_for_degrees(' + degVal + ',' + pwmSend + ',-' + pwmSend + ')\r\n' #Concatenate all strings. ie "b'p.run_for_degrees(360,55,-55)\r\n'"
        commandSend = str.encode(command) #Encode command into bytes
        return commandSend
    
    elif DesDist < 0:
        pwmSend = str(pwmVal) #Set PWM value so both motors move backward
        
        command = 'p.run_for_degrees(' + degVal + ',' + pwmSend + ',-' + pwmSend + ')\r\n' 
        commandSend = str.encode(command) #Encode command into bytes
        return commandSend
    
    else:
        return #Shouldn't ever go through here, actually. 
        
    
#Movement Functions
def LinearSPIKE(DesDist): #Move the SPIKE linearly
    calibVal = 1                                                                  #Calibration value for linear movement
    
    if DesDist == 0:
        return #No need to do anything if we don't wish to move linearly
    
    else:
        NumDegrees = Convert(DesDist, diam, calibVal)

        commandSend = DistCommand(DesDist, NumDegrees)

        ser.write(commandSend)

        #Serial read to confirm completion 
        time.sleep(2) #Pauses to avoid reading before motors have actually started moving
        busy = False
        while not busy:
            ser.write(b'hub.port.A.motor.get()[0]\r\n')
            reply = ser.readline()
            val = reply.decode() #Check this
            
            if (val[0] == '0'):
                busy = True

            time.sleep(0.01)


        
def AngCommand(NumDegrees): #Used to detect if the desired angle indicates to rotate counter/clockwise or if it's zero. Returns string to be executed
    degVal = str(NumDegrees) #Converting our calculated degrees into a string value
    if NumDegrees > 0: #Rotate clockwise
        pwmSend = str(-1*pwmVal) #Set PWM values to negative (With my build, BOTH pwm values need to be positive or negative)
        
        command = 'p.run_for_degrees(' + degVal + ',' + pwmSend + ',' + pwmSend + ')\r\n' #Concatenate all strings. ie "b'p.run_for_degrees(360,55,-55)\r\n'"
        commandSend = str.encode(command) #Encode command into bytes
        return commandSend
    
    elif NumDegrees < 0: #Rotate counterclockwise
        pwmSend = str(pwmVal) #Set PWM value to positive
        
        command = 'p.run_for_degrees(' + degVal + ',' + pwmSend + ',' + pwmSend + ')\r\n' 
        commandSend = str.encode(command) #Encode command into bytes
        return commandSend
    
    else:
        return #Shouldn't ever go through here, actually.
    

def RotateSPIKE(DesAng, NeccRot): #Rotate SPIKE to the desired angle. Calculates based on the necessary rotations determined from the "RotVari" function     
    calibVal = 1.05 #Calibration value for rotational movement
    
    if DesAng == 0:
        return #No need to do anything if we don't wish to rotate
    
    else:
        AngMove = DesAng/360 #Ratio between desired turning angle and 360 of a full circle
        NumRot = AngMove*NeccRot #Multiply necessary rotations and turn ratio to determine number of rotations for specific desired input
        NumDegrees = round(NumRot*360*calibVal) #Calculate degrees motors need to spin for to rotate the SPIKE
        
        commandSend = AngCommand(NumDegrees)
        
        #Writing to the SPIKE 
        ser.write(commandSend)

        #Serial read to confirm completion 
        time.sleep(2) #Pauses to avoid reading before motors have actually started moving
        busy = False
        while not busy:
            ser.write(b'hub.port.A.motor.get()[0]\r\n')
            reply = ser.readline()
            val = reply.decode() #Check this

            
            if (val[0] == '0'):
                busy = True

            time.sleep(0.01)




#--------Variable Setups--------#
global diam, pwmVal, WheelDist

diam = 0.055 #Diameter of the wheels in Meters
pwmVal = 45 #pwm to set default motor speed. -100 to 100
WheelDist = 0.143 #Distance between the centers of the wheels (axle distance)

DesAng = float(sys.argv[1]) #Rotating angle. Positive for clockwise, negative for counterclockwise
DesDist = float(sys.argv[2]) #Desired distance to travel. Units in meters

#--------Intial Functions--------#
startSerial() #Begin communication with the SPIKE
NeccRot = RotVari(diam, WheelDist) #Calculate rotation ratio for the specific robot build


#--------MAIN CODE--------#
RotateSPIKE(DesAng, NeccRot) #SPIKE will rotate first, then linearate
#time.sleep(4)
LinearSPIKE(DesDist)

print("\nHave finished moving")

endSerial() #Close serial port to SPIKE
