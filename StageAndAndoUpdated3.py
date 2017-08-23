# -*- coding: utf-8 -*-
"""
Created on Mon Jun 19 15:14:22 2017

This program acquires spectra using a ANDO 6315E and rotates a Thorlabs K10CR1 rotation stage.

By Grace Kerber and Dan Hickstein

This program requires that the Thorlabs Kinesis drivers are in the same folder as this program.

pyVISA is used to communicate with the ANDO OSA using GPIB.
"""

import ctypes as c
import numpy as np
import os, time
import sys
import visa
import datetime
import matplotlib.pyplot as plt

#Set OSA Sensitivity (with time for each in parenthesis)
# 0 = Norm Range Hold (7 sec for full spectrum) # Don't use this one! It's terrible!!!
# 1 = Norm Range Auto : 1 (10 sec)
# 2 = High 1 : 2 (1.2 min)
# 3 = High 2 : 3 ( 2.4 min)
# 4 = High 3 : 4 (17ish min~estimated I didn't want to wait that long)
 
val_sens = int(1)

#Determines what angles to collect data at
ExtinguishAngle = 58.7  # Angle at which minimum power occurs
MaxPower        = 135.9 # Maximum Power through half wave plate
MinPower        = 2.5   # Minimum Power through half wave plate (power measured at ExtinguishAngle)
NumOfPoints     = 150   # number of data points (different power) to be collected 


# initialize the ANDO
sens = ["SNHD", "SNAT", "SHI1", "SHI2", "SHI3"]
if val_sens == 0:
    raise ValueError('Oh god! Did you really mean to set this to Norm Hold? (val_sens=0). That mode is terrible.')
rm = visa.ResourceManager()
osa = rm.get_instrument("GPIB0::20::INSTR")
osa.write(sens[val_sens])

def angleFromPower(power, minPower=MinPower, maxPower=MaxPower, extinguishingAngle=ExtinguishAngle):
    return -(np.arcsin( ((power-minPower)/(maxPower-minPower))**0.5))*(180/np.pi)/2.  + extinguishingAngle

powerarray = np.linspace(MaxPower, MinPower, NumOfPoints)
anglearray = angleFromPower(powerarray)

if val_sens == int(0):
    TimeToFinish = (7./60.)*NumOfPoints
    print 'The Estimated Time to finish is {0} minutes, or {1} hours'.format(TimeToFinish, (TimeToFinish/60.))
elif val_sens == int(1):    
    TimeToFinish = (10./60.)*NumOfPoints
    print 'The Estimated Time to finish is {0} minutes, or {1} hours'.format(TimeToFinish, (TimeToFinish/60.))
elif val_sens == int(2):
    TimeToFinish = (1.20)*NumOfPoints
    print 'The Estimated Time to finish is {0} minutes, or {1} hours'.format(TimeToFinish, (TimeToFinish/60.))
elif val_sens == int(3):
    TimeToFinish = (2.4)*NumOfPoints
    print 'The Estimated Time to finish is {0} minutes, or {1} hours'.format(TimeToFinish, (TimeToFinish/60.))
elif val_sens == int(4):
    TimeToFinish = (17.)*NumOfPoints
    print 'The Estimated Time to finish is {0} minutes, or {1} hours'.format(TimeToFinish, (TimeToFinish/60.))
else:
    print 'Sensitivity Was Not Correctly Registered'
    

dllname = os.path.join(os.path.dirname(__file__), 'Thorlabs.MotionControl.IntegratedStepperMotors.dll')

if not os.path.exists(dllname):
    print "ERROR: DLL not found"


# Initializing the Thorlabs rotation stage
#p = c.windll.LoadLibrary(dllname)  #Alternate between dll loading method
p = c.CDLL(dllname) #Alternate between dll loading method

def getHardwareInfo(SN):
    
    modelNo =         c.c_buffer(255)
    sizeOfModelNo =   c.c_ulong(255)
    hardwareType =    c.c_ushort()
    numChannels =     c.c_short()
    notes =           c.c_buffer(255)
    sizeOfNotes =     c.c_ulong(255)
    firmwareVersion = c.c_ulong()
    hardwareVersion = c.c_ushort()
    modState        = c.c_ushort()
    # p.PCC_GetHardwareInfo(SN)

    p.ISC_GetHardwareInfo(SN, 
                          c.pointer(modelNo), 
                          c.pointer(sizeOfModelNo), 
                          c.pointer(hardwareType),
                          c.pointer(numChannels),
                          c.pointer(notes),
                          c.pointer(sizeOfNotes),
                          c.pointer(firmwareVersion),
                          c.pointer(hardwareVersion),
                          c.pointer(modState) )
    

    return [x.value for x in (modelNo, sizeOfModelNo, hardwareType, numChannels, notes, sizeOfNotes, firmwareVersion, hardwareVersion, modState)]


def getMotorParamsExt(SN):
	# this doesn't work for some reason...
	stepsPerRev =  c.c_double()
	gearBoxRatio = c.c_double()
	pitch =        c.c_double()

	p.ISC_GetMotorParamsExt(SN, c.pointer(stepsPerRev), 
							       c.pointer(gearBoxRatio), 
							       c.pointer(pitch))

	return stepsPerRev.value, gearBoxRatio.value, pitch.value


def getDeviceList():
    p.TLI_BuildDeviceList()
    receiveBuffer = c.c_buffer(200)
    sizeOfBuffer  = c.c_ulong(255)
    p.TLI_GetDeviceListExt(c.pointer(receiveBuffer), c.pointer(sizeOfBuffer))
    return [x for x in (receiveBuffer.value).split(',')[:-1]]


def MoveToPosition(SN, deviceUnits, timeout=20, queryDelay=0.01, tolerance=1):
    """
    Moves the rotation stage to a certain position (given by device units).
    This call blocks future action until the move is complete.
    The timeout is in seconds
    
    SN is a c_buffer of the serial number string
    deviceUnits shold be a int.
    tolerance is when the blocking should end (device units)
    """
    
    GetStatus(SN)
    p.ISC_MoveToPosition(SN, c.c_int(int(deviceUnits)))

    t = time.time()

    while time.time()<(t+timeout):
        GetStatus(SN)
        p.ISC_RequestStatus(SN) # order the stage to find out its location
        currentPosition = p.ISC_GetPosition(SN)
        error = currentPosition - deviceUnits
        if np.abs(error)<tolerance: 
            return
        else:
            time.sleep(queryDelay)
    raise ValueError('Oh no!!! We never got there!! Maybe you should make the timeout longer than %.3f seconds dude.'%timeout)

    

try:
    serialNumber = getDeviceList()[0]
except:
    raise ValueError('Couldn\'t get the list of serial numbers! Is your stage plugged in? Or is Thorlabs Kinesis open?')

def GetStatus(SN):
    p.ISC_RequestStatus(SN)
    #bits = p.ISC_GetStatusBits(SN)
    #print bin(bits)
    
    
#---Create Base Directory for saving data
today = datetime.datetime.now().strftime("%Y-%m-%d")
cwd = os.getcwd()
base_dir = os.path.join(cwd, today)
if not(os.path.isdir(base_dir)):
    os.mkdir(base_dir)

run_counter = 1 
run_folder  = 'run %04i'%(run_counter)

# find the first available file name:
while os.path.isdir(os.path.join(base_dir, run_folder)):
    run_counter = run_counter+1
    run_folder = 'run %04i'%(run_counter)
new_base_dir = os.path.join(base_dir,run_folder)
os.mkdir(new_base_dir)    

print '\n\nWill save Data Log to run folder %i\n\n' %(run_counter);

SN = c.c_buffer(serialNumber)

try:
    p.ISC_Close(SN)
    print 'Previous stage connection closed.'
except:
    pass

p.ISC_Open(SN)

#hardwareinfoval = getHardwareInfo(SN)
p.ISC_StartPolling(SN,c.c_int(20))
p.ISC_LoadSettings(SN)

with open(os.path.join(new_base_dir, 'LOGFILE.txt'), 'w') as logfile:
    time_now  = datetime.datetime.now().strftime("%Y-%m-%d %X")
    logfile.write('Instrument: ANDO\n')
    logfile.write('Time\t'+time_now+'\n')
    logfile.write('Min_pow angle: %.4f\n'%ExtinguishAngle)
    logfile.write('Max Power: %.4f\n'%MaxPower)
    logfile.write('Min Power: %.4f\n'%MinPower)
    logfile.write('Num Points: %i\n'%NumOfPoints)
    logfile.write('FileNum\tPower\t Angle (deg)\n')
    
    for count, (power, angle) in enumerate(zip(powerarray, anglearray)):
        logfile.write('%04i\t%.6f\t%.6f\n'%(count+1, power,angle))


#Begin moving stage to Home- defines where zero is
print 'Homing...'; sys.stdout.flush()
p.ISC_Home(SN)
time.sleep(0.5)

while (p.ISC_GetStatusBits(SN))!=(-2147482624):  #While Motor is moving.  Stopped is -2147482624
    #print p.ISC_GetStatusBits(SN)
    #print 'in the process of homing'
    time.sleep(1)
    
print 'Rotation Stage Has Been Homed'


#initializes conversion between 
stepsPerRev, gearBoxRatio, pitch = getMotorParamsExt(SN)
#print stepsPerRev, gearBoxRatio, pitch
microstepsPerFullstep = 2048 # from https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=8750
conversion = stepsPerRev * microstepsPerFullstep * gearBoxRatio / pitch # convert to degrees
#print conversion


### This is where the MAGIC happens. ###
degree = anglearray
for count, (degree) in enumerate(degree[::]):
    DegreePosition = degree #value in degrees

    deviceUnits = abs(int(DegreePosition*conversion))#-deviceUnitsZero) #this involved rounding!
    print 'Moving to %5.3f degrees (%i Device Units)...'%(DegreePosition, deviceUnits); sys.stdout.flush()
    MoveToPosition(SN, deviceUnits)
    new_position = p.ISC_GetPosition(SN)
    new_degrees = new_position/conversion
    # print 'Reported %5.3f degrees (%i Device Units).'%(new_degrees, new_position); sys.stdout.flush()
    
    #Tells OSA to begin sweep
    osa.write("SGL")
    
    query = int(osa.query('SWEEP?'))  #greater that zero means OSA is sweeping

    #Checking if OSA Done Sweeping
    while query > 0:
        time.sleep(.2) #in seconds  
        query = int(osa.query('SWEEP?'))
        
            
    #Capturing Data Trace from OSA
    #---Active Trace
    t_active = int(osa.query("ACTV?"))
    trace = "ABC"[t_active]
    #---Instrument ID
    osa_ID = ''.join([i if ord(i) < 128 else ' ' for i in osa.read_raw().rstrip()]) #strips non-ASCII characters
    
    #---Time Stamp
    time_now = datetime.datetime.now().strftime("%Y-%m-%d %X")
    #---Measurement Characteristics
    t_list_hds = "Center Wvl:,Span Range:,REF Level:,Level Scale:,Wvl Resolution:,Avg Count:,Sampl Count:,Sensitivity:,Monochro:,Waveform Type:".split(',')
    t_list = osa.query("ST"+trace+"?").rstrip().split(',')
    #---Spectral Data
    osa.write("LDTDIG3") #sets retrieval the maximum of 3 decimal places
    level_unit = ["W","dBm"][bool(float(osa.query("LSCL?")))]
    abs_or_dens = ["","/nm"][int(osa.query("LSUNT?"))]
    t_wave = osa.query("WDAT"+trace).rstrip().split(',')[1:] #discards the sample count
    t_level = osa.query("LDAT"+trace).rstrip().split(',')[1:]
    #---Format Data String
    col_1 = ["Instrument:"] + ["Time Stamp:"] + t_list_hds + ["", "Wavelength(nm)"] + t_wave
    col_2 = [osa_ID] + [time_now] + t_list + ["", "Level("+level_unit+abs_or_dens+")"] + t_level
    col_comb = zip(col_1, col_2)
    data_list = []
    for data_row in col_comb:
        data_list.append('\t'.join(data_row))
    data_string = "\n".join(data_list)
    
    #Saving Data Trace   
    
    with open(os.path.join(new_base_dir,'ando-osa-data_'+today+'_%04i.txt'%(count+1)), 'w') as data_file:
        data_file.write(data_string)
        
    print 'Saved Trace to {} out of {}'.format(count+1, NumOfPoints); sys.stdout.flush()
        
    plt.plot(t_wave,t_level)


#print 'Moving Back to Position Zero'
#MoveToPosition(SN,0)
p.ISC_Close(SN)