# -*- coding: utf-8 -*-
"""
This program acquires spectra using a ANDO 6315E and rotates a Thorlabs K10CR1 rotation stage.
By Grace Kerber and Dan Hickstein
This program requires that the Thorlabs Kinesis drivers are in the same folder as this program.
pyVISA is used to communicate with the ANDO OSA using GPIB.
"""
from __future__ import print_function
import ctypes as c
import numpy as np
import os, time
import sys
import pyvisa as visa
import datetime
import matplotlib.pyplot as plt
import platform


print('Hello!')
#Set OSA Sensitivity (with time for each in parenthesis)
# 0 = Norm Range Hold (7   sec) # Don't use this one! It's terrible!!!
# 1 = Norm Range Auto (10  sec)
# 2 = High 1          (1.2 min)
# 3 = High 2          (2.4 min)
# 4 = High 3          (~17 min)

val_sens = int(1)

# Determines what angles to collect data at
ExtinguishAngle = 47.5  # Angle at which minimum power occurs
MaxPower = 180  # Maximum Power through half wave plate
MinPower = 2.03
NumOfPoints = 1   # number of data points (different power) to be collected

#powerarray = np.linspace(135, 35, NumOfPoints)
powerarray = np.linspace(120, 20, NumOfPoints)
#powerarray = np.linspace(MaxPower-0.1, MinPower, NumOfPoints)
print(powerarray)

# powerarray = np.array([3, 5, 10, 20, 30])

if np.any(powerarray>MaxPower) or np.any(powerarray<MinPower):
    raise ValueError('Power outside of range requested.',
                     'Check the values of powerarray Zachary.')



# initialize the ANDO
sens = ["SNHD", "SNAT", "SHI1", "SHI2", "SHI3"]
rm = visa.ResourceManager()
print(rm.list_resources())
#osa = rm.get_instrument("GPIB0::23::INSTR")
osa = rm.open_resource("GPIB0::23::INSTR")
# osa = rm.open_resource("ASRL3::INSTR")


osa.write(sens[val_sens])

def angleFromPower(power, minPower=MinPower, maxPower=MaxPower, extinguishingAngle=ExtinguishAngle):
    return -(np.arcsin( ((power-minPower)/(maxPower-minPower))**0.5))*(180/np.pi)/2.  + extinguishingAngle

anglearray = angleFromPower(powerarray)
print(anglearray)

if   val_sens == int(0):
    scanTime = 7./60.
    raise ValueError('Oh dear! Did you really mean to set this to Norm Hold? (val_sens=0). That mode is terrible.')
elif val_sens == int(1):
    scanTime = 10./60.
elif val_sens == int(2):
    scanTime = 1.2
elif val_sens == int(3):
    scanTime = 2.4
elif val_sens == int(4):
    scanTime = 17
elif val_sens == int(0):
    scanTime = 7./60.
else:
    raise ValueError('Sensitivity Was Not Correctly Registered')

print('Setting up the Thorlabs K10CR1:')
bits, version = platform.architecture()
print('    Detected %s Python on %s. Loading %s DLLs' % (bits, version, bits))

dllname = os.path.join(os.path.dirname(__file__), 'dll%s' %
                       bits[:2], 'Thorlabs.MotionControl.IntegratedStepperMotors.dll')
os.environ['PATH'] = os.environ['PATH'] + ';' + \
    os.path.join(os.path.dirname(__file__), 'dll%s' % bits[:2])

if not os.path.exists(dllname):
    raise ValueError('DLL Not found! dllname=%s' % dllname)

if bits == '32bit':
    p = c.CDLL(dllname)  # Alternate between dll loading method
else:
    p = c.windll.LoadLibrary(dllname)


def getHardwareInfo(SN):

    modelNo = c.c_buffer(255)
    sizeOfModelNo = c.c_ulong(255)
    hardwareType = c.c_ushort()
    numChannels = c.c_short()
    notes = c.c_buffer(255)
    sizeOfNotes = c.c_ulong(255)
    firmwareVersion = c.c_ulong()
    hardwareVersion = c.c_ushort()
    modState = c.c_ushort()
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
                          c.pointer(modState))

    return [x.value for x in (modelNo, sizeOfModelNo, hardwareType,
                              numChannels, notes, sizeOfNotes, firmwareVersion,
                              hardwareVersion, modState)]


def getMotorParamsExt(SN):
    # p.ISC_ClearMessageQueue(SN);

    stepsPerRev = c.c_double()
    gearBoxRatio = c.c_double()
    pitch = c.c_double()

    p.ISC_GetMotorParamsExt(SN, c.pointer(stepsPerRev),
                                  c.pointer(gearBoxRatio),
                                  c.pointer(pitch))

    if stepsPerRev.value < 1 or gearBoxRatio.value < 1 or pitch.value < 1:
        print('    Failed to get motor params, using default values!')
        print('        stepsPerRev=200, gearBoxRatio=120, pitch=360')

        return 200, 120, 360
    else:
        return stepsPerRev.value, gearBoxRatio.value, pitch.value


def getDeviceList():
    p.TLI_BuildDeviceList()
    receiveBuffer = c.c_buffer(200)
    sizeOfBuffer = c.c_ulong(255)
    p.TLI_GetDeviceListExt(c.pointer(receiveBuffer), c.pointer(sizeOfBuffer))
    ser_num = [x.replace('b\'', '') for x in
            (str(receiveBuffer.value)).split(',')[:-1]]
    return ser_num


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

    while time.time() < (t+timeout):
        GetStatus(SN)
        p.ISC_RequestStatus(SN)  # order the stage to find out its location
        currentPosition = p.ISC_GetPosition(SN)
        error = currentPosition - deviceUnits
        if np.abs(error) < tolerance:
            return
        else:
            time.sleep(queryDelay)
    raise ValueError('Did not reach position!',
                     'Increase timeout from %.3f seconds?' % timeout)


def GetStatus(SN):
    p.ISC_RequestStatus(SN)
    # bits = p.ISC_GetStatusBits(SN)
    # print bin(bits)

try:
    serialNumber = getDeviceList()[0]
except:
    raise ValueError(
        'Couldn\'t get the list of serial numbers! Is your stage plugged in?',
        ' Or is Thorlabs Kinesis/APT open?')


SN = c.c_buffer(serialNumber.encode('UTF-8'))
print('    Stage found! Serial number %s'%serialNumber)

try:
    p.ISC_Close(SN)
    print('    Previous stage connection closed.')
except:
    pass

p.ISC_Open(SN)
print('    New stage connection opened.')


hardwareinfoval = getHardwareInfo(SN)
p.ISC_StartPolling(SN, c.c_int(20))
# p.ISC_LoadSettings(SN)

# Calculate the conversion between "Device units" and degrees
stepsPerRev, gearBoxRatio, pitch = getMotorParamsExt(SN)
# from https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=8750
microstepsPerFullstep = 2048
conversion = stepsPerRev * microstepsPerFullstep * \
    gearBoxRatio / pitch  # convert to degrees
# conversion is in "Device units per degree"

# Begin moving stage to Home- defines where zero is
print('\nHoming...', end='')
sys.stdout.flush()
p.ISC_Home(SN)
time.sleep(0.2)

# While Motor is moving.  Stopped is -2147482624
while (p.ISC_GetStatusBits(SN)) != (-2147482624):
    # print(p.ISC_GetStatusBits(SN))
    # print 'in the process of homing'
    print('.', end='')
    sys.stdout.flush()
    time.sleep(0.5)

print('   Done!\n')


#---Create Base Directory for saving data
today = datetime.datetime.now().strftime("%Y-%m-%d")
cwd = os.getcwd()
base_dir = os.path.join(cwd, 'ando-' + today)
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

print('Saving to:   %s\n' %(new_base_dir))


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


### This is where the MAGIC happens. ###
degree = anglearray
for count, (degree) in enumerate(degree[::]):
    DegreePosition = degree #value in degrees

    # convert the desired position to integer "Device units" to be passed to the stage
    # NOTE: this involves rounding, and could introduce errors, especially if you are making
    # steps of just a few device units.
    try:
        deviceUnits = abs(int(DegreePosition*conversion))  # -deviceUnitsZero)
    except ValueError:
        raise ValueError(('Could not get the position from the stage.',
                          ' This typically means that you need to unplug the',
                          ' stage and plug it back in. And restart your',
                          ' python terminal.'))

    print('Run %04i % 3i of % 3i - %5.3f degrees - %6.2f mW - '%(run_counter, count+1, powerarray.size,DegreePosition, powerarray[count]), end='')
    sys.stdout.flush()

    MoveToPosition(SN, deviceUnits)
    new_position = p.ISC_GetPosition(SN)
    new_degrees = new_position/conversion
    # print 'Reported %5.3f degrees (%i Device Units).'%(new_degrees, new_position); sys.stdout.flush()

    #Tells OSA to begin sweep
    osa.write("SGL")

    query = int(osa.query('SWEEP?')) # greater that zero means OSA is currently performing a sweep

    #Checking if OSA Done Sweeping
    while query > 0:
        time.sleep(.2) # in seconds
        query = int(osa.query('SWEEP?'))


    ### Capturing Data Trace from OSA

    # Active Trace
    t_active = int(osa.query("ACTV?"))
    trace = "ABC"[t_active]

    # # Instrument ID
    # osa_ID = ''.join([i if ord(i) < 128 else ' ' for i in osa.read_raw().rstrip()]) # strips non-ASCII characters
    osa_ID = str(osa.read_raw().rstrip()) # strips non-ASCII characters

    # Time Stamp
    time_now = datetime.datetime.now().strftime("%Y-%m-%d %X")

    # Measurement Characteristics
    t_list_hds = "Center Wvl:,Span Range:,REF Level:,Level Scale:,Wvl Resolution:,Avg Count:,Sampl Count:,Sensitivity:,Monochro:,Waveform Type:".split(',')
    t_list_cmds = ['CTRWL', 'SPAN', 'REFL', 'LSCL', 'RESLN', 'AVG', 'SEGP', 'SENS', 'MONO', 'TR' + trace]
    t_list = []
    
    d_mono = {0: 'SGL', 1: 'DBL'}
    d_sens = {1: 'HIGH 1', 2: 'HIGH 2', 3: 'HIGH 3', 4: 'NORMAL RANGE HOLD', 5: 'NORMAL RANGE AUTO'}
    d_trace = {0: {0: 'MEAS', 1: 'FIX', 2: 'MAX HOLD', 3: 'ROL AVG'},
               1: {0: 'MEAS', 1: 'FIX', 2: 'MIN HOLD', 3: 'ROL AVG'},
               2: {0: 'MEAS', 1: 'FIX', 2: 'A-B', 3: 'B-A', 4: 'A-B (LIN)', 5: 'B-A (LIN)',6: 'A+B (LIN)', 
                   7: 'NORMALIZE', 8: 'DOMINANT', 10: 'CURVE FIT', 110: 'CURVE FIT PK'}}
    d_commands = {'MONO': d_mono, 'SENS': d_sens, 'TRA': d_trace[0], 'TRB': d_trace[1], 'TRC': d_trace[2]}
    
    for cmd in t_list_cmds:
        cmd_response = osa.query(cmd+'?').rstrip()
        try:
            t_dic = d_commands[cmd]
            try:
                t_list.append(t_dic[int(cmd_response)])
            except KeyError:
                t_list.append('N/A')
        except KeyError:
            t_list.append(cmd_response)
     
        #TODO: probably would get a key exception, so would for t_trace, would need to figure that one out.
            
    # Spectral Data
    osa.write("LDTDIG3") #sets retrieval the maximum of 3 decimal places
    level_unit = ["W","dBm"][bool(float(osa.query("LSCL?")))]
    abs_or_dens = ["","/nm"][int(osa.query("LSUNT?"))]
    t_wave = osa.query("WDAT"+trace).rstrip().split(',')[1:] #discards the sample count
    t_level = osa.query("LDAT"+trace).rstrip().split(',')[1:]
    # Format Data String:
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
        print('Saved')

    # plt.plot(t_wave,t_level)


print('Moving Back to Max Power')
MoveToPosition(SN, abs(int((ExtinguishAngle+45)*conversion)))
print('Power scan complete!')
p.ISC_Close(SN)
