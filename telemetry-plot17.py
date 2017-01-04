# converted from ipython notebook to make possible to debug
# coding: utf-8

# In[20]:

#get_ipython().magic(u'pylab')
# Imports
import numpy as np
import matplotlib.pyplot as plot
plot.close("all")   # try to close all open figs
#import sys
#print sys.path
#comment next line to have plot windows outside browser
#%matplotlib inline

legScale = 95.8738e-6 # 16 bit to radian
vref = 3.3 # for voltage conversion
vdivide = 3.7/2.7  # for battery scaling
vgain = 15.0/47.0  # gain of differential amplifier
RMotor = 3.3   # resistance for SS7-3.3 ** need to check **
Kt = 1.41 #  motor toriqe constant mN-m/A  ** SS7-3.3 **

#acelerometer scale in mpu6000.c set to +- 8g
# +- 32768 data
xlScale = (1/4096.0) * 9.81

# gyro in mpu6000.c scale set to +-2000 degrees per second
# +- 32768
gyroScale = (1/16.384) * (np.pi/180.0)  

height = 8
width = 9


# In[21]:

# Insert filename here
#filename = './2014.02.03_17.47.54_trial_imudata.txt'
#filename = './2014.02.03_17.53.20_trial_imudata.txt'
#filename = './2014.02.03_17.54.06_trial_imudata.txt'
#filename = '.\power-test.txt'
#filename = '../turner-ip2.5/Python/Data/imudata.txt'
#filename = 'c:/users/ronf/research/development/turner-ip2.5/Python/Data/imudata.txt'
#filename = 'c:/users/ronf/research/development/telemetryData/imudata16Feb15.txt'
#filename = 'c:/users/ronf/research/development/telemetryData/left1000right1000.txt'
filename = '../telemetryData/tripod300l290r-air.txt'
#filename = '../telemetryData/tripod300l290r-gnd-1sec.txt'
#filename = '../telemetryData/Jul15-AccelSteer/200ms35351515l1.txt'
#filename = '../telemetryData/Jul15-RollSteer/200ms130degl1.txt'
#filename = '../turner-ip2.5/Python/Data/2014.12.02_16.31.40_trialAir_JL_imudata.txt'
#filename= '../turner-ip2.5/Python/Data/2014.12.03_12.45.27_trialAir_1Hz_JL_imudata.txt'  
print filename


# In[22]:

# Read in the file
f = open(filename)
lines = f.readlines()
f.close()

# Time and Date stamps of trial
dateStamp = lines[0].split()[3]
timeStamp = lines[0].split()[4][:-1]
print 'date, time:', dateStamp, timeStamp

# Stride Frequency R,L
rightStrideFreq = float(lines[1].split()[5][:-1])
leftStrideFreq = float(lines[2].split()[5][:-1])

# Phase (fractional)
phaseFractional = float(lines[3].split()[4][:-1])

# Script that generated data
script = lines[4].split()[1]
# Motor Gains
motorGains = map(float, lines[5][lines[5].find('[')+1:lines[5].find(']')].replace(',','').split())
print 'motorGains =', motorGains
# test data read
#print 'lines[8:15]', lines[8:15]

# Data splitting
columnHeaders = lines[7].replace("\"% ","").replace(" \"\r\n","").split(' | ')
columns = len(columnHeaders)
rows = len(lines)-8
print 'rows,columns=', rows, columns
print lines[8:12]
#unix: 
#print map(float,"".join(lines[8:10]).replace('\r\n',',')[:-1].split(','))
# windows
print map(float,"".join(lines[8:10]).replace('\n',',')[:-1].split(','))

# unix: 
#data = np.reshape(map(float,"".join(lines[8:]).replace('\r\n',',')[:-1].split(',')),(rows,columns))
# windows
data = np.reshape(map(float,"".join(lines[8:]).replace('\n',',')[:-1].split(',')),(rows,columns))


# In[24]:


###########################
#"% time | LPos | RPos | LRef | RRef | dcL | dcR | GyX | GyY | GyZ | AX | AY | AZ | LEMF | REMF | BAT "
#     0      1     2      3       4     5     6     7     8     9    10    11   12   13    14      15

#print 'first 10 lines of data[]\n', data[0:10]
# Timestamps
time = data[:,0]/1000.0

# Leg Position R,L
#left 
#rightLegPos = (data[:,1]*legScale)% (2.0*np.pi)
#leftLegPos = (data[:,2]*legScale) % (2.0*np.pi)
leftLegPos = (data[:,1]*legScale)
rightLegPos = (data[:,2]*legScale) 

# Commanded Leg Position R,L
#commandedRightLegPos = (data[:,3]*legScale) % (2.0*np.pi)
#commandedLeftLegPos = (data[:,4]*legScale) % (2.0*np.pi)
commandedLeftLegPos = (data[:,3]*legScale) 
commandedRightLegPos = (data[:,4]*legScale)

# Duty Cycle R,L
#DCR = -data[:,5]/4000.0
#DCL = -data[:,6]/4000.0
#DCR[DCR < -4000.0] = -4000.0
#DCL[DCL < -4000.0] = -4000.0
# raw 12 bit PWM value. Max = 3900.
# convert to per cent
DCR = 100.0 * data[:,6]/4096.0
DCL = 100.0 * data[:,5]/4096.0

# Gyro X,Y,Z
GyroX = data[:,7]*gyroScale
GyroY = data[:,8]*gyroScale
GyroZ = data[:,9]*gyroScale

# Accelerometer X,Y,Z
AX = data[:,10]*xlScale
AY = data[:,11]*xlScale
AZ = data[:,12]*xlScale

# BackEMF R,L
# A/D data is 10 bits, Vref+ = AVdd = 3.3V, VRef- = AVss = 0.0V
# BEMF volts = (15K)/(47K) * Vm + vref/2 - pidObjs[i].inputOffset
#RBEMF = -data[:,13]*vdivide*vref/1023.0
#LBEMF = -data[:,14]*vdivide*vref/1023.0
RBEMF = data[:,14]*vref/1024.0/vgain  # scale A/D to 0 to 3.3 V range and undo diff amp gain
LBEMF = data[:,13]*vref/1024/vgain

# Battery Voltage in volts
VBatt = data[:,15]*vdivide*vref/1023.0

#Power calculations
# i_m = (VBatt - BEMF)/R
# V_m is just VBatt
# using motor duty cycle as 0-100%
CurrentR = (np.abs(DCR)/100.0)*(np.sign(DCR)*VBatt - RBEMF)/RMotor # i_m_avg = i_m x duty cycle
CurrentL = (np.abs(DCL)/100.0)*(np.sign(DCL)*VBatt - LBEMF)/RMotor # i_m_avg = i_m x duty cycle

# torque calculation
TorqueR = Kt * CurrentR # \Tau = Kt i_m_avg
TorqueL = Kt * CurrentL # \Tau = Kt i_m_avg
    

PowerR = np.abs(VBatt* CurrentR) # P = V_m i_m_avg
PowerL = np.abs(VBatt* CurrentL) # P = V_m i_m_avg

Energy = np.zeros(len(VBatt))
#energy calculation, account that some time samples may be missing
# dt = (time[1] - time[0]) / 1000.0 # time in seconds
for i in range(1,len(VBatt)):
    Energy[i] = Energy[i-1] + (PowerR[i] + PowerL[i]) * (time[i]-time[i-1])/1000.0

#approximate integral of angle
AngleZ = np.zeros(len(GyroZ))
AngleY = np.zeros(len(GyroY))
for i in range(1,len(GyroZ)):
    AngleZ[i] = AngleZ[i-1] + GyroZ[i]/1000.0
    AngleY[i] = AngleY[i-1] + GyroY[i]/1000.0


# In[25]:

# calculate turn we get as function of phase
#from turnCalc import turnCalc
#turnCalc(leftLegPos, rightLegPos, AngleZ)
#fig = plot.figure(figsize = (length, width/3))
#leg angle vs gyro angle
#plot.subplot(1,2,1)
#plot.xlabel('right leg angle [rad]')
# Total Angle in radians
#plot.plot(-rightLegPos,AngleZ, 'g') # gyro angle
#plot.plot(-rightLegPos,leftLegPos)
#plot.plot(rightLegPos, rightLegPos+leftLegPos,'b') # difference in leg angles
#plot.plot(-commandedRightLegPos,commandedRightLegPos+commandedLeftLegPos,'g--') # difference in ref leg angles
#plot.ylabel('Angle diff (rad)')
#plot.legend(['Gz','leg phase','cmd phase'])
#ax = fig.add_subplot(1,2,1)
#ax.axhline(linewidth=1, color='m')
#xticks=np.linspace(0,11,12)*np.pi
#ax.set_xticks(np.round(xticks,1))
#ax.xaxis.grid() #vertical lines

maxAngle = np.max(rightLegPos) - np.min(rightLegPos) # get max change in angle
# print 'rightLegPos', rightLegPos
print 'maxAngle=', maxAngle
maxCycle = np.int(maxAngle/np.pi)  # half steps
print 'maxCycle=', maxCycle
startCycle=np.linspace(0,maxCycle,maxCycle+1)*np.pi  # start of next half step
phases = np.zeros(maxCycle+2)
phases[0] = rightLegPos[0]+leftLegPos[0]  # assumed initial phase difference
yawAngle = np.zeros(maxCycle+2)
yawAngle[0]= AngleZ[0]
yawChange = np.zeros(maxCycle+2)
deltaYaw = np.zeros(len(rightLegPos)) # record change in yaw angle at each step
yawStep = np.zeros(len(rightLegPos))
xticks = np.zeros(maxCycle+4)  # hold tick locations
xticks = np.arange(0,maxCycle+4)  # non-zero place holder for xticks

#############
# pick longer leg cycle, and reset yaw angle at start of this step
# should trigger near 180 degrees phase on one leg
minThresh =  0.1  ## left angle is decreasing and cycling to 2 pi
maxThresh = 1
minFound = False
yawHold = 0.0
j = 0
for i in range(150,len(rightLegPos)):
    pos = rightLegPos[i] % (2.0*np.pi)   # get right leg position mod 2 pi
    if not minFound and (pos > (np.pi-maxThresh)) and (pos < (np.pi-minThresh)):
        minFound = True
    if minFound and (pos > (np.pi+minThresh)):
        xticks[j] = i
        j = j+1
        yawHold = AngleZ[i]  # save yaw value at end of leg cycle
        print 'i=%d yawHold =%6.3f' %(i,yawHold)
        minFound = False
        print 'i=%d, rightLegPos %6.3f' %(i, (rightLegPos[i] % (2.0*np.pi)))
    yawStep[i] = yawHold 
    deltaYaw[i] = AngleZ[i] -yawHold  # shift value for next step
 #   print 'i,leftLegPos',i,(leftLegPos[i] % (2.0*np.pi))
print 'yaw max =%6.3f' %(AngleZ[i])
        
#plot.subplot(1,2,2)
#plot.xlabel('$\Delta \Theta$ [rad]')
# Total Angle in radians
#plot.plot(time,AngleZ, 'g') # gyro angle vs phase difference
#plot.plot(phases,yawAngle,'o')
#plot.plot(time,deltaYaw,'-k')
#plot.ylabel('Angle Est.(rad)')
#plot.legend(['Gz','$\Delta \Theta_z$'])
#plot.plot(time,yawStep,'b-')
#ax = fig.add_subplot(1,2,2)
#ax.axhline(linewidth=1, color='m')
#xticks=np.linspace(0,12,13)*np.pi/18  # hard coded 10 degree advance per step
#ax.set_xticks(np.round(xticks,1))
#ax.xaxis.grid() #vertical lines


# In[26]:

##############################################
### position, gyro, and torque data plotting
########################################
height = 8
width = 9
fig = plot.figure(figsize = (width, height))
min= 150
max = 2500
max=np.min([max, len(rightLegPos)])  # make sure max < length

# gyro data
plot.subplot(3,1,2)
#plot.plot(time[min:max],GyroX[min:max],'k--')
plot.plot(time[min:max],GyroY[min:max], 'g.')
plot.plot(time[min:max],GyroZ[min:max], 'b')
plot.xlabel('time [ms]')
plot.ylabel('Gyro rad/s')
plot.legend([ 'Y', 'Z'])
ax = fig.add_subplot(3,1,2)
ax.axhline(linewidth=1, color='m')
ax.set_xticks(np.round(xticks,1))
ax.xaxis.grid() #vertical lines

# actual and commanded leg position
plot.subplot(3,1,1)
plot.plot(time[min:max],rightLegPos[min:max]% (2.0*np.pi),'k')
plot.plot(time[min:max],leftLegPos[min:max]% (2.0*np.pi),'b')
plot.plot(time[min:max],commandedRightLegPos[min:max]% (2.0*np.pi), 'k--')
plot.plot(time[min:max],commandedLeftLegPos[min:max]% (2.0*np.pi), 'b-.')
plot.xlabel('time [ms]')
plot.ylabel('Leg Position')
plot.legend(['RPos','LPos','Rref','Lref'])
ax = fig.add_subplot(3,1,1)
ax.axhline(linewidth=1, color='m')
ax.set_xticks(np.round(xticks,1))
ax.xaxis.grid() #vertical lines

#Torque#
plot.subplot(3,1,3)
plot.plot(time[min:max],TorqueR[min:max],'k')
plot.plot(time[min:max],TorqueL[min:max],'b--')
#plot.plot(time[min:max],TorqueR[min:max] + TorqueL[min:max], 'g.')
plot.xlabel('time [ms]')
plot.ylabel('Torque (mN-m)')
plot.legend(['Right', 'Left'])
ax = fig.add_subplot(3,1,3)
ax.axhline(linewidth=1, color='m')
ax.set_xticks(np.round(xticks,1))
ax.xaxis.grid() #vertical lines




# In[18]:

##################################
### not so useful data for plotting for understanding turning
#######################


# accelerometer data
#plot.subplot(3,2,3)
#plot.plot(time[min:max],AX[min:max],'k--')
#plot.plot(time[min:max],AY[min:max],'g.')
#plot.plot(time[min:max],AZ[min:max],'b')
#xlabel('time [ms]')
#ylabel('Accel $ m s^{-2}$')
#legend(['X', 'Y', 'Z'])







# In[19]:

# plot position and turning angle
height = 8
width = 9
fig = plot.figure(figsize = (width,height))

print 'np.size', np.size(rightLegPos)
#min=100 
# max=np.size(rightLegPos)
#max=5000

phase = commandedRightLegPos+commandedLeftLegPos
phase_offset=np.average(phase[min:max])



# actual and commanded leg position
plot.subplot(3,1,1)
plot.xlabel('time [ms]')
# actual and commanded leg position
plot.plot(time[min:max],rightLegPos[min:max]% (2.0*np.pi),'g--')
plot.plot(time[min:max],2.0*np.pi-leftLegPos[min:max]% (2.0*np.pi),'b')
#plot.plot(time[min:max],phase[min:max],'r')
#plot.plot(time[min:max],commandedRightLegPos[min:max]% (2.0*np.pi), 'k--')
#plot.plot(time[min:max],commandedLeftLegPos[min:max]% (2.0*np.pi), 'b-.')
plot.ylabel('Leg Position')
plot.legend(['RPos','LPos'],bbox_to_anchor=(0.85, 1), loc=2, borderaxespad=0.)
yticks=np.linspace(0,2,3)*np.pi
ax = fig.add_subplot(3,1,1)
ax.set_yticks(np.round(yticks,2))
ax.yaxis.grid() #horiz lines
ax.set_xticks(np.round(xticks,1))
ax.xaxis.grid() #vertical lines
plot.title(filename+' Phase offset ='+format(180.0*phase_offset/np.pi,'.2f'))
#ax.title('Phase offset')

#plot.plot(time,commandedRightLegPos- rightLegPos,'k')
#plot.plot(time,commandedLeftLegPos - leftLegPos,'b')
#plot.plot(time,commandedRightLegPos, 'k--')
#plot.plot(time,commandedLeftLegPos, 'b-.')

#ylabel('Leg Position error')
#legend(['RErr', 'LErr'])
#battery voltage
#plot.subplot(3,2,4)
#plot.plot(time,VBatt)
#xlabel('time [ms]')
#ylabel('Battery Voltage (V)')


plot.subplot(3,1,2)
plot.xlabel('time [ms]')
# Total Angle in radians
#plot.plot(time,AngleY,'k')
plot.plot(time[min:max],AngleZ[min:max], 'g')
plot.ylabel('Angle (rad)')
plot.legend(['Gz'],bbox_to_anchor=(0.85, 1), loc=2, borderaxespad=0.)
ax = fig.add_subplot(3,1,2)
ax.axhline(linewidth=1, color='m')
ax.set_xticks(np.round(xticks,1))
ax.xaxis.grid() #vertical lines

#leg angle vs gyro angle
#plot.subplot(3,2,6)
#xlabel('right leg angle [rad]')
# Total Angle in radians
#plot.plot(-rightLegPos,AngleY,'k')
#plot.plot(-rightLegPos,AngleZ, 'g')
#ylabel('Angle Est.(rad)')
#legend(['Gy', 'Gz'])
#ax = fig.add_subplot(3,2,6)
#ax.axhline(linewidth=1, color='m')
#xticks=np.linspace(0,11,12)*np.pi
#ax.set_xticks(np.round(xticks,1))
#ax.xaxis.grid() #vertical lines

plot.subplot(3,1,3)
plot.xlabel('time (sec)')
# Total Angle in radians
plot.plot(time[min:max],deltaYaw[min:max], 'g') # gyro angle vs phase difference
#plot.plot(phases,yawAngle,'o')
#plot.plot(time[min:max], AngleZ[min:max],'-k')
#plot.plot(time[min:max], yawStep[min:max],'-k')
plot.ylabel('Angle Est.(rad)')
#plot.legend(['Gz','$\Delta \Theta_z$'])
plot.legend(['$\Delta Gz$'], bbox_to_anchor=(0.85, 1), loc=2, borderaxespad=0.)
ax = fig.add_subplot(3,1,3)
ax.axhline(linewidth=1, color='m')
#xticks=np.linspace(0,12,13)*np.pi/18 # for 10 degrees phase diff per step
ax.set_xticks(np.round(xticks,1))
ax.xaxis.grid() #vertical lines



############################################
# low level plot information on one page
# figure 3
#############################################
height = 8
width = 14
fig = plot.figure(figsize = (width,height))


# leg position error
plot.subplot(3,2,1)
plot.xlabel('time [ms]')
plot.plot(time,commandedRightLegPos- rightLegPos,'k')
plot.plot(time,commandedLeftLegPos - leftLegPos,'b')
plot.ylabel('Leg Position error')
plot.legend(['RErr', 'LErr'])
ax = fig.add_subplot(3,2,1)
ax.axhline(linewidth=1, color='m')

# actual and commanded leg position
plot.subplot(3,2,2)
plot.plot(time[min:max],rightLegPos[min:max]% (2.0*np.pi),'k')
plot.plot(time[min:max],leftLegPos[min:max]% (2.0*np.pi),'b')
plot.plot(time[min:max],commandedRightLegPos[min:max]% (2.0*np.pi), 'k--')
plot.plot(time[min:max],commandedLeftLegPos[min:max]% (2.0*np.pi), 'b-.')
plot.xlabel('time [ms]')
plot.ylabel('Leg Position')
plot.legend(['RPos','LPos','Rref','Lref'])
ax = fig.add_subplot(3,2,2)
ax.axhline(linewidth=1, color='m')
ax.set_xticks(np.round(xticks,1))
ax.xaxis.grid() #vertical lines

#Torque#
plot.subplot(3,2,3)
plot.plot(time[min:max],TorqueR[min:max],'k')
plot.plot(time[min:max],TorqueL[min:max],'b--')
#plot.plot(time[min:max],TorqueR[min:max] + TorqueL[min:max], 'g.')
plot.xlabel('time [ms]')
plot.ylabel('Torque (mN-m)')
plot.legend(['Right', 'Left'])
ax = fig.add_subplot(3,2,3)
ax.axhline(linewidth=1, color='m')
ax.set_xticks(np.round(xticks,1))
ax.xaxis.grid() #vertical lines

# plot.tight_layout() gives a math error
#plot.tight_layout()


# Motor PWM 
plot.subplot(3,2,5)
plot.plot(time,DCR,'k')
plot.plot(time,DCL,'b--')
plot.xlabel('time [ms]')
plot.ylabel('Duty Cycle (%)')
plot.legend(['Right', 'Left'])
ax = fig.add_subplot(3,2,5)
ax.axhline(linewidth=1, color='m')

#battery voltage
plot.subplot(3,2,6)
plot.plot(time[min:max],VBatt[min:max])
plot.xlabel('time [ms]')
plot.ylabel('Battery Voltage (V)')


#back EMF
plot.subplot(3,2,4)
plot.plot(time[min:max],RBEMF[min:max],'g')
plot.plot(time[min:max],LBEMF[min:max],'b')
plot.xlabel('time [ms]')
plot.ylabel('Back EMF (V)')
plot.legend(['Right', 'Left'])
ax = fig.add_subplot(3,2,4)
ax.axhline(linewidth=1, color='m')


# In[ ]:




# In[ ]:



