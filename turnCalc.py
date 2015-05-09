###################
#
#  not useful when called from iPython, as need to restart each time
#
########################
# calculate induced turn as function of offset leg phase

import numpy as np
import matplotlib
import matplotlib.pyplot as plot

length = 16
width = 8

def turnCalc(leftPos, rightPos,  AngleZ):
    fig = plot.figure(figsize = (length,width))
    #leg angle vs gyro angle
    plot.subplot(1,2,1)
    plot.xlabel('right leg angle (rad)')
    # Total Angle in radians
    plot.plot(-rightPos,AngleZ, 'g') # gyro angle
    plot.plot(-rightPos, -rightPos-leftPos,'b--') # difference in leg angles
    plot.ylabel('Angle Est.(rad)')
    plot.legend(['GZ','$\Delta \Theta$'])
    ax = fig.add_subplot(1,2,1)
    ax.axhline(linewidth=1, color='m')
    xticks=np.linspace(0,11,12)*np.pi
    ax.set_xticks(np.round(xticks,1))
    ax.xaxis.grid() #vertical lines

   



# test data
leftPos=np.array([0,1,2,3,4,5,6])
rightPos=np.array([0,0.9,1.8,2.7,3.6,4.5,5.4])
AngleZ=([0,0.1,0.2,0.3,0.0,0.2,-0.2])
turnCalc(leftPos, rightPos,  AngleZ)
