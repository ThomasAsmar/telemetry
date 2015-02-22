# calculate induced turn as function of offset leg phase

import numpy as np
import matplotlib
import matplotlib.pyplot as plot

length = 16
width = 8

def turnCalc(leftPos, rightPos,  AngleZ):
    fig = figure(figsize = (length,width))
    #leg angle vs gyro angle
    plot.subplot(1,2,1)
    xlabel('right leg angle [ms]')
    # Total Angle in radians
    plot.plot(-rightLegPos,AngleY,'k')
    plot.plot(-rightLegPos,AngleZ, 'g')
    ylabel('Angle Est.(rad)')
    legend(['GY', 'GZ'])
    ax = fig.add_subplot(3,2,6)
    ax.axhline(linewidth=1, color='m')
    xticks=np.linspace(0,11,12)*np.pi
    ax.set_xticks(np.round(xticks,1))
    ax.xaxis.grid() #vertical lines



# test data
leftPos=[0,1,2,3,4,5,6]
rightPos=[0,0.9,1.8,2.7,3.6,4.5,5.4]
AngleZ=[0,0.1,0.2,0.3,0.0,0.2,-0.2]
turnCalc(leftPos, rightPos,  AngleZ)
