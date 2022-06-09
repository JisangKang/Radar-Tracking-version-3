#!/usr/bin/env python
from operator import ge
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3
from ti_mmwave_rospkg.msg import RadarScan
from object_msgs.msg import Objects
from object_msgs.msg import Object
from sklearn.cluster import DBSCAN
import numpy as np

############################################################################################################################################
##################PARAMETERS################################################################################################################
objectSize=0.4           #eps for Clustering
filteringRange=0.3       #eps for ClusteringVel
sizeOfWindow=15          #number of elements in window
maxNumOfSkip=20          #maximum number of skip count
filterMode=0             #0: Simple 1: Weighted 2: Current-Weighted
weight=5                 #weight for Current-Weighted
############################################################################################################################################
############################################################################################################################################

model = DBSCAN(eps=objectSize, min_samples=2)
modelFilter = DBSCAN(eps=filteringRange, min_samples=2)

getPoint=list()
getTime=list()
WindowList=[0 for i in range(sizeOfWindow*2)]
cntWindowSkip=[0]
velWindowList=[0 for i in range(2*5)]
filteredPointVel=list()
filtering=list()
seq = [0]

empty=np.empty(shape=[0,2])


def clustering(getPointArr):    #SECOND STEP
    centerPointArr=empty

    #Clustering the points fram radar
    Clustering = model.fit_predict(getPointArr)
    
    #Calculate the center points
    for object in range(0,max(Clustering)+1):
        sumX=0
        sumY=0
        avgX=0
        avgY=0
        n=0
        corePointIndex=model.core_sample_indices_
        for index in corePointIndex:
            if Clustering[index]==object:
                sumX+=getPointArr[index][0]
                sumY+=getPointArr[index][1]
                n+=1
        avgX=sumX/n
        avgY=sumY/n
        centerPointArr=np.append(centerPointArr,[[avgX,avgY]],axis=0)
          
    return centerPointArr      #curPointArr is current center points


def movingAverageFilter(pointArr,WindowSet,VelWindowSet,Time,numWindow):
    
    skipWindowList=[i for i in range(0,numWindow)]
    updateWindowList=[]

    #Run filtering when current center point is obtained
    if len(pointArr) != 0:
        
        #THIRD STEP
        #Window Set
        for point in pointArr:
            if WindowSet[0][0][0] == 0: #Initial
                WindowSet[0][0]=point
                
            else :
                for window in range(0,numWindow):
 
                    clusterData = np.empty(shape=[2,2])
                    clusterData[0][0] = WindowSet[window][0][0]
                    clusterData[0][1] = WindowSet[window][0][1]
                    clusterData[1][0] = point[0]
                    clusterData[1][1] = point[1]

                    ClusteringFilter = modelFilter.fit_predict(clusterData)

                    #Window is founded
                    if ClusteringFilter[0] == 0: 

                        #Window is updated
                        for push in range(sizeOfWindow-1,0,-1):
                            WindowSet[window][push]=WindowSet[window][push-1]
                        WindowSet[window][0]=point
                        cntWindowSkip[window] = 0
                        updateWindowList.append(window)
                        
                        break

                    elif ClusteringFilter[0] == -1:
                        
                        #Window is not founded
                        if window == numWindow-1:

                            #New window is created
                            WindowSet=np.append(WindowSet,[np.zeros(shape=[sizeOfWindow,2])],axis=0)
                            WindowSet[len(WindowSet)-1][0]=point
                            cntWindowSkip.append(0)
                            VelWindowSet=np.append(VelWindowSet,[np.zeros(shape=[2,5])],axis=0)
                            
        #Add 1 to skip count of window which is not updated
        for skip in list(set(skipWindowList)-set(updateWindowList)):
            cntWindowSkip[skip] += 1

        #Delete window and velocity window when skip count exceed maximum number of skip count
        while (maxNumOfSkip in cntWindowSkip):
            index=cntWindowSkip.index(maxNumOfSkip)
            del cntWindowSkip[index]
            WindowSet=np.delete(WindowSet,index,axis=0)
            VelWindowSet=np.delete(VelWindowSet,index,axis=0)
        
        #Obtain the filtered point
        for win in range(0,len(WindowSet)):
            if WindowSet[win][sizeOfWindow-1][0] != 0 and cntWindowSkip[win] == 0:
                if filterMode == 0: # Simple
                    filtering=list(WindowSet[win].mean(axis=0))

                elif filterMode == 1: # Weighted
                    Weighted = np.zeros(shape=2)
                    den=0
                    for i in range(0,sizeOfWindow):
                        Weighted += np.multiply(WindowSet[win][i],(sizeOfWindow-i))
                        den += (i+1)
                    Weighted = Weighted/den
                    filtering=list(Weighted)

                elif filterMode == 2: # Current-Weighted
                    filtering=list((WindowSet[win].mean(axis=0)*sizeOfWindow+WindowSet[win][0]*(weight-1))/(sizeOfWindow+weight-1))

                #FOURTH STEP
                #Get the filtered velocity
                #Velocity Window Set
                VelWindowSet[win][1]=VelWindowSet[win][0]
                VelWindowSet[win][0][0]=filtering[0]
                VelWindowSet[win][0][1]=filtering[1]
                VelWindowSet[win][0][4]=Time
                VelWindowSet[win][0][2]=(VelWindowSet[win][0][0]-VelWindowSet[win][1][0])/(VelWindowSet[win][0][4]-VelWindowSet[win][1][4])
                VelWindowSet[win][0][3]=(VelWindowSet[win][0][1]-VelWindowSet[win][1][1])/(VelWindowSet[win][0][4]-VelWindowSet[win][1][4])
                filteredPointVel.append(list([VelWindowSet[win][0][0],VelWindowSet[win][0][1],VelWindowSet[win][0][2],VelWindowSet[win][0][3]]))

    return WindowSet, VelWindowSet
                        


            



def callback(data):

    #FIRST STEP
    #Get the points from radar
    getPointId=data.point_id
    getPointX=data.x
    getPointY=data.y
    getPointTime=data.header.stamp.secs+float(data.header.stamp.nsecs)/1000000000

    if getPointId == 0:

        getPointArr=np.array(getPoint)            

        #current Time is average time of points  
        curTimeArr=np.array(getTime)  
        curTime=np.mean(curTimeArr)
        
        numWindow=int(len(WindowList)/(sizeOfWindow*2))
        pastWindowSet=np.empty(shape=[numWindow,sizeOfWindow,2])
        pastVelWindowSet=np.empty(shape=[numWindow,2,5])

        for i in range(0,numWindow):
            for j in range(0,sizeOfWindow):
                for k in range(0,2):
                    pastWindowSet[i][j][k]=WindowList[sizeOfWindow*2*i+2*j+k]

        for i in range(0,numWindow):
            for j in range(0,2):
                for k in range(0,5):
                    pastVelWindowSet[i][j][k]=velWindowList[2*5*i+5*j+k]

        #SECOND STEP
        #Clustering the points from radar and Get the current center points
        centerPointArr = clustering(getPointArr)

        #THIRD STEP & FOURTH STEP
        #Moving Average Filtering
        curWindowSet, curVelWindowSet = movingAverageFilter(centerPointArr,pastWindowSet,pastVelWindowSet,curTime,numWindow)

        #Publish the matrix
        if filteredPointVel !=[]:
            pub=rospy.Publisher('object_tracking', Objects, queue_size=1)
            pl=Objects()
            seq[0] += 1
            pl.header.seq = seq[0]
            pl.header.stamp = rospy.Time.now()
            for i in range(0,len(filteredPointVel)):
                
                p = Object()
                p.name = str("Object ")+str(i+1)
                p.position = Vector3(filteredPointVel[i][0],filteredPointVel[i][1],0)
                p.velocity = Vector3(filteredPointVel[i][2],filteredPointVel[i][3],0)
                pl.objects.append(p)
            pub.publish(pl)
            print(pl)
        
        
        
        #Initialize the variable
        del getPoint[:]
        del getTime[:]
        del WindowList[:]
        del velWindowList[:]
        del filteredPointVel[:]

        for i in range(0,len(curWindowSet)):
            for j in range(0,sizeOfWindow):
                for k in range(0,2):
                    WindowList.append(curWindowSet[i][j][k])
        for i in range(0,len(curVelWindowSet)):
            for j in range(0,2):
                for k in range(0,5):
                    velWindowList.append(curVelWindowSet[i][j][k])
        


    getPoint.append([getPointX,getPointY])
    getTime.append(getPointTime)

    
    

def listener():

    #Making a clustering node
    rospy.init_node('clustering')

    #Subscribing the /ti_mmwave/radar_scan topic from RADAR
    rospy.Subscriber("/ti_mmwave/radar_scan", RadarScan, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()