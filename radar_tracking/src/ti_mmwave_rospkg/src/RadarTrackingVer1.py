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
objectMovingRange=0.3    #eps for ClusteringVel 
filteringRange=0.3       #eps for ClusteringFilter
sizeOfWindow=15          #number of elements in window
maxNumOfSkip=20          #maximum number of skip count 
filterMode=0             #0: Simple 1: Weighted 2: Current-Weighted
weight=3                 #weight for Current-Weighted
############################################################################################################################################
############################################################################################################################################

model = DBSCAN(eps=objectSize, min_samples=2)
modelVel = DBSCAN(eps=objectMovingRange, min_samples=1) 
modelFilter = DBSCAN(eps=filteringRange, min_samples=2)
getPoint=list()
pastPoint=list()
getTime=list()
pastTimeList=list()
filterList=[0 for i in range(sizeOfWindow*4)]
cntWindowSkip=[0]
filteredPointVel=list()
seq=[0]

empty=np.empty(shape=[0,2])


def clustering(getPointArr): #SECOND STEP
    curPointArr=empty

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
        curPointArr=np.append(curPointArr,[[avgX,avgY]],axis=0)   
    return curPointArr, Clustering    #curPointArr is current center points


def velocity(curPointArr,pastPointArr,curTimeAvg,pastTimeAvg):  #THIRD STEP

    curPointNum=len(curPointArr)
    pastPointNum=len(pastPointArr)
    uniCurPast=empty                             #unionCurrentPast
    curPointVel=np.array([])                     #currentPointVelocity
    timeDifference=curTimeAvg-pastTimeAvg
    

    #Calculate the velocity
    if curPointNum != 0 and pastPointNum != 0:

        #Combine current and previous center points
        uniCurPast=np.append(uniCurPast,curPointArr,axis=0)
        uniCurPast=np.append(uniCurPast,pastPointArr,axis=0)
        
        #Clustering the current and previous center points
        ClusteringVel = modelVel.fit_predict(uniCurPast)
        corePointArr=modelVel.components_        #core points of Clusteringvel
        for curPoint in range(0,curPointNum):
            for pastPoint in range(curPointNum,curPointNum+pastPointNum):
                if ClusteringVel[curPoint] == ClusteringVel[pastPoint]:
                    vel=(corePointArr[curPoint]-corePointArr[pastPoint])/timeDifference
                    curPointVel=np.append(curPointVel,corePointArr[curPoint][0])
                    curPointVel=np.append(curPointVel,corePointArr[curPoint][1])
                    curPointVel=np.append(curPointVel,vel[0])
                    curPointVel=np.append(curPointVel,vel[1])
      
        return curPointVel # X Y Vx Vy Array (before filtering)
        
        
def movingAverageFilter(curPointVel,WindowSet,numWindow):  #FOURTH STEP

    skipWindowList=[i for i in range(0,numWindow)]
    updateWindowList=[]
    
    #Run filtering when current center point is obtained
    if type(curPointVel) != type(None) and curPointVel != []:

        for pv in range(len(curPointVel)/4):
            
            pointVel=[curPointVel[4*pv],curPointVel[4*pv+1],curPointVel[4*pv+2],curPointVel[4*pv+3]]
            
            if WindowSet[0][0][0] == 0:   #Initial
                WindowSet[0][0]=pointVel
            else : 
                for window in range(0,numWindow):

                    clusterData = np.empty(shape=[2,2])
                    clusterData[0][0] = WindowSet[window][0][0]
                    clusterData[0][1] = WindowSet[window][0][1]
                    clusterData[1][0] = pointVel[0]
                    clusterData[1][1] = pointVel[1]

                    ClusteringFilter = modelFilter.fit_predict(clusterData)
                    
                    #Window is founded 
                    if ClusteringFilter[0] == 0:

                        #Window is updated
                        for push in range(sizeOfWindow-1,0,-1):
                            WindowSet[window][push]=WindowSet[window][push-1]
                        WindowSet[window][0]=pointVel
                        cntWindowSkip[window] = 0
                        updateWindowList.append(window)
                        
                        break

                    elif ClusteringFilter[0] == -1:
                        
                        #Window is not founded
                        if window == numWindow-1:

                            #New window is created
                            WindowSet=np.append(WindowSet,[np.zeros(shape=[sizeOfWindow,4])],axis=0)
                            WindowSet[len(WindowSet)-1][0]=pointVel
                            cntWindowSkip.append(0)

        #Add 1 to skip count of window which is not updated
        for skip in list(set(skipWindowList)-set(updateWindowList)):
            cntWindowSkip[skip] += 1

        #Delete window when skip count exceed maximum number of skip count
        while (maxNumOfSkip in cntWindowSkip):
            index=cntWindowSkip.index(maxNumOfSkip)
            del cntWindowSkip[index]
            WindowSet=np.delete(WindowSet,index,axis=0)

        #Get the filtered position and velocity
        for win in range(0,len(WindowSet)):
            if WindowSet[win][sizeOfWindow-1][0] != 0 and cntWindowSkip[win] == 0:

                if filterMode == 0: # Simple
                    filteredPointVel.append(list(WindowSet[win].mean(axis=0)))

                elif filterMode == 1: # Weighted
                    Weighted = np.zeros(shape=4)
                    den=0
                    for i in range(0,sizeOfWindow):
                        Weighted += np.multiply(WindowSet[win][i],(sizeOfWindow-i))
                        den += (i+1)
                    Weighted = Weighted/den
                    filteredPointVel.append(list(Weighted))

                elif filterMode == 2: # Current-Weighted
                    filteredPointVel.append(list((WindowSet[win].mean(axis=0)*sizeOfWindow+WindowSet[win][0]*(weight-1))/(sizeOfWindow+weight-1)))

        
    return WindowSet
        
           
def callback(data):
    
    #FIRST STEP
    #Get the points from radar
    getPointId=data.point_id
    getPointX=data.x
    getPointY=data.y
    time=data.header.stamp.secs+float(data.header.stamp.nsecs)/1000000000

    if getPointId == 0:
                
        getPointArr=np.array(getPoint)
        pastPointArr=np.array(pastPoint)         
        
        #current Time is average time of points
        curTimeArr=np.array(getTime)  
        curTime=np.mean(curTimeArr)
        
        pastTimeArr=np.array(pastTimeList)
        pastTime = np.mean(pastTimeArr)
        
        numWindow=int(len(filterList)/(sizeOfWindow*4))
        
        pastWindowSet=np.empty(shape=[numWindow,sizeOfWindow,4])
        
        for i in range(0,numWindow):
            for j in range(0,sizeOfWindow):
                for k in range(0,4):
                    pastWindowSet[i][j][k]=filterList[sizeOfWindow*4*i+4*j+k]

        #SECOND STEP
        #Clustering the points from radar and Get the current center points
        curPointArr,Clustering = clustering(getPointArr)

        #THIRD STEP
        #Get the point and velocity (before filtering)    
        curPointVel=velocity(curPointArr,pastPointArr,curTime,pastTime)

        #FOURTH STEP        
        #Moving Average Filtering
        curWindowSet=movingAverageFilter(curPointVel,pastWindowSet,numWindow)
        
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
        del pastPoint[:]
        del getTime[:]
        del pastTimeList[:]
        del filterList[:] 
        del filteredPointVel[:]

        #Assign current value to past value      
        for i in curPointArr:
            pastPoint.append([i[0],i[1]])
        for i in curTimeArr:
            pastTimeList.append(i)
        for i in range(0,len(curWindowSet)):
            for j in range(0,sizeOfWindow):
                for k in range(0,4):
                    filterList.append(curWindowSet[i][j][k])
       
    getPoint.append([getPointX,getPointY])
    getTime.append(time)
    
    
def listener():

    #Making a clustering node
    rospy.init_node('clustering')

    #Subscribing the /ti_mmwave/radar_scan topic from RADAR
    rospy.Subscriber("/ti_mmwave/radar_scan", RadarScan, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()



    

