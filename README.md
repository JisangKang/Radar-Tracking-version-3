# Radar-Tracking-version3

## Contents

1. [Clustering](#1-clustering)
2. [Radar Tracking ver.3](#2-radar-tracking-ver3)
3. [Running on ROS](#3-running-on-ros)

---
---

 ## 1. Clustering

 ### Scikit-learn.DBSCAN (Density-Based Spatial Clustering of Applications with Noise)

 
 ### Fundamental
 

 <p align="center"><img src="https://user-images.githubusercontent.com/97038348/171078139-856c053e-b4e2-45b1-9a85-647ec12f37c7.PNG" width="50%" height="50%"/>
 
 [A]Suppose that there are 7 points. [B]First, draw a circle with a radius around Point 1. The radius can be set by parameter ‘eps’. There are another parameter ‘min_samples’ which means minimum number of points in the circle of Point. In this case, the min_sample is set to 3. Because circle of Point 1 does not contain 3 points, Point 1 is not core point. [C]Second, draw a circle with a radius around Point 2. Circle of Point 2 contain 4 points and Point 2 is defined as the core point. Also, Point 1 is included in the circle of core point. Point 1 is defined as border point. The same processes are repeated on all points. [D]Point 7 does not include 3 points and is not included in other circles. Point 7 is defined as noise point. [E]Point 1~6 are recognized as an object. 

 ### Parameters
* **eps**: The maximum distance between two samples for one to be considered as in the neighborhood of the other, default=0.5
* **min_samples**: The number of samples in a neighborhood for a point to be considered as a core point, default=5
* **metric**: The metric to use when calculating distance between insatances in a feature array, default='euclidean'


<img src="https://user-images.githubusercontent.com/97038348/164665803-93d061e6-0b21-4176-8621-e106bf3c597f.png" width="30%" height="30%"/>
    
### Reference
 
https://scikit-learn.org/stable/modules/clustering.html#dbscan   (2.3.7. DBSCAN)   
https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
   
 ---
 ---
 ## 2. Radar Tracking ver.3   
 Radar Tracking version 3 is in *radar_tracking/src/ti_mmwave_rospkg/src/RadarTrackingVer3.py*
  
 ### Procedure diagram
 <img src="https://user-images.githubusercontent.com/97038348/171998821-6995030f-f9a5-4386-ae7f-9dbc83f568b4.PNG" width="80%" height="80%"/>
  
 ### Procedure 1 and 2: Obtain the points from Radar (IWR6843ISK)   
 The values (point id, x position, y position, time) of detected points are subscribed from the ti_mmwave_rospkg.   
 
        * Package name: ti_mmwave_rospkg  
        * Launch name: 6843_multi_3d_0.launch
        * Topic name: /ti_mmwave/radar_scan      
        * msg Type : ti_mmwave_rospkg/RadarScan

           point_id - The number of scaned points
           x - x-coordinate of the point
           y - y-coordinate of the point
           header.stamp.secs - second
           header.stamp.nsecs - nano second
 
 #### Coordinate
<img src="https://user-images.githubusercontent.com/97038348/170430559-b56a5096-7a4c-40f0-bca2-26d9a74ec548.PNG" width="30%" height="30%"/>
 
 
 ### Procedure 3 and 4: Clustering the points from Radar and Finding Center point   
 Point id = 0 is the criterian for separating current and previous points.    
 When the point id is 0, add 1 to zeroCount.      
 When the zeroCount exceed the maximum, the getPoint (Array) is clustered.    
 The center point from **Clustering** is assigned to the *'centerPoint'*   
 currentTime (from radar) is the average of the time of getTime (Array).     
  #### Clustering
 The eps for **Clustering** should be the size of one object(*'objectSize'*).    
<img src="https://user-images.githubusercontent.com/97038348/171994633-eb68152c-2fef-42bb-83bf-3e02092563fc.PNG" width="60%" height="60%"/>
  
 ### Procedure 5: Moving average Filter
 Because radar detect the points within the object randomly, *'centerPoint'* is not always same with real center of object.    
 This is called **"radar error"**.   
 <img src="https://user-images.githubusercontent.com/97038348/171995897-370df870-c9b4-475e-be44-f477a44dd348.PNG" width="60%" height="60%"/>
 
 The sum of radar errors of many points would be zero.   
 Therefore, radar error can be reduced by moving average filter.    
 <img src="https://user-images.githubusercontent.com/97038348/171996822-1e6a86e3-3d42-45fb-8a96-d03f650d19de.PNG" width="60%" height="60%"/>
 
 #### Formula of moving average filter
 There are Simple moving average and Weighted moving average formula to calculate the average.    
<img src="https://user-images.githubusercontent.com/97038348/171997469-01d313bb-7445-4bef-b084-3dac229f0ce4.png" width="70%" height="70%"/>
 
 ##### Reference
 https://en.wikipedia.org/wiki/Moving_average
 
 #### Window Set
 For making the moving average filter, **"Window set"** is defined.     
 <img src="https://user-images.githubusercontent.com/97038348/171997105-a192346a-4612-4eb2-b6da-2f6a8ab323cc.PNG" width="80%" height="80%"/>
 
 Window set contain k number of "window".   
 The number of window is equal to the number of objects currently observed.    
 Each window has n number of "Elements".    
 Element is 1 x 2 matrix which contains *'centerPoint'*.    
 
 When the *'centerPoint'* is obtained from previous procedures, the distance is calculated between *'centerPoint'* and first element of each window.      
 If the minimum of distances is smaller than maximum(*'filteringRange'*), the *'centerPoint'* becomes the first element of window and the previous elements of window are going down one by one.     
 <img src="https://user-images.githubusercontent.com/97038348/171998588-6382b350-b69e-475c-87ce-6ffbca47bd92.PNG" width="60%" height="60%"/>   

 The *'filtered position'* is obtained by calculating a moving average to that window.     
 
 If the minimum of distances is larger than *'filteringRange'*, the new window is created.   
 
 "skip count" is for deleting noise and old window which is not updated for long time.    
 When window is updated, skip count becomes 0. When window is not updated, 1 is added to skip count.       
 When the skip count exceeds the maximum number of skip count(*'maxNumOfSkip'*), the window would be deleted.    
 
 ### Procedure 6: Obtaining velocity
 For obtaining the velocity of object, **"Velocity Window set"** is defined.     
 <img src="https://user-images.githubusercontent.com/97038348/171998888-52feb3a8-3c36-4343-ae83-b6500b11a136.PNG" width="80%" height="80%"/>   

 Velocity Window set contain k number of "velocity window".    
 The number of velocity window is equal to the number of window.   
 Each velocity window has 2 number of "Elements".
 Element is 1 x 5 matrix which contains *'filtered position'* and *'filtered velocity'* and time.   
 
 Velocity is obtained by following equation.   
 <img src="https://user-images.githubusercontent.com/97038348/171999080-032e8dd4-ff5c-46da-b057-ef81c3418164.PNG" width="60%" height="60%"/>   

 Velocity window is also deleted with window, depending on skip count.
 
 ### Flowchart
 <img src="https://user-images.githubusercontent.com/97038348/171993981-cad844ed-e7bb-42bb-8fd5-7b0668c1118b.PNG" width="90%" height="90%"/>
 
 ---
 ---
 ## 3. Running on ROS
   TI mmWave's IWR6843ISK is used for this radar tracking.   
    <img src="https://user-images.githubusercontent.com/97038348/169230164-4c639ac4-2d81-40c8-93e9-5b02415d1502.png" width="30%" height="30%"/>
   
   
 ### 5.1 Set up

install numpy and sklearn!

    $ sudo apt install python3-pip
    $ sudo pip3 install numpy
    $ sudo pip3 install scikit-learn
   
 git clone
   
    $ cd ~
    $ git clone https://github.com/nabihandres/RADAR_Cluster-and-tracking.git
   
 catkin make
   
    $ cd ~/RADAR_Cluster-and-tracking/radar_tracking
    $ catkin_make
    $ source devel/setup.bash
   
 add dialout to have acess to the serial ports on Linux 
   
    $ sudo adduser <your_username> dialout
   
 allow permission to serial port. (with connecting radar by usb port)
   
    $ sudo chmod a+rw /dev/ttyUSB0
 
 ### 5.2 Parameter Setting
you have to set the parameters depending on what you want to detect. 
For example, if you want to detect car, then objectSize would be big.    
If you want to detect rabbit, then the objectSize would be small   
     
   
     
 #### Radar Tracking version 2  
 &nbsp;&nbsp;&nbsp;&nbsp; *# You can set these values by adjusting the PARAMETER in RadarTrackingVer2.py* 
 
 <img src="https://user-images.githubusercontent.com/97038348/168715439-9b1a7074-5f56-476c-8430-5cb68311338d.png" width="30%" height="30%"/>

 * **objectSize** = eps for Clustering
 * **filteringRange** = eps for ClusteringFilter
 * **sizeOfWindow** = number of elements in window
 * **maxNumOfSkip** = the number of maximum skip count of window. If skip count over this number, that window will be deleted.
 * **filterMode** =
   * 0: Simple moving average
   * 1: Weighted moving average
   * 2: Current-Weighted moving average
 * **weight** = weight for Current-Weighted moving average
     
     
 ### 5.3 Run the Radar Tracking
 
launch and get the data from radar

    $ roslaunch ti_mmwave_rospkg 6843_multi_3d_0.launch
    
rosrun the RadarTracking.py

    $ rosrun ti_mmwave_rospkg RadarTrackingVer2.py
    
        
        
RadarTracking.py publish following topic     
    
    Published Topic : /object_tracking
    
    msg Type : object_msgs/Objects
 
        Objects - [Object 1, Object 2, ... ]
 
    msg Type : Object_msgs/Object
 
        name - object number in Window Set
        position - [X, Y, Z]  
                    X: x-coordinate of the object
                    Y: y-coordinate of the object
                    Z: z-coordinate of the object (= 0)
        velocity - [Vx, Vy, Vz]  
                    Vx: x-velocity of the object
                    Vy: y-velocity of the object
                    Vz: z-velocity of the object (= 0)
 
 
        


    
### Reference
 
https://dev.ti.com/  (Explore/ Resource Explorer/ Software/ mmWave Sensors/ Industrial Toolbox/ Labs/ Robotics/ ROS Driver)
    
 
 
 
 
 
 
 
  
