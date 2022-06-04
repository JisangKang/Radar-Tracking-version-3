# Radar-Tracking-version3

## Contents

1. [Clustering](#1-clustering)
2. [Radar Tracking ver.3](#2-radar-tracking-ver3)
3. [Experiments](#3-experiments)
4. [Running on ROS](#4-running-on-ros)

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

### Attributes
 * **core_sample_indices_**: Indices of core samples
 * **components_**: Copy of each core sample found by training
 * **labels_**: Cluster labels for each point in the dataset given to fit(). Noisy samples are give the label -1
 

### Examples
    
    from sklearn.cluster import DBSCAN
    import numpy as np
    
    model = DBSCAN(eps=0.5, min_samples=1)
    # vector array
    data = np.array([[1,1],[2,0],[3,0],[1,0]])       
    Clustering = model.fit_predict(data)
    print(Clustering)
    
### Reference
 
https://scikit-learn.org/stable/modules/clustering.html#dbscan   (2.3.7. DBSCAN)   
https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
   
 ---
 ---
 ## 2. Radar Tracking ver.3   
 Radar Tracking version 3 is in *radar_tracking/src/ti_mmwave_rospkg/src/RadarTrackingVer3.py*
 ### Flowchart
 <img src="https://user-images.githubusercontent.com/97038348/171993981-cad844ed-e7bb-42bb-8fd5-7b0668c1118b.PNG" width="90%" height="90%"/>
  
 ### Process diagram
 <img src="https://user-images.githubusercontent.com/97038348/171994089-372dbeb9-e91a-4843-b7bf-2a422c0772cf.PNG" width="90%" height="90%"/>
  
 ### Process 1 and 2: Obtain the points from Radar (IWR6843ISK)   
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
 
 #### Specification
Detection range in radial axis : 175(car), 98(person) [m]

Range resolution : 0.039 ~ 0.044 [m]

Azimuth angle of detection : ±60 [deg]

Elevation angle of detection : ±20 [deg]

Angular resolution (Azimuth) : 20 [deg]

Angular resolution (Elevation) : 58 [deg]

Maximum radial velocity : 9.59 [m/s]

(Note: Radial velocity represents how fast object go close or far from RADAR. It does not contain transverse velocity.)
   
   #### Reference 
   https://www.ti.com/design-resources/embedded-development/industrial-mmwave-radar-sensors.html#Evaluation
 
 ### Procedure 3 and 4: Clustering the points from Radar and Finding Center point   
 Point id =0 is the criterian for separating current and previous points.    
 When the point id is 0, add 1 to zeroCount.      
 When the zeroCount exceed the maximum, the getPoint (Array) is clustered.    
  
 #### Clustering
 The eps for **Clustering** should be the size of one object.    
<img src="https://user-images.githubusercontent.com/97038348/171994633-eb68152c-2fef-42bb-83bf-3e02092563fc.PNG" width="50%" height="50%"/>
  
 The center point from **Clustering** is assigned to the *'centerPoint'*   
 currentTime (from radar) is the average of the time of getTime (Array).   
  
 ### Procedure 5: Moving average Filter
 *'centerPoint'* is not always same with real center of object.
 This is called "radar error".
 
  
