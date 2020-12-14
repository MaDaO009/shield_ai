Shield Ai
============
# Getting start
## A.Download code from Github    
### Sbpl (Download to src folder of your catkin workspace and follow its instruction to install)    
https://github.com/sbpl/sbpl
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/1.png)  



### Smpl (Download to src folder of the catkin workspace and follow the instruction to install)  
https://github.com/aurone/smpl   
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/2.png)




### Our Planner (Download to src folder of the catkin workspace then use “catkin_make_isolated" to compile the whole project)
https://github.com/MaDaO009/shield_ai
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/3.png)


## B.Run with preprocess mode first, which may take about two hours.
Set query to “false” and launch the programs by “roslaunch smpl_ztp goal_pr2.launch”
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/4.png)

## C.Run simulation with query mode
After the preprocessing phase ends, change “query” to “true” and launch again 	(“roslaunch 	smpl_ztp goal_pr2.launch”)
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/5.png)
# II.About Project
## A.Simulation
We have two packages, “smpl_moveit_object” and “smpl_ztp”, for simulation. The former launches a node updating object’s position and velocity while the latter launches a node to generate corresponding plans and let the robot execute them to block the object. There are two topics, “obj_state” (although it’s called object state, it actually indicates whether the blocking process is restarting or finishing) and “obj_array”, for intercommunication of these two nodes. “obj_array” contains the position and velocity of the object.


The object-updater node creates a spherical object with a radius of 10cm and keeps updating its position and velocity. The object will be moving along a parabolic trajectory as we currently assume that the only force on it is gravity force. When the node receives a “finished” signal from “obj_state” or the object’s position excesses a threshold, the node will reinitialize the object with a random position and velocity (the object will be aimed at the robot).
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/6.png)



The planner node subscribes the two aforementioned topics. After the object has been launched, the planner will predict the landing pose of the object once, generate a plan for robot’s right arm to block the object, execute the plan and execute the reverse plan to get the arm back to its start configuration when the blocking process is finished. To do this, we employ three flags: “to_block”, “if_get_prediction” and “if_reach”. The flow chart below shows how this node works, where the left part demonstrates the basic idea and the right part shows details within the main function and callback function.
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/7.png)
## B.Preprocess
The preprocessing part is almost the same with the paper Provable Indefinite-Horizon Real-Time Planning for Repetitive Tasks. The code starts from PreProcess() in zero_time_planner.cpp

## C.Query
The query part starts from solveZero() in planner_interface.cpp. I’ve put comment in the function and the program structure is shown in the flow chart above.

## D.Visualization of attractors
Visualize attractors by selecting attractor items to show how much area the planner can cover
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/8.png)
## E.Additional transformation
When we plan for the right arm, the goal state is for the last frame of its end effector instead of the shield while we wanna control the shield. We therefore need to get the shield’s pose from the last frame when we check if the shield pose is within the predefined goal region. The corresponding code is in IsWorkspaceStateInGoalRegion() in workspace_lattice_zero.cpp. Besides, in query phase, after we get the landing pose of the object, we need to get the last frame from shield pose in order to query plans (see arrayCallback() in planner_interface.cpp)

## F.Collision checking
In order to check collision of the shield and the object, the planner needs to synchronize its planning scene with “/move_group/monitored_planning_scene”
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/9.png)
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/10.png)
And we set the AllowedCollisionMatrix only contains the shield and the object:
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/11.png)
To print acm:
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/12.png)
The collision checking is done by 
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/13.png)
**TO DO: implement collision checking between the robot body and the object (indicator of being failed to block). You can create another acm where only the collision of robot body links and the object will be checked.**

# III.To modify the code
## A.Change goal region/surface definition
Need to change: 
WorkspaceLatticeZero::readGoalRegion()
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/14.png)
WorkspaceLatticeZero::IsWorkspaceStateInGoalRegion:
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/15.png)
WorkspaceLatticeZero::SampleRobotState: (make sure the first state is within the goal region)
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/16.png)
PlannerInterface::arrayCallback:
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/17.png)
![image](https://github.com/MaDaO009/shield_ai/blob/master/figs/18.png)

**TO DO: write those definition into rosparams to avoid taking so many modifications.**

## B.Change the shield
### 1: Design a shield using Solidworks and export STL file
### 2: Download Solidworks URDF Exporter Plugin and export URDF file
### 3: Add the urdf code to {pr2_description}/urdf/gripper_v0/gripper.urdf.xacro 
e.g.
<!-- Shield -->
			    <xacro:if value="${prefix=='r_gripper'}">
			      <joint name="${prefix}_shield_joint" type="fixed">
				<axis xyz="0 0 -1" />
				<origin xyz="0.02 -0.02 0.02" rpy="0 0 0" />
				<!-- <xacro:pr2_finger_limits_v0/> -->
				<!-- <dynamics damping="${finger_tip_damping}" /> -->
				<!-- <mimic joint="${prefix}_l_finger_joint" multiplier="1" offset="0"/> -->
				<parent link="${prefix}_l_finger_tip_link"/>
				<child link="${prefix}_shield"/>
			      </joint>
			      <link name="${prefix}_shield">

				<inertial>
				  <mass value="0.27042" />
				  <origin xyz="0.01 -0.02 0.0" rpy="0 0 0" />
				  <inertia  ixx="0.00074319"   ixy="7.157E-20"   ixz="-8.379E-21"
				  iyy="0.00074319"  iyz="1.14490000004451E-09"
				  izz="0.0014701" />
				</inertial>
				<visual>
				  <origin
				    xyz="0 0 0"
				    rpy="0 0 4.71" />
				  <geometry>
				    <mesh
				      filename="package://pr2_description/meshes/gripper_v0/test.STL" />
				  </geometry>
				  <material
				    name="">
				    <color
				      rgba="0.70588 0.70588 0.70588 1" />
				    <!-- <texture
				      filename="package://pr2_description/meshes/gripper_v0/textures/" /> -->
				  </material>
				</visual>
				<collision>
				  <origin
				    xyz="0 0 0"
				    rpy="0 0 4.71" />
				  <geometry>
				    <mesh
				      filename="package://pr2_description/meshes/gripper_v0/test.STL" />
				  </geometry>
				</collision>
			      </link> 
			    </xacro:if>
### 4: Add STL file to {pr2_description}/meshes/gripper_v0/


## C.Change Resolution of graph (Seems to have some issues now)
in workspace_lattice_base.cpp ::InitSimpleWorkspaceLatticeActions  
		m_res[0] = _params.res_x;  
		m_res[1] = _params.res_y;  
		m_res[2] = _params.res_z;  
		make resolution 0.02  

In simple_workspace_lattice_action_space.cpp ::InitSimpleWorkspaceLatticeActions  
		d[FK_PX] = 0.005 * dx;  
		d[FK_PY] = 0.005 * dy;  
		d[FK_PZ] = 0.005 * dz;  

## D.To add obstacles  
Either add them in smpl_moveit_object or in smpl_ztp/teleop.env  
An example of smpl_ztp/teleop.env:  
	4  
	obj0 0.5 0.6 0.47175 0.26 1.8 0.04  
	obj1 0.37 -0.3 0.77175 0.04 0.04 0.64  
	obj2 0.63 -0.3 0.77175 0.04 0.04 0.64  
	obj3 0.5 -0.3 1.07175 0.26 0.04 0.04  
	obj4 0.37 0.6 0.47175 0.02 1.8 0.3  


# IV Current Issues
## A.Coverage: The robot is not able to cover the whole surface under current setting. The reason may be that free angle (the angle of redundant joint) is limited, or PR2 is inherently unable to “protect” its head in our task.
## B.#Subregions: The region will may be unacceptable when the limitation of Euler angle and free angle loose. Can we find some ways to reduces the number of subregions?
## C.Execution: Current execution takes too much time. (I have to set gravity acceleration to -2m/s2 to better demonstrate the result.) And a minor problem is that the planner loads robot model several times during the execution.
