#include<stdio.h>
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shapes.h"
#include <shape_msgs/SolidPrimitive.h>
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"

     class collisionObjectAdder
     {
     protected:

          ros::NodeHandle nh;
          ros::Publisher add_collision_object_pub; 
          ros::Publisher obj_state_pub;
          // ros::Subscriber obj_state_sub;
          ros::Publisher obj_pos_vec_pub ;
     ;
     private:
          float v_x=-2;
          float v_y=-0.1;
          float v_z=1.7;
          float p_x=4;
          float p_y=0;
          float p_z=1;
          

     public:

          collisionObjectAdder() 
          {
               add_collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
               obj_state_pub = nh.advertise<std_msgs::String>("obj_state", 1);
               obj_pos_vec_pub = nh.advertise<std_msgs::Float32MultiArray>("obj_array", 1);
               // obj_state_sub = nh.subscribe("obj_state", 1, &collisionObjectAdder::chatterCallback, this);
          }
          float update_time=0.05;

          // void chatterCallback(const std_msgs::String::ConstPtr& msg)
          //      {
          //      ROS_INFO("I heard: [%s]", msg->data.c_str());
          // }


          void addCollisionObject()
               {
               sleep(1.0); // To make sure the node can publish  
               moveit_msgs::CollisionObject co;
               
               co.header.frame_id = "base_link";
               co.id="sphere";
               // Define the shape size of the object
               shape_msgs::SolidPrimitive primitive;
               primitive.type=primitive.SPHERE;
               primitive.dimensions.resize(1); //dimensions is a vector with 3 element spaces assigned to it
               primitive.dimensions[0] =0.06; //length on the x-axis
               //     primitive.dimensions[1] =0.1; //length on the y-axis
               //     primitive.dimensions[2] =0.1; //length on the z-axis
               
               // Define the object orientation
               geometry_msgs::Pose pose;
               
               pose.position.x =p_x;
               pose.position.y =p_y;
               pose.position.z =p_z;
               // Add the shape to obj
               co.primitives.push_back(primitive);
               co.primitive_poses.push_back(pose);
               // Define the operation to add
               co.operation = co.ADD;
               
               

               add_collision_object_pub.publish(co);
               ROS_INFO("Collision object published");
               
               // Publish object state
               std_msgs::String msg;
               std::stringstream ss;
               ss << "flying";
               msg.data = ss.str();
               ROS_INFO("%s", msg.data.c_str());
               obj_state_pub.publish(msg);


               std_msgs::Float32MultiArray array;
		
		     array.data.clear();
               
			array.data.push_back(p_x);
               array.data.push_back(p_y);
               array.data.push_back(p_z);
               array.data.push_back(v_x);
               array.data.push_back(v_y);
               array.data.push_back(v_z);
		
		     obj_pos_vec_pub.publish(array);
               }
          
          bool get_if_query(){
               bool query;
               if (!nh.getParam("query", query)) {
                    ROS_ERROR("Failed to read 'query' from the param server");
               }
               return query;
          }

          void updateCollisionObject()
               {
               moveit_msgs::CollisionObject co;
               
               co.header.frame_id = "base_link";
               co.id="sphere";
               // Define the shape size of the object
               shape_msgs::SolidPrimitive primitive;
               primitive.type=primitive.SPHERE;
               primitive.dimensions.resize(1); //dimensions is a vector with 3 element spaces assigned to it
               primitive.dimensions[0] =0.1; //length on the x-axis
              
               v_z -=2*update_time;
               p_x +=v_x*update_time;
               p_y +=v_y*update_time;
               p_z +=v_z*update_time;
               if (p_x<0.4){
                    re_init_object();
                    return;
               }

               // Define the object orientation
               geometry_msgs::Pose pose;
               pose.position.x =p_x;
               pose.position.y =p_y;
               pose.position.z =p_z;
               
               // Add the shape to obj
               co.primitives.push_back(primitive);
               co.primitive_poses.push_back(pose);
               // Define the operation to add
               co.operation = co.MOVE;
               
               
               add_collision_object_pub.publish(co);
               // ROS_INFO("Collision object updated %f",pose.position.x);
               }

          void re_init_object(){
               sleep(1.0);
               p_x=4+fRand(-0.1,0.1);
               p_y=-0.1+fRand(-0.1,0.1);
               p_z=1+fRand(-0.1,0.1);
               v_z=1.7+fRand(-0.05,0.05);
               v_x=fRand(-2.05,-1.95);
               v_y=fRand(-0.03,0.03);

               std_msgs::String msg;

               std::stringstream ss;
               ss << "flying";
               msg.data = ss.str();
               ROS_INFO("%s", msg.data.c_str());
               obj_state_pub.publish(msg);


               std_msgs::Float32MultiArray array;
		
		     array.data.clear();
               
			array.data.push_back(p_x);
               array.data.push_back(p_y);
               array.data.push_back(p_z);
               array.data.push_back(v_x);
               array.data.push_back(v_y);
               array.data.push_back(v_z);
		
		     obj_pos_vec_pub.publish(array);
               // ROS_INFO("update array %f", p_x);
          }

          float fRand(float fMin, float fMax)
               {
               float f = (float)std::rand() / RAND_MAX;
               return fMin + f * (fMax - fMin);
               }

          };



          int main(int argc, char** argv)
          {
               
               ros::init(argc, argv, "add_collision_object");
               std::cout<<"Initialized..." << std::endl;
               collisionObjectAdder coAdder; 
               bool query=coAdder.get_if_query();
               
               if (query==false) return 0;
               coAdder.addCollisionObject();

               while (ros::ok()){
                    coAdder.updateCollisionObject();
                    ros::Duration(coAdder.update_time).sleep();
               }


               ros::spin();

               return 0;
          }
