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
          ros::Subscriber obj_state_sub;
          ros::Publisher obj_pos_vec_pub ;
     ;
     private:
          float v_x=-2;
          float v_y=-0.1;
          float v_z=1.7;
          float p_x=4;
          float p_y=0;
          float p_z=0.8;
          float g=2;
          bool re_init_flag=false;
          

     public:

          collisionObjectAdder() 
          {
               add_collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
               obj_state_pub = nh.advertise<std_msgs::String>("obj_state", 1);
               obj_pos_vec_pub = nh.advertise<std_msgs::Float32MultiArray>("obj_array", 1);
               obj_state_sub = nh.subscribe("obj_state", 0, &collisionObjectAdder::chatterCallback, this);
          }
          float update_time=0.001;

          void chatterCallback(const std_msgs::String::ConstPtr& msg)
               {
               std::string my_string=msg->data.c_str();
               if(my_string=="Finished"){
                    re_init_flag=true;
               }
          }


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
               ss << "Blocking";
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
               


               //Get object state
               v_z -=g*update_time;
               p_x +=v_x*update_time;
               p_y +=v_y*update_time;
               p_z +=v_z*update_time;
               // if (p_x-0.85>-0.03 && p_x-0.85<0.03) ROS_INFO("x: %f, y: %f, z: %f",p_x, p_y, p_z);
               // std::cout<<re_init_flag;
               //if (re_init_flag || (p_z<-5 && v_z<0))
               if (re_init_flag || (p_z<-30 && v_z<0)){
                    re_init_object();
                    re_init_flag=false;
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

               //Update object pose
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

          void re_init_object(){
               
               // p_x=4+fRand(-0.1,0.1);
               // p_y=-0.15+fRand(-0.1,0.1);
               // p_z=0.84+fRand(-0.03,0.03);
               // v_z=1.7+fRand(-0.05,0.05);
               // v_x=fRand(-2.05,-1.95);
               // v_y=fRand(-0.1,0.1);

               
               double x_org=-0.4;
               double y_org=0;
               double z_org=0.9;
               double r=1.15;

               float angle=2;
               double l_x,l_y,l_z,l_time,l_p_angle,l_y_angle,l_vz;
               // while (angle>0.48){
               //      tf::Vector3 v_vector(fRand(-0.1,0.1),fRand(-0.1,0.0),fRand(-0.1,0.1));
               //      v_vector.normalize();

               //      const tf::Vector3 x_axis(1,0,0);
               //      angle = acos(v_vector.dot(x_axis));
               // }

               // l_x=x_org+v_vector[0];
               // l_y=y_org+v_vector[1];
               // l_z=z_org+v_vector[2];
               // l_time=fRand(2.5,3.5);
               // l_p_angle=fRand(0.0,0.5);
               // v_x=2*v_vector[0]/sqrt(v_vector[0]*v_vector[0]+v_vector[1]*v_vector[1]);
               // v_y=2*-v_vector[1]/sqrt(v_vector[0]*v_vector[0]+v_vector[1]*v_vector[1]);
               // l_vz=-2*sin(l_p_angle);

               l_z=fRand(0.45,0.65);
               l_y_angle=fRand(0.0,0.3);
               l_p_angle=fRand(0.7,0.85);
               v_x=-2*cos(l_y_angle);
               v_y=2*sin(l_y_angle);
               l_time=fRand(2.8,3.8);
               l_vz=2*sin(l_p_angle);

               p_x=0.2+(-v_x)*l_time;
               p_y=-v_y*l_time;

               v_z=-(l_vz-g*l_time);
               p_z=l_z+l_vz*l_time-0.5*g*l_time*l_time;
               


               sleep(4.0);

               std_msgs::String msg;

               std::stringstream ss;
               ss << "Blocking";
               msg.data = ss.str();
               ROS_INFO("%s", msg.data.c_str());
               obj_state_pub.publish(msg);
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
                    ros::spinOnce();
                    ros::Duration(coAdder.update_time).sleep();
                    
               }


               ros::spin();

               return 0;
          }
