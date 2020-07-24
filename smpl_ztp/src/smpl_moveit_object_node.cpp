#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shapes.h"
#include <shape_msgs/SolidPrimitive.h>

    class collisionObjectAdder
    {
    protected:

         ros::NodeHandle nh;
         ros::Publisher add_collision_object_pub; 
         //ros::Publisher planning_scene_diff_pub;
    ;

    public:

       collisionObjectAdder() 
       {
            add_collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
       }

       void addCollisionObject()
       {
         sleep(1.0); // To make sure the node can publish  
         moveit_msgs::CollisionObject co;
         // shapes::Mesh* m = shapes::createMeshFromResource("package://motoman_sda10f_support/meshes/bin.stl");
         
         // ROS_INFO("mesh loaded");
         // shape_msgs::Mesh co_mesh;
         // shapes::ShapeMsg co_mesh_msg;  
         // shapes::constructMsgFromShape(m, co_mesh_msg);    
         // co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);  
         // co.meshes.resize(1);
         // co.mesh_poses.resize(1);
         // co.meshes[0] = co_mesh;
         // co.header.frame_id = "mainulator";
         // co.id = "ring";     
         // co.mesh_poses[0].position.x = 0.75;
         // co.mesh_poses[0].position.y = 0.0;
         // co.mesh_poses[0].position.z = 0.525;
         // co.mesh_poses[0].orientation.w= 1.0;
         // co.mesh_poses[0].orientation.x= 0.0;
         // co.mesh_poses[0].orientation.y= 0.0;
         // co.mesh_poses[0].orientation.z= 0.0;   

         // co.meshes.push_back(co_mesh);
         // co.mesh_poses.push_back(co.mesh_poses[0]);
         // co.operation = co.ADD;

	  
         co.header.frame_id = "base_link";
         co.id="box";
         // Define the shape size of the object
         shape_msgs::SolidPrimitive primitive;
         primitive.type=primitive.BOX;
         primitive.dimensions.resize(3); //dimensions is a vector with 3 element spaces assigned to it
         primitive.dimensions[0] =0.1; //length on the x-axis
         primitive.dimensions[1] =0.1; //length on the y-axis
         primitive.dimensions[2] =0.1; //length on the z-axis
         // Define the object orientation
         geometry_msgs::Pose pose;
         pose.orientation.w =1.0;
         // Add the shape to obj
         co.primitives.push_back(primitive);
         co.primitive_poses.push_back(pose);
         // Define the operation to add
         co.operation = co.ADD;
	
	

          add_collision_object_pub.publish(co);
          ROS_INFO("Collision object published");
       }
    };


    int main(int argc, char** argv)
    {
            ros::init(argc, argv, "add_collision_object");
            std::cout<<"Initialized..." << std::endl;
            collisionObjectAdder coAdder; 

            coAdder.addCollisionObject();

            ros::spin();

            return 0;
    }
