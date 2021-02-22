#ifndef _LIDAR_PLUGIN_HH_
#define _LIDAR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

namespace gazebo
{
  /// \brief A plugin to control a RPLiDAR sensor.
  class LidarPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: LidarPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
	  if (_model->GetJointCount() == 0)
	  {
	    std::cerr << "Invalid joint count, lidar plugin not loaded\n";
	    return;
	  }

	  // Store the model pointer for convenience.
	  this->model = _model;

	  // Get the first joint. We are making an assumption about the model
	  // having one joint that is the rotational joint.
	  this->joint1 = _model->GetJoints()[0];
	  this->pid1 = common::PID(3, 5, 0);
	  //this->GetErrors = common::GetErrors (20,20,10);
	  
	  //std::cerr << this->pid1::GetErrors (20,20,10);
	  
	  this->model->GetJointController()->SetPositionPID(
	      this->joint1->GetScopedName(), this->pid1);

	  this->model->GetJointController()->SetPositionTarget(
	      this->joint1->GetScopedName(), 0.0);
	  /*    std::cerr << "Joint1 ";
	      std::cerr << joint1->Position() << "\r\n";*/
	      
	  this->joint2 = _model->GetJoints()[1];
	  this->pid2 = common::PID(60, 60, 0);
	  
	  this->model->GetJointController()->SetPositionPID(
	      this->joint2->GetScopedName(), this->pid2);

	  this->model->GetJointController()->SetPositionTarget(
	      this->joint2->GetScopedName(), 0.0);
	  /*    std::cerr << "Joint2: ";
	      std::cerr << joint2->Position() << "\r\n";*/
	  
	  /*double position1 = 0;
	  double position2 = 0;
	  
	  if (_sdf->HasElement("position1"))
	  position1 = _sdf->Get<double>("position1");

	//this->model->GetJointController()->SetPositionTarget(
	    //this->joint->GetScopedName(), position1);   
	    
	  if (_sdf->HasElement("position2"))
	  position2 = _sdf->Get<double>("position2");*/

	//this->model->GetJointController()->SetPositionTarget(
	    //this->joint->GetScopedName(), position2);  
	  //this ->SetPosition(position1); 
	  //this ->SetPosition(position2); 
	
	// Create the position node
	this->nodePos = transport::NodePtr(new transport::Node());
	#if GAZEBO_MAJOR_VERSION < 8
	this->nodePos->Init(this->model->GetWorld()->GetName());
	#else
	this->nodePos->Init(this->model->GetWorld()->Name());
	#endif

	// Create a topic name
	std::string topicNamePos = "~/" + this->model->GetName() + "/pos_cmd";

	// Subscribe to the topic, and register a callback
	this->subPos = this->nodePos->Subscribe(topicNamePos,
	   &LidarPlugin::OnMsgPos, this);

	// Create the PID1 node
	this->nodePID = transport::NodePtr(new transport::Node());
	#if GAZEBO_MAJOR_VERSION < 8
	this->nodePID->Init(this->model->GetWorld()->GetName());
	#else
	this->nodePID->Init(this->model->GetWorld()->Name());
	#endif

	// Create a topic name
	std::string topicNamePID1 = "~/" + this->model->GetName() + "/pid1_cmd";
	std::string topicNamePID2 = "~/" + this->model->GetName() + "/pid2_cmd";
	

	// Subscribe to the topic, and register a callback
	this->subPID1 = this->nodePID->Subscribe(topicNamePID1,
	   &LidarPlugin::OnMsgPID1, this);  

	this->subPID2 = this->nodePID->Subscribe(topicNamePID2,
	   &LidarPlugin::OnMsgPID2, this); 

	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
	{
	  int argc = 0;
	  char **argv = NULL;
	  ros::init(argc, argv, "gazebo_client",
	      ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to
	// the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

	// Create a named topic, and subscribe to it.
	/*ros::SubscribeOptions so =
	  ros::SubscribeOptions::create<std_msgs::Float32>(
	      "/" + this->model->GetName() + "/pos_cmd",
	      1,
	      boost::bind(&LidarPlugin::OnRosMsg, this, _1),
	      ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);*/
	
	ros::SubscribeOptions so =
	  ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
	      "/" + this->model->GetName() + "/pos_cmd",
	      1,
	      boost::bind(&LidarPlugin::OnRosMsg, this, _1),
	      ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);

	// Spin up the queue helper thread.
	this->rosQueueThread =
	  std::thread(std::bind(&LidarPlugin::QueueThread, this));
    }
    
    /*public: void SetPosition(const double &_pos)
    {
     // Set the joint's target position.
      this->model->GetJointController()->SetPositionTarget(
          this->joint1->GetScopedName(), _pos);     
    }
    
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
        this->SetPosition(_msg->x());
    }
    
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
    	std::cerr << "Joint 1: ";
    	std::cerr << _msg->data;
       this->SetPosition(_msg->data);
    }*/
    
    public: void SetPosition(const int &_jn, const double &_pos)
    {
     //Chosing the joint
     this->joint1 = model->GetJoints()[_jn];
     // Set the joint's target position.
      this->model->GetJointController()->SetPositionTarget(
          this->joint1->GetScopedName(), _pos);  
          std::cerr << "Joint: " << _jn << " ";
          std::cerr << joint1->Position() << "\r\n";  
    }

	public: void SetPID1(const int &_P, const double &_I,const int &_D)
    {
     //Chosing the pid    
		this->pid1 = common::PID(_P, _I, _D);
		this->model->GetJointController()->SetPositionPID(
	      this->joint1->GetScopedName(), this->pid1);
		  std::cerr << "PID1 - "<< "P:" << _P << " I:" <<  _I << " D:" <<  _D << "\r\n";
    }

	public: void SetPID2(const int &_P, const double &_I,const int &_D)
    {
     //Chosing the pid
		this->pid2 = common::PID(_P, _I, _D);
	 	this->model->GetJointController()->SetPositionPID(
	      this->joint2->GetScopedName(), this->pid2);
		  std::cerr << "PID2 -" << "P:" << _P << " I:" <<  _I << " D:" <<  _D << "\r\n";
    }
    
    private: void OnMsgPos(ConstVector3dPtr &_msg)
    {
        this->SetPosition(_msg->x(), _msg->y());
    }

	private: void OnMsgPID1(ConstVector3dPtr &_msg)
    {
        this->SetPID1(_msg->x(), _msg->y(), _msg->z());
    }

	private: void OnMsgPID2(ConstVector3dPtr &_msg)
    {
        this->SetPID2(_msg->x(), _msg->y(), _msg->z());
    }

    
    public: void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
    {
    	std::cerr << "Joint 1: ";
    	std::cerr << _msg->data[0] << "  ";
    	std::cerr << "Joint 2: ";
    	std::cerr << _msg->data[1] << "\r\n";
       this->SetPosition(_msg->data[0],_msg->data[1]);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
       static const double timeout = 0.01;
       while (this->rosNode->ok())
       {
          this->rosQueue.callAvailable(ros::WallDuration(timeout));
       }
    }
    	
	/// \brief A node used for transport
	private: transport::NodePtr nodePos;
	private: transport::NodePtr nodePID;

	/// \brief A subscriber to a named topic.
	private: transport::SubscriberPtr subPos;
	private: transport::SubscriberPtr subPID1;
	private: transport::SubscriberPtr subPID2;

    	/// \brief Pointer to the model.
	private: physics::ModelPtr model;

	/// \brief Pointer to the joint.
	private: physics::JointPtr joint1;
	private: physics::JointPtr joint2;

	/// \brief A PID controller for the joint.
	private: common::PID pid1;
	private: common::PID pid2;
	private: common::PID GetErrors;
	
	/// \brief A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;

	/// \brief A ROS subscriber
	private: ros::Subscriber rosSub;

	/// \brief A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;

	/// \brief A thread the keeps running the rosQueue
	private: std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(LidarPlugin)
}
#endif
