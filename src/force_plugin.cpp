#include "ros/ros.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include "geometry_msgs/Wrench.h"

using namespace std;

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {

  	private: ros::NodeHandle* _node_handle;
    private: ros::Subscriber  _ft_sub;
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: physics::LinkPtr  _humm_base_link;
    private: geometry_msgs::Wrench _wrench;
    private: string _link_name;
    private: string _ft_topic_name;

    public: void FTs_cb( geometry_msgs::Wrench w ) {
      _wrench = w;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      
      _node_handle = new ros::NodeHandle();	
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      _wrench.force.x = 0.0;
      _wrench.force.y = 0.0;
      _wrench.force.z = 0.0;

      _wrench.torque.x = 0.0;
      _wrench.torque.y = 0.0;
      _wrench.torque.z = 0.0;



      _link_name = _sdf->Get<string>("link_name");
      _ft_topic_name = _sdf->Get<string>("topic_name");
      _ft_sub = _node_handle->subscribe(_ft_topic_name, 1, &ModelPush::FTs_cb, this );


    }

    // Called by the world update start event
    public: void OnUpdate()
    {   
      common::Time currTime = this->model->GetWorld()->SimTime();
      // Apply a small linear velocity to the model.
      if (currTime>=3) {
        _humm_base_link = model->GetLink( _link_name );

        cout << "Publishing on: " << _ft_topic_name << endl;
        cout << "Wrench: " << _wrench << endl;
        _humm_base_link->AddRelativeForce(ignition::math::Vector3d( _wrench.force.x, _wrench.force.y, _wrench.force.z ));
        _humm_base_link->AddRelativeTorque(ignition::math::Vector3d( _wrench.torque.x, _wrench.torque.y, _wrench.torque.z ));
      }	


    }


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
