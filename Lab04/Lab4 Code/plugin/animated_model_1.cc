#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>


// The code makes a Gazebo model follow a pre-defined trajectory
namespace gazebo
{
  class ModelTraj : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test", set it in a loop
              // make it last 16 seconds for each loop
              new gazebo::common::PoseAnimation("test", 16.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box (time = 0)
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(-2, 0, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set waypoint location (time = 4.0)
        key = anim->CreateKeyFrame(4.0);
        key->Translation(ignition::math::Vector3d(0, -1, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));


        key = anim->CreateKeyFrame(8.0);
        key->Translation(ignition::math::Vector3d(1, 1, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));


        key = anim->CreateKeyFrame(12.0);
        key->Translation(ignition::math::Vector3d(-1, 2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set final location equal to starting location
        key = anim->CreateKeyFrame(16.0);
        key->Translation(ignition::math::Vector3d(-2, 0, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelTraj)
}
