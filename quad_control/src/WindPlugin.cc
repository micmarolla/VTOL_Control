#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "quad_control/wind.h"

using namespace std;

namespace gazebo{

    /*
     * This plugin just apply a constant force (wind) on the UAV model.
     * Currently, it applies a constant force of 1N along the x-direction of the world frame.
     */
    class WindPlugin : public WorldPlugin {

    public:
        WindPlugin() : WorldPlugin(){ }

        void Load(physics::WorldPtr world, sdf::ElementPtr){
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized()){
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                   << "Load the Gazebo system plugin 'liblink_attacher.so'");
                return;
            }

            _world = world;
            _enabled = _nh.param<bool>("wind/enabled", false);
            if(_enabled){
                // Retrieve force
                _fx = _nh.param<double>("wind/fx", 0.0);
                _fy = _nh.param<double>("wind/fy", 0.0);
                _fz = _nh.param<double>("wind/fz", 0.0);

                _updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&WindPlugin::OnUpdate, this));

                ROS_INFO("Wind Plugin initialized.");
            }
        }

        // Apply force
        void OnUpdate(){
            // Plugin not enabled
            if(!_enabled)
                return;

            // Normal state
            if(_uavBase)
                _uavBase->SetForce(ignition::math::Vector3d(_fx, _fy, _fz));

            // Waiting for UAV spawning
            else{
                // Retrieve model
                string modelName = _nh.param<string>("wind/model", "");
                physics::ModelPtr model = _world->ModelByName(modelName);
                if(!model)
                    return;

                // Retrieve link
                string linkName = _nh.param<string>("wind/link", "");
                _uavBase = model->GetLink(linkName);
            }
        }


    private:
        ros::NodeHandle _nh;
        physics::WorldPtr _world;
        physics::LinkPtr _uavBase;
        event::ConnectionPtr _updateConnection;
        double _fx, _fy, _fz;
        bool _enabled;


    };

    GZ_REGISTER_WORLD_PLUGIN(WindPlugin)
}
