#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "quad_control/wind.h"

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

            this->_world = world;
            this->_server = this->_nh.advertiseService("/quad_control/wind", &WindPlugin::callback, this);

            _updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&WindPlugin::OnUpdate, this));

            ROS_INFO("Wind Plugin initialized.");
        }

        // Apply force
        void OnUpdate(){
            if(_uavBase)
                _uavBase->SetForce(ignition::math::Vector3d(_fx, _fy, _fz));
        }

        // Retrieve the base link reference and the wanted wind
        bool callback(quad_control::wind::Request &req, quad_control::wind::Response &res){
            _uavBase = _world->ModelByName("hummingbird")->GetLink("hummingbird/base_link");
            _fx = req.fx;
            _fy = req.fy;
            _fz = req.fz;
            res.ok = true;
            ROS_INFO("Wind: link attached");
            return true;
        }


    private:
        ros::NodeHandle _nh;
        ros::ServiceServer _server;

        physics::WorldPtr _world;
        physics::LinkPtr _uavBase;
        event::ConnectionPtr _updateConnection;

        double _fx, _fy, _fz;

    };

    GZ_REGISTER_WORLD_PLUGIN(WindPlugin)
}
