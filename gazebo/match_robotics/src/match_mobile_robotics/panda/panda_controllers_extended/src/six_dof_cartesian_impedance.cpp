#include<panda_controllers_extended/six_dof_cartesian_impedance.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace panda_controllers_extended {

bool SixDofCartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) 
{

    //#################################################################
    //Get ros parameters
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) 
    {
        ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
        return false;
    }
  
    double rate;
    if (!node_handle.getParam("rate", rate)) 
    {
        ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter rate");
        return false;
    }

    std::vector<std::string> joint_names_impedance;
    if (!node_handle.getParam("joint_names_impedance", joint_names_impedance) || joint_names_impedance.size() != 6) 
    {
        ROS_ERROR(
            "CartesianImpedanceController: Invalid or no joint_names_impedance parameters provided, "
            "aborting controller init!");
        return false;
    }

    std::string joint_name_free;
    if (!node_handle.getParam("joint_name_free", joint_name_free))
    {
        ROS_ERROR(
            "CartesianImpedanceController: Invalid or no joint_name_free parameters provided, "
            "aborting controller init!");
        return false;
    }


    //#################################################################
    //Alloc Interface handles
    //ModelInterface
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) 
    {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Error getting model interface from hardware");
        return false;
    }
    try 
    {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } 
    catch (hardware_interface::HardwareInterfaceException& ex) 
    {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Exception getting model handle from interface: "
            << ex.what());
        return false;
    }
    //StateInterface
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) 
    {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Error getting state interface from hardware");
        return false;
    }
    try 
    {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } 
    catch (hardware_interface::HardwareInterfaceException& ex) 
    {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }

    //#################################################################
    //EffortJointInterfaces
    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) 
    {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < joint_names_impedance.size(); ++i) 
    {
        try 
        {
            joint_handles_impedance_.push_back(effort_joint_interface->getHandle(joint_names_impedance[i]));
        } 
        catch (const hardware_interface::HardwareInterfaceException& ex) 
        {
            ROS_ERROR_STREAM(
                "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }
    try 
    {
        joint_handle_free_=effort_joint_interface->getHandle(joint_name_free);
    } 
    catch (const hardware_interface::HardwareInterfaceException& ex) 
    {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
        return false;
    }



    this->calculation_scope_timer_=node_handle.createTimer( ros::Rate(rate),
                                                &SixDofCartesianImpedanceController::calculationScope,this);


    dyn_compliance_server_ = std::make_unique<dynamic_reconfigure::Server
                                                <panda_controllers_extended::StiffnessConfig>>
                                            (node_handle);
    dyn_compliance_server_->setCallback(
        boost::bind(&SixDofCartesianImpedanceController::complianceParamCallback, this, _1, _2));

    this->equi_sub_=node_handle.subscribe("target_pose",1,&SixDofCartesianImpedanceController::equilibriumPoseCallback,this);
    
    return true;

}

void SixDofCartesianImpedanceController::starting(const ros::Time&)
{
    std::array<double,16> pose=this->model_handle_->getPose(franka::Frame::kJoint6);
    ee_start_= Eigen::Affine3d(Eigen::Matrix4d::Map(pose.data()));
    ee_target_=ee_start_;
    return;
}

void SixDofCartesianImpedanceController::update(const ros::Time&, const ros::Duration& period)
{
    for(int i=0;i<this->joint_handles_impedance_.size();i++)
    {
        this->joint_handles_impedance_[i].setCommand(this->tau_d_(i));
    }
    this->joint_handle_free_.setCommand(0.0);
}

void SixDofCartesianImpedanceController::calculationScope(const ros::TimerEvent &)
{
    //Get current error of pose
    std::array<double,16> pose=this->model_handle_->getPose(franka::Frame::kJoint6);
    Eigen::Affine3d ee_current(Eigen::Matrix4d::Map(pose.data()));
    Eigen::Matrix<double, 6, 1> error=computeError(ee_current);

    //Get current error of velocity
    std::array<double,7> joint_vel_array=this->state_handle_->getRobotState().dq;
    Eigen::Map<Eigen::Matrix<double, 7, 1>> joint_vel_seven_dof(joint_vel_array.data());
    Eigen::Matrix<double,6, 1> joint_vel(joint_vel_seven_dof.block<6,1>(0,0));
    

    //Get coriolis vector
    std::array<double,7> coriolis=this->model_handle_->getCoriolis();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_coriolis_seve_dof(coriolis.data());
    Eigen::Matrix<double, 6, 1> tau_coriolis(tau_coriolis_seve_dof.block<6,1>(0,0));

    //Get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kJoint6);  
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_seven_dof(jacobian_array.data());
    Eigen::Matrix<double,6,6> jacobian=jacobian_seven_dof.block<6,6>(0,0);

    Eigen::Matrix<double,6,1> tau_task;
    tau_task<<jacobian.transpose()*(this->cartesian_damping_*(-1*jacobian*joint_vel)+
                                    this->cartesian_stiffness_*error);
    this->tau_d_=tau_task+tau_coriolis;
    this->tau_d_=saturateTorqueRate(tau_d_);
        

}

 void SixDofCartesianImpedanceController::complianceParamCallback(panda_controllers_extended::StiffnessConfig& config,
                               uint32_t level)
{
//Alloc cartesian stiffnes
  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner(3, 3)
      << config.Kx,0.0,0.0,0.0,config.Ky,0.0,0.0,0.0,config.Kz;
  cartesian_stiffness_.bottomRightCorner(3, 3)
      << config.Kroll,0.0,0.0,0.0,config.Kpitch,0.0,0.0,0.0,config.Kyaw;
  
  //Alloc cartesian damping
  cartesian_damping_.setIdentity();
  cartesian_damping_.topLeftCorner(3, 3)
       << config.Dx,0.0,0.0,0.0,config.Dy,0.0,0.0,0.0,config.Dz;
  cartesian_damping_.bottomRightCorner(3, 3)
      << config.Droll,0.0,0.0,0.0,config.Dpitch,0.0,0.0,0.0,config.Dyaw;

}





void SixDofCartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) 
{
    //Set desired position
    this->ee_target_.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    
    //Set desired orientation
    Eigen::Quaterniond quat,last_quat;
    quat.coeffs()<< msg->pose.orientation.x, 
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w;    
    last_quat=this->last_ee_target_.linear();
    //Flip orientation if nescessary                                  
    if (last_quat.coeffs().dot(quat.coeffs()) < 0.0) {
        quat.coeffs() << -quat.coeffs();
    }
    this->ee_target_.linear()<<quat.matrix();
}


//#################################################
//HELPER

Eigen::Matrix<double, 6, 1> SixDofCartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 6, 1>& tau_d_calculated)
{  
    const double delta_tau_max_{1.0};
    std::array<double,7> tau_j_d=this->state_handle_->getRobotState().tau_J_d;
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(tau_j_d.data());
    
    Eigen::Matrix<double, 6, 1> tau_d_saturated{};
    for (size_t i = 0; i < tau_d_calculated.size(); i++) {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] =
            tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
  return tau_d_saturated;
}

Eigen::Matrix<double, 6, 1> SixDofCartesianImpedanceController::computeError(Eigen::Affine3d current)
{
    Eigen::Matrix<double,3,1> position;
    Eigen::Matrix<double,3,1> position_d;

    Eigen::Quaterniond orientation;
    Eigen::Quaterniond orientation_d;

    position_d=this->ee_target_.translation();
    position=current.translation();

    orientation_d=this->ee_target_.linear();
    orientation=current.linear();

    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position_d-position;

    // orientation error
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation_d.inverse() * orientation);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -ee_target_.linear() * error.tail(3);

    return error;
}


}//end namespace

PLUGINLIB_EXPORT_CLASS(panda_controllers_extended::SixDofCartesianImpedanceController,
                       controller_interface::ControllerBase)
