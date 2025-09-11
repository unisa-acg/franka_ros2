#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>

#include "franka_hardware/panda_joint_command_governor.hpp"

namespace acg_optimal_control
{

PandaJointCommandGovernor::PandaJointCommandGovernor(const std::string &data_folder_name, const Eigen::Vector<double, N_JOINTS_>& initial_joint_positions)
{
    if(initial_joint_positions.size() != N_JOINTS_){
        throw std::runtime_error("Initial joint positions size mismatch.");
    }

    // Set system matrices
    A_ << 1, T_delta_, T_delta_ * T_delta_,
        0, 1, T_delta_,
        0, 0, 1;
    B_ << T_delta_ * T_delta_ * T_delta_,
        T_delta_ * T_delta_,
        T_delta_;

    // Load constraint matrices from binary files
    bool success = load_constraint_matrices_binary_(data_folder_name);
    if (!success) {
        throw std::runtime_error("Failed to load matrices from " + data_folder_name);
    }

    // Transformation matrix
    Tx_.setZero();
    Tx_.diagonal() << 1.0, T_delta_, T_delta_ * T_delta_;

    // Apply transformation to system and constraint matrices
    A_scaled_ = Tx_ * A_ * Tx_.inverse();
    B_scaled_ = Tx_ * B_ * (1 / Tu_);
    for (int i = 0; i < N_JOINTS_; ++i) {
        Ax_vec_[i] = Ax_vec_[i] * Tx_.inverse();
        u_min_vec_[i] = u_min_vec_[i] * Tu_;
        u_max_vec_[i] = u_max_vec_[i] * Tu_;
    }

    Eigen::Matrix<double, 3, N_JOINTS_> initial_states;
    initial_states.setZero();
    initial_states.row(0) = initial_joint_positions.transpose();
    initial_states = Tx_ * initial_states;

    // Weighting matrix
    W_.setZero();
    W_.diagonal() << 1.0, 1e2, 5e-1;

    // Initialize Optimal Controllers for each joint
    std::vector<std::unique_ptr<acg_optimal_control::OptimalController>> controllers;
    for (int i = 0; i < N_JOINTS_; ++i) {
        auto cg = std::make_unique<acg_optimal_control::OptimalController>();
        QPVector x0 = initial_states.col(i);
        cg->init(A_scaled_, B_scaled_, W_, Ax_vec_[i], bx_vec_[i], u_min_vec_[i], u_max_vec_[i], &x0);
        controllers.push_back(std::move(cg));
    }

    // Initialize Parallel Executor
    parallel_controllers_executor_ = std::make_unique<acg_optimal_control::ParallelOptimalControllers>(std::move(controllers));
}

PandaJointCommandGovernor::~PandaJointCommandGovernor()
{
    parallel_controllers_executor_->stop_all();
}

void PandaJointCommandGovernor::update(std::array<double, N_JOINTS_>& command,
    const std::array<double, N_JOINTS_>& current_joint_positions,
    const std::array<double, N_JOINTS_>& current_joint_velocities,
    const std::array<double, N_JOINTS_>& current_joint_accelerations,
    const std::array<double, N_JOINTS_>& desired_joint_positions,
    const std::array<double, N_JOINTS_>& desired_joint_velocities)
{
    Eigen::Matrix<double, 3, N_JOINTS_> x_k, x_d;

    for(int i=0; i<N_JOINTS_; i++){
        x_k(0,i) = current_joint_positions[i];
        x_k(1,i) = current_joint_velocities[i];
        x_k(2,i) = current_joint_accelerations[i];
        x_d(0,i) = desired_joint_positions[i];
        x_d(1,i) = desired_joint_velocities[i];
        x_d(2,i) = 0.0;
    }

    Eigen::Matrix<double, 1, N_JOINTS_> u_k;
    update(u_k, x_k, x_d);
    std::copy(u_k.data(), u_k.data() + N_JOINTS_, command.begin());
}

void PandaJointCommandGovernor::update(Eigen::Ref<Eigen::Vector<double, N_JOINTS_>> command,
    Eigen::Ref<const Eigen::Matrix<double, 3, N_JOINTS_>> x_k,
    Eigen::Ref<const Eigen::Matrix<double, 3, N_JOINTS_>> x_d)
{    
    // Transform current and desired states
    Eigen::Matrix<double, 3, N_JOINTS_> z_k, z_d;
    z_k = Tx_ * x_k;
    z_d = Tx_ * x_d;

    Eigen::Matrix<double, 1, N_JOINTS_> u_k;
    parallel_controllers_executor_->run(u_k, z_k, z_d);

    // Transform back the input command
    u_k = u_k / Tu_;

    // Invert dynamics to get the actual command
    Eigen::Matrix<double, 3, N_JOINTS_> x_k_new;
    step_system(x_k_new, x_k, u_k);

    // Extract the command (positions only)
    command = x_k_new.row(0).transpose();
}

bool PandaJointCommandGovernor::load_constraint_matrices_binary_(const std::string &dir_name){
    // Check if path exists
    if (!std::filesystem::exists(dir_name)) {
        std::cerr << "Path does not exist: " << dir_name << std::endl;
        return false;
    }
    // Load matrices for each joint
    Ax_vec_.resize(N_JOINTS_);
    bx_vec_.resize(N_JOINTS_);
    u_min_vec_.resize(N_JOINTS_);
    u_max_vec_.resize(N_JOINTS_);

    bool success = true;
    for(int i=1; i<=N_JOINTS_ && success; i++){
        success &= load_matrix_binary_(Ax_vec_[i-1], dir_name + "/joint_" + std::to_string(i) + "_Ax_cdd.bin");
        success &= load_matrix_binary_(bx_vec_[i-1], dir_name + "/joint_" + std::to_string(i) + "_bx_cdd.bin");
        success &= load_matrix_binary_(u_min_vec_[i-1], dir_name + "/joint_" + std::to_string(i) + "_u_min.bin");
        success &= load_matrix_binary_(u_max_vec_[i-1], dir_name + "/joint_" + std::to_string(i) + "_u_max.bin");
    }
    return success;
}

void PandaJointCommandGovernor::step_system(Eigen::Ref<Eigen::Matrix<double, 3, N_JOINTS_>> new_state, Eigen::Ref<const Eigen::Matrix<double, 3, N_JOINTS_>> state, Eigen::Ref<const Eigen::Matrix<double, 1, N_JOINTS_>> u)
{   
    new_state = A_ * state + B_ * u;
}

} // namespace acg_optimal_control