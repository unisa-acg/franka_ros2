#pragma once

#include <fstream>

#include <acg_optimal_control/parallel_optimal_controllers.hpp>
#include <Eigen/Dense>

namespace acg_optimal_control
{

class PandaJointCommandGovernor
{
static constexpr double T_delta_ = 0.001;
static constexpr int N_JOINTS_ = 7;

public:
    PandaJointCommandGovernor(const std::string& data_folder_name, const Eigen::Vector<double, N_JOINTS_>& initial_joint_positions);
    ~PandaJointCommandGovernor();

    void update(std::array<double, N_JOINTS_>& command,
        const std::array<double, N_JOINTS_>& current_joint_positions,
        const std::array<double, N_JOINTS_>& current_joint_velocities,
        const std::array<double, N_JOINTS_>& current_joint_accelerations,
        const std::array<double, N_JOINTS_>& desired_joint_positions,
        const std::array<double, N_JOINTS_>& desired_joint_velocities);
    
    // This function does modify x_k and x_d
    void update(Eigen::Ref<Eigen::Vector<double, N_JOINTS_>> command,
        Eigen::Ref<const Eigen::Matrix<double, 3, N_JOINTS_>> x_k,
        Eigen::Ref<const Eigen::Matrix<double, 3, N_JOINTS_>> x_d);
private:
    bool load_constraint_matrices_binary_(const std::string &filename);

    void step_system(Eigen::Ref<Eigen::Matrix<double, 3, N_JOINTS_>> new_state, Eigen::Ref<const Eigen::Matrix<double, 3, N_JOINTS_>> state, Eigen::Ref<const Eigen::Matrix<double, 1, N_JOINTS_>> u);

    // Robot specific parameters
    Eigen::Matrix<double, 3, 3> A_;
    Eigen::Matrix<double, 3, 1> B_;
    Eigen::Matrix<double, 3, 3> A_scaled_;
    Eigen::Matrix<double, 3, 1> B_scaled_;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> Ax_vec_;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> bx_vec_;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> u_min_vec_;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> u_max_vec_;

    // Transformation matrices
    const double Tu_ = 1e-9;
    Eigen::Matrix<double, 3, 3> Tx_ = Eigen::Matrix<double, 3, 3>::Identity();

    // Parallel Executor
    std::unique_ptr<acg_optimal_control::ParallelOptimalControllers> parallel_controllers_executor_;

    // QP solver parameters
    Eigen::Matrix<double, 3, 3> W_; // Weighting matrix

};


template<typename Derived>
bool load_matrix_binary_(Eigen::PlainObjectBase<Derived> &out, const std::string &filename)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in)
        return false;

    int64_t rows, cols;
    in.read(reinterpret_cast<char *>(&rows), sizeof(int64_t));
    in.read(reinterpret_cast<char *>(&cols), sizeof(int64_t));

    std::cout << "Loading matrix from " << filename << " with shape (" << rows << ", " << cols << ")" << std::endl;

    std::vector<double, Eigen::aligned_allocator<double>> buffer(rows * cols);
    in.read(reinterpret_cast<char *>(buffer.data()), rows * cols * sizeof(double));

    // Copy data into Eigen matrix
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eigen_buffer(buffer.data(), rows, cols);
    out.resize(rows, cols);
    out = eigen_buffer;
    return true;
}

} // namespace acg_optimal_control