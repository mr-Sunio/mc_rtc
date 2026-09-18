#pragma once

#include <mc_tasks/TransformTask.h>
#include <RBDyn/Jacobian.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI CompliantTransformTask : public TransformTask
{
public:
  /*! \brief Constructor
   *
   * \param frame Frame controlled by this task
   * \param stiffness Task stiffness
   * \param weight Task weight
   * \param showTarget Toggle target visualization in GUI
   * \param showPose Toggle current pose visualization in GUI
   */
  CompliantTransformTask(const mc_rbdyn::RobotFrame & frame,
                         double stiffness = 2.0,
                         double weight = 500.0,
                         bool showTarget = true,
                         bool showPose = true);

  /*! \brief Constructor (Surface name based)
   *
   * \param surfaceName Name of the surface frame to control
   * \param robots Robots controlled by this task
   * \param robotIndex Index of the robot controlled by this task
   * \param stiffness Task stiffness
   * \param weight Task weight
   * \param showTarget Toggle target visualization in GUI
   * \param showPose Toggle current pose visualization in GUI
   */
  CompliantTransformTask(const std::string & surfaceName,
                         const mc_rbdyn::Robots & robots,
                         unsigned int robotIndex,
                         double stiffness = 2.0,
                         double weight = 500.0,
                         bool showTarget = true,
                         bool showPose = true);

  ~CompliantTransformTask() override;

  using TransformTask::refAccel;

  /** Change reference acceleration
   *
   * \p refAccel Spatial acceleration vector (angular, linear) of size 6
   */
  void refAccel(const Eigen::Vector6d & refAccel) noexcept;

  // Set the compliant behavior of the task
  void makeCompliant(bool compliance);
  void setComplianceVector(Eigen::Vector6d gamma);

  // Get compliance state of the task
  bool isCompliant(void);
  Eigen::Vector6d getComplianceVector(void);

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  void addToLogger(mc_rtc::Logger & logger) override;

  void addToSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver & solver) override;

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

  Eigen::Matrix6d Gamma_;

  const mc_rbdyn::Robot & robot_;
  mc_tvm::Robot & tvm_robot_;

  unsigned int rIdx_;

  std::string bodyName_;
  const mc_rbdyn::RobotFrame & frame_;

  rbd::Jacobian * jac_ = nullptr;

  Eigen::Vector6d refAccel_;
  Eigen::VectorXd inputAccel_;
  Eigen::Vector6d frameAccel_;
  Eigen::Vector6d disturbance_;
  Eigen::Vector6d disturbedAccel_;
};

} // namespace mc_tasks