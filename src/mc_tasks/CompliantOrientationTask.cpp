#include <mc_tasks/CompliantOrientationTask.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tvm/Robot.h>
#include <RBDyn/Jacobian.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{

CompliantOrientationTask::CompliantOrientationTask(const std::string & bodyName_,
                                                   const mc_rbdyn::Robots & robots,
                                                   unsigned int robotIndex,
                                                   double stiffness,
                                                   double weight)
: OrientationTask(robots.robot(robotIndex).frame(bodyName_), stiffness, weight), Gamma_(Eigen::Matrix3d::Zero()),
  robot_(robots.robot(robotIndex)), tvm_robot_(robots.robot(robotIndex).tvmRobot()), rIdx_(robotIndex),
  frame_(robots.robot(robotIndex).frame(bodyName_)), refAccel_(Eigen::Vector3d::Zero()),
  inputAccel_(Eigen::VectorXd::Zero(robots.robot(robotIndex).mb().nrDof())), frameAccel_(Eigen::Vector3d::Zero()),
  disturbance_(Eigen::Vector3d::Zero()), disturbedAccel_(Eigen::Vector3d::Zero())
{
  switch(backend_)
  {
    case Backend::Tasks:
    case Backend::TVM:
      break;
    default:
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_tasks] Can't use CompliantOrientationTask with {} backend, please use Tasks or TVM backend", backend_);
      break;
  }

  type_ = "compliant_position";
  name_ = std::string("compliant_position_") + frame_.robot().name() + "_" + frame_.name();
  OrientationTask::name(name_);
}

void CompliantOrientationTask::refAccel(const Eigen::Vector3d & refAccel) noexcept
{
  refAccel_ = refAccel;
}

void CompliantOrientationTask::update(mc_solver::QPSolver & solver)
{
  auto J = jac_->jacobian(robots.robot(rIndex).mb(), robots.robot(rIndex).mbc());
  if(backend_ == Backend::Tasks)
  {
    if(solver.robot().compensationTorquesAcc()) { inputAccel_ = solver.robot().compensationTorquesAcc().value(); }
    else
    {
      inputAccel_ = solver.robot().externalTorquesAcc();
    }
  }
  else
  {
    if(tvm_robot_.alphaDCompensation()) { inputAccel_ = tvm_robot_.alphaDCompensation().value(); }
    else
    {
      inputAccel_ = tvm_robot_.alphaDExternal();
    }
  }
  frameAccel_ = (J * inputAccel_).head(3);
  disturbance_ = Gamma_ * frameAccel_;
  disturbedAccel_ = refAccel_ + disturbance_;
  OrientationTask::refAccel(disturbedAccel_);
  OrientationTask::update(solver);
}

void CompliantOrientationTask::makeCompliant(bool compliance)
{
  if(compliance) { Gamma_.diagonal().setOnes(); }
  else
  {
    Gamma_.diagonal().setZero();
  }
}

void CompliantOrientationTask::setComplianceVector(Eigen::Vector3d Gamma)
{
  Gamma_.diagonal() = Gamma;
}

bool CompliantOrientationTask::isCompliant(void)
{
  return Gamma_.diagonal().norm() > 0;
}

Eigen::Vector3d CompliantOrientationTask::getComplianceVector(void)
{
  return Gamma_.diagonal();
}

void CompliantOrientationTask::addToSolver(mc_solver::QPSolver & solver)
{
  OrientationTask::addToSolver(solver);
  jac_ = new rbd::Jacobian(robots.robot(rIdx_).mb(), frame_.body());
}

void CompliantOrientationTask::addToLogger(mc_rtc::Logger & logger)
{
  OrientationTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_compliance", this, [this]() -> Eigen::Vector3d { return Gamma_.diagonal(); });
  logger.addLogEntry(name_ + "_complianceInputAccel", this,
                     [this]() -> const Eigen::VectorXd & { return inputAccel_; });
  logger.addLogEntry(name_ + "_complianceFrameAccel", this,
                     [this]() -> const Eigen::Vector3d & { return frameAccel_; });
  logger.addLogEntry(name_ + "_complianceDisturbance", this,
                     [this]() -> const Eigen::Vector3d & { return disturbance_; });
  logger.addLogEntry(name_ + "_complianceRefAccel", this,
                     [this]() -> const Eigen::Vector3d & { return disturbedAccel_; });
}

void CompliantOrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  OrientationTask::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_, "Compliance"},
      mc_rtc::gui::Checkbox(
          "Compliance is active", [this]() { return isCompliant(); }, [this]() { makeCompliant(!isCompliant()); }),
      mc_rtc::gui::ArrayInput("Gamma", {"x", "y", "z"}, Gamma_));
}

} // namespace mc_tasks
