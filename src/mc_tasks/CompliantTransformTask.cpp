#include <mc_tasks/CompliantTransformTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tvm/Robot.h>

namespace mc_tasks
{

CompliantTransformTask::CompliantTransformTask(const mc_rbdyn::RobotFrame & frame,
                                               double stiffness,
                                               double weight,
                                               bool showTarget,
                                               bool showPose)
: TransformTask(frame, stiffness, weight, showTarget, showPose),
  Gamma_(Eigen::Matrix6d::Zero()),
  robot_(frame.robot()),
  tvm_robot_(frame.robot().tvmRobot()),
  rIdx_(frame.robot().robotIndex()),
  bodyName_(frame.body()),
  frame_(frame),
  refAccel_(Eigen::Vector6d::Zero()),
  inputAccel_(Eigen::VectorXd::Zero(frame.robot().mb().nrDof())),
  frameAccel_(Eigen::Vector6d::Zero()),
  disturbance_(Eigen::Vector6d::Zero()),
  disturbedAccel_(Eigen::Vector6d::Zero())
{
  switch(backend_)
  {
    case Backend::Tasks:
    case Backend::TVM:
      break;
    default:
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_tasks] Can't use CompliantTransformTask with {} backend, please use Tasks or TVM backend", backend_);
      break;
  }

  type_ = "compliant_transform";
  name_ = "compliant_transform_" + frame_.robot().name() + "_" + frame_.name();
}

CompliantTransformTask::CompliantTransformTask(const std::string & surfaceName,
                                               const mc_rbdyn::Robots & robots,
                                               unsigned int robotIndex,
                                               double stiffness,
                                               double weight,
                                               bool showTarget,
                                               bool showPose)
: CompliantTransformTask(robots.robot(robotIndex).frame(surfaceName), stiffness, weight, showTarget, showPose)
{
}

CompliantTransformTask::~CompliantTransformTask()
{
  delete jac_;
}

void CompliantTransformTask::refAccel(const Eigen::Vector6d & refAccel) noexcept
{
  refAccel_ = refAccel;
}

void CompliantTransformTask::makeCompliant(bool compliance)
{
  if(compliance)
  {
    Gamma_.diagonal().setOnes();
  }
  else
  {
    Gamma_.diagonal().setZero();
  }
}

void CompliantTransformTask::setComplianceVector(Eigen::Vector6d gamma)
{
  Gamma_.diagonal() = gamma;
}

bool CompliantTransformTask::isCompliant(void)
{
  return Gamma_.diagonal().norm() > 0.0;
}

Eigen::Vector6d CompliantTransformTask::getComplianceVector(void)
{
  return Gamma_.diagonal();
}

void CompliantTransformTask::addToSolver(mc_solver::QPSolver & solver)
{
  TransformTask::addToSolver(solver);
  delete jac_;
  jac_ = new rbd::Jacobian(robot_.mb(), frame_.body());
}

void CompliantTransformTask::update(mc_solver::QPSolver & solver)
{
  if(!jac_)
  {
    jac_ = new rbd::Jacobian(robot_.mb(), frame_.body());
  }

  Eigen::MatrixXd J = jac_->jacobian(solver.robot(rIdx_).mb(), solver.robot(rIdx_).mbc());

  if(backend_ == Backend::Tasks)
  {
    if(robot_.compensationTorquesAcc())
    {
      inputAccel_ = robot_.compensationTorquesAcc().value();
    }
    else
    {
      inputAccel_ = robot_.externalTorquesAcc();
    }
  }
  else
  {
    if(tvm_robot_.alphaDCompensation())
    {
      inputAccel_ = tvm_robot_.alphaDCompensation().value();
    }
    else
    {
      inputAccel_ = tvm_robot_.alphaDExternal();
    }
  }

  // J * inputAccel_ produces spatial acceleration: [angular; linear]
  frameAccel_ = J * inputAccel_;
  disturbance_ = Gamma_ * frameAccel_;
  disturbedAccel_ = refAccel_ + disturbance_;

  TransformTask::refAccel(disturbedAccel_);
  TransformTask::update(solver);
}

void CompliantTransformTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  TransformTask::load(solver, config);

  if(config.has("compliance"))
  {
    auto g = config("compliance");
    if(g.size())
    {
      Eigen::Vector6d v = g;
      setComplianceVector(v);
    }
    else
    {
      makeCompliant(static_cast<double>(g) != 0.0);
    }
  }
}

void CompliantTransformTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TransformTask::addToGUI(gui);

  gui.addElement(
      {"Tasks", name_, "Compliance"},
      mc_rtc::gui::Checkbox(
          "Compliance is active", [this]() { return isCompliant(); }, [this]() { makeCompliant(!isCompliant()); }),
      mc_rtc::gui::ArrayInput(
          "Compliance parameters", {"rx", "ry", "rz", "x", "y", "z"}, [this]() { return getComplianceVector(); },
          [this](Eigen::Vector6d v) { setComplianceVector(v); }));
}

void CompliantTransformTask::addToLogger(mc_rtc::Logger & logger)
{
  TransformTask::addToLogger(logger);

  logger.addLogEntry(name_ + "_compliance", this, [this]() -> Eigen::Vector6d { return Gamma_.diagonal(); });
  logger.addLogEntry(name_ + "_complianceInputAccel", this,
                     [this]() -> const Eigen::VectorXd & { return inputAccel_; });
  logger.addLogEntry(name_ + "_complianceFrameAccel", this,
                     [this]() -> const Eigen::Vector6d & { return frameAccel_; });
  logger.addLogEntry(name_ + "_complianceDisturbance", this,
                     [this]() -> const Eigen::Vector6d & { return disturbance_; });
  logger.addLogEntry(name_ + "_complianceRefAccel", this,
                     [this]() -> const Eigen::Vector6d & { return disturbedAccel_; });
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "compliant_transform",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "compliant_transform");
      const auto & robot = solver.robots().robot(robotIndex);
      const auto & frame = [&]() -> const mc_rbdyn::RobotFrame &
      {
        if(config.has("surface"))
        {
          return robot.frame(config("surface"));
        }
        return robot.frame(config("frame"));
      }();

      auto t = std::make_shared<mc_tasks::CompliantTransformTask>(frame);
      t->load(solver, config);
      return t;
    });

} // namespace