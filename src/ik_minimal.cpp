#include <exotica_core/exotica_core.h>
#include <exotica_core/unconstrained_end_pose_problem_initializer.h>

// #define USE_ROS

int main(int argc, char **argv) {

#ifdef USE_ROS
  // If we want to visualise via RViz, we need to uncomment the following:
  ros::init(argc, argv, "exotica_standalone");
  exotica::Server::InitRos(std::make_shared<ros::NodeHandle>("~"));
#endif

  // Theoretically, we could load the solver & problem from XML in a single call
  // - but this will require the URDF and SRDF paths to either be ROS-pack
  // resolvable or absolute paths exotica::MotionSolverPtr solver =
  // exotica::XMLLoader::LoadSolver(std::string(EXOTICA_RESOURCE_DIR) +
  // "/resources/kinova_ik.xml");

  // Instead, we are going to load the XML as an initializer and then replace
  // the paths with full paths by prepending the resource directory.
  exotica::Initializer solver_initializer, problem_initializer;
  exotica::XMLLoader::Load(
      std::string(EXOTICA_RESOURCE_DIR) + "/resources/kinova_ik.xml",
      solver_initializer, problem_initializer, "", "", false);

  // Derive specific initializer from generic to make modifying elements easier
  exotica::UnconstrainedEndPoseProblemInitializer pp_init =
      exotica::UnconstrainedEndPoseProblemInitializer(problem_initializer);
  exotica::SceneInitializer scene_init =
      exotica::SceneInitializer(pp_init.PlanningScene);

  // Modify elements
  scene_init.URDF =
      std::string(EXOTICA_RESOURCE_DIR) + "/resources/" + scene_init.URDF;
  scene_init.SRDF =
      std::string(EXOTICA_RESOURCE_DIR) + "/resources/" + scene_init.SRDF;
  pp_init.PlanningScene = scene_init;

  auto problem = exotica::Setup::CreateProblem(exotica::Initializer(pp_init));
  auto solver = exotica::Setup::CreateSolver(solver_initializer);
  solver->SpecifyProblem(problem);

  // Create necessary variables and solve the problem
  Eigen::MatrixXd
      solution; // Matrix to store solutions. Each configuration is in a row,
                // i.e. for an IK problem the first row is the transposed
                // configuration vector. Will be resized in the solver when
                // calling Solve() if necessary

  Eigen::VectorXd q_current = Eigen::VectorXd::Zero(solver->GetProblem()->N);
  q_current << -1.47, 3.45, 5.28, 0.0, 0.0, 0.0;
  Eigen::VectorXd q_ik = Eigen::VectorXd::Zero(solver->GetProblem()->N);

  // target_pose can either be a 6d vector (xyz,rpy) or a 7d vector(xyz,xyzw)
  Eigen::VectorXd target_pose = Eigen::VectorXd::Zero(6);
  target_pose(0) = 0.3;
  target_pose(2) = 0.3;
  target_pose(5) = -1.57;

  for (int i = 0; i < 1000; ++i) {
    exotica::Timer timer;
    // Update problem (current state and target)
    solver->GetProblem()->SetStartState(q_current);
    solver->GetProblem()->GetScene()->AttachObjectLocal("Target", "",
                                                        target_pose);
    solver->Solve(solution);
    q_ik = solution.row(0).transpose(); // next joint angle
    double time_taken = timer.GetDuration();

    // Get error of end-effector
    solver->GetProblem()->GetScene()->Update(q_ik);
    auto ee = solver->GetProblem()->GetScene()->GetKinematicTree().FK(
        "end_effector", KDL::Frame::Identity(), "", KDL::Frame::Identity());

    // Check maximum change
    auto diff = (q_current - q_ik).cwiseAbs().maxCoeff();

    // Scale to maximum step size

    HIGHLIGHT("Finished solving in "
              << time_taken << "s. Solution [" << solution << "] - EE: ["
              << Eigen::Map<Eigen::Vector3d>(ee.p.data).transpose()
              << "] - max step: " << diff);
    q_current = q_ik; // Pretend the simulator made a full step ...

#ifdef USE_ROS
    // Publish to RViz
    problem->GetScene()->GetKinematicTree().PublishFrames();
    ros::Rate(10).sleep();
#endif
  }

  // Clean-up Exotica: Since we manually created the objects from initializers,
  // we need to reset them.
  problem.reset();
  solver.reset();
  exotica::Setup::Destroy();

  return EXIT_SUCCESS;
}
