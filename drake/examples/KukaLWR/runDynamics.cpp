#include "LCMSystem.h"
#include "RigidBodySystem.h"
#include "LinearSystem.h"
#include "BotVisualizer.h"
#include "drakeAppUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char *argv[]) {
    auto rigid_body_sys = make_shared<RigidBodySystem>(getDrakePath() +
                                                       "/examples/KukaLWR/lwr_defs/robots/lwr_drake.urdf",
                                                       DrakeJoint::FIXED); // TODO: copy URDF instead of symlinking oh-distro lwr_defs

    auto const &tree = rigid_body_sys->getRigidBodyTree();

    shared_ptr <lcm::LCM> lcm = make_shared<lcm::LCM>();

    MatrixXd Kp(getNumInputs(*rigid_body_sys), tree->num_positions);
    MatrixXd Kd(getNumInputs(*rigid_body_sys), tree->num_velocities);
    Kp.setZero();
    Kd.setZero();

    { // Simple PD Controller
        Kp.setZero();
        Kd.setZero();

        for (int actuator_idx = 0; actuator_idx < tree->actuators.size(); actuator_idx++) {
//            std::cout << tree->actuators[actuator_idx].name << std::endl;
            auto const &b = tree->actuators[actuator_idx].body;
            Kp(actuator_idx, b->position_num_start) = 50;
            Kd(actuator_idx, b->velocity_num_start) = 1;
        }
    }
    auto lwr_with_pd = make_shared < PDControlSystem < RigidBodySystem >> (rigid_body_sys, Kp, Kd);

    auto visualizer = make_shared < BotVisualizer < RigidBodySystem::StateVector >> (lcm, tree);
    auto sys = cascade(lwr_with_pd, visualizer);

    SimulationOptions options = default_simulation_options;
    rigid_body_sys->penetration_stiffness = 5000.0;
    rigid_body_sys->penetration_damping = rigid_body_sys->penetration_stiffness / 10.0;
    options.initial_step_size = 5e-3;
    options.timeout_seconds = numeric_limits<double>::infinity();

    VectorXd x0(rigid_body_sys->getNumStates());
    x0.head(tree->num_positions) = tree->getZeroConfiguration();
    for (int i = 7; i < 14; i++)
        x0(i) = 1.1; // Example Test: give it some initial velocity

    runLCM(sys, lcm, 0, std::numeric_limits<double>::max(), x0, options);

    return 0;
}