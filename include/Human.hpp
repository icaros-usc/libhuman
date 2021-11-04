#ifndef LIBHUMAN_HUMAN_HPP_
#define LIBHUMAN_HUMAN_HPP_

#include <future>
#include <memory>
#include <Eigen/Core>
#include <aikido/common/RNG.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/robot/util.hpp>
#include <aikido/control/ros/RosJointStateClient.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/optional.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/dart.hpp>
#include <aikido/planner/kunzretimer/KunzRetimer.hpp>
#include <aikido/robot/ros/RosRobot.hpp>

namespace human {

/// URI to retrieve Human URDF from.
    extern dart::common::Uri defaultPRLUrdfUri;
/// URI to retrieve Human SRDF from.
    extern dart::common::Uri defaultPRLSrdfUri;
/// URI to retrieve visual Human URDF from.
    extern dart::common::Uri defaultVisPRLUrdfUri;
/// URI to retrieve visual Human SRDF from.
    extern dart::common::Uri defaultVisPRLSrdfUri;
/// URI to retrieve Human_short URDF from
    extern dart::common::Uri defaultICAROSUrdfUri;
/// URI to retrieve Human_short SRDF from.
    extern dart::common::Uri defaultICAROSSrdfUri;
/// URI to retrieve visual human_short URDF from
    extern dart::common::Uri defaultVisICAROSUrdfUri;
/// URI to retrieve visual human_short SRDF from
    extern dart::common::Uri defaultVisICAROSSrdfUri;

/// URI to retrieve named arm configurations from.
    extern const dart::common::Uri namedConfigurationsUri;

/// Human-specific defaults for the KunzRetimer.
// Default kunz parameters
    constexpr static double DEFAULT_KUNZ_DEVIATION = 1e-3;
    constexpr static double DEFAULT_KUNZ_STEP = 1e-3;

    struct KunzParams : aikido::planner::kunzretimer::KunzRetimer::Params {
        KunzParams(
                double _maxDeviation = DEFAULT_KUNZ_DEVIATION,
                double _timeStep = DEFAULT_KUNZ_STEP)
                : aikido::planner::kunzretimer::KunzRetimer::Params(
                _maxDeviation, _timeStep) {
            // Do nothing.
        }
    };

    class Human final : public aikido::robot::ros::RosRobot {
    public:
        /// Inner HumanHnad class that implements Aikido's Hand Interface
        class HumanHand;

        // Default Parameters
#define DEFAULT_THREAD_CYCLE std::chrono::milliseconds(10)
#define DEFAULT_ROS_TRAJ_INTERP_TIME 0.1
#define DEFAULT_ROS_TRAJ_GOAL_TIME_TOL 5.0
#define DEFAULT_CONF_OBJ_NS "humanConf"
#define DEFAULT_ARM_TRAJ_CTRL "trajectory_controller"
#define DEFAULT_HAND_TRAJ_CTRL "hand_controller"
#define DEFAULT_EE_NAME "end_effector"

#define DEFAULT_HUMAN_URDF "package://libhuman/robot/human_short.urdf"
#define DEFAULT_HUMAN_SRDF "package://libhuman/robot/human_short.srdf"

        /// Construct the Human model.
        ///
        /// \param[in] env World (either for planning, post-processing, or executing)
        /// \param[in] rngSeed seed for initializing random generator
        ///        May be nullptr if simulation is true
        /// \param[in] humanUrdfUri Path to Human URDF model.
        /// \param[in] shortUrdfUri Path to Human short URDF model.
        /// \param[in] retriever Resource retriever for retrieving human.
        Human(
                bool simulation,
                aikido::planner::WorldPtr env,
                const dart::common::Uri &humanUrdfUri = dart::common::Uri(""),
                const dart::common::Uri &humanSrdfUri = dart::common::Uri(""),
                const std::string confNamespace = DEFAULT_CONF_OBJ_NS,
                const std::chrono::milliseconds threadCycle = DEFAULT_THREAD_CYCLE,

                const ::ros::NodeHandle *node = nullptr,
                aikido::common::RNG::result_type rngSeed = std::random_device{}(),
                const dart::common::ResourceRetrieverPtr &retriever
                = std::make_shared<aikido::io::CatkinResourceRetriever>());

        virtual ~Human();

        void step(const std::chrono::system_clock::time_point &timepoint) override;

        /// Opens Ada's hand
        std::future<void> openHand();

        /// Closes Ada's hand
        std::future<void> closeHand();

        /// Get Body Node of End Effector
        dart::dynamics::BodyNodePtr getEndEffectorBodyNode();

        /// Get the right arm
        aikido::robot::RobotPtr getRightArm();

        /// Get the right arm
        aikido::robot::ConstRobotPtr getRightArm() const;

        aikido::robot::ConstRobotPtr getRightHandRobot() const;

        aikido::robot::RobotPtr getRightHandRobot();

        /// Get the hand as an Aikido::Hand
        std::shared_ptr<HumanHand> getHand();

        aikido::trajectory::TrajectoryPtr computeArmJointSpacePath(
                const std::vector<std::pair<double, Eigen::VectorXd>> &waypoints,
                const aikido::constraint::dart::CollisionFreePtr &collisionFree,
                const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
                trajPostProcessor
                = nullptr);

        aikido::trajectory::TrajectoryPtr computeArmJointSpacePath(
                const std::vector<std::pair<double, Eigen::VectorXd>> &waypoints,
                const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
                trajPostProcessor
                = nullptr) {
            return computeArmJointSpacePath(
                    waypoints, getSelfCollisionConstraint(), trajPostProcessor);
        }

        template<typename PostProcessor = aikido::planner::kunzretimer::KunzRetimer>
        void setDefaultPostProcessor(
                const Eigen::VectorXd &velocityLimits = Eigen::VectorXd(),
                const Eigen::VectorXd &accelerationLimits = Eigen::VectorXd(),
                const typename PostProcessor::Params &params = KunzParams());

        /// Starts the provided trajectory controllers if not started already.
        /// Makes sure that no other controllers are running and conflicting
        /// with the ones we are about to start.
        /// \return true if all controllers have been successfully switched
        bool startTrajectoryControllers();

        /// Turns off provided trajectory controllers controllers
        /// \return true if all controllers have been successfully switched
        bool stopTrajectoryControllers();

        /// Compute velocity limits from the MetaSkeleton
        Eigen::VectorXd getVelocityLimits(bool armOnly = false) const;

        /// Compute acceleration limits from the MetaSkeleton
        Eigen::VectorXd getAccelerationLimits(bool armOnly = false) const;

        /// Get Body Node of End Effector

    private:
        /// Switches between controllers.
        /// \param[in] startControllers Controllers to start.
        /// \param[in] stopControllers Controllers to stop.
        /// Returns true if controllers successfully switched.
        bool switchControllers(
                const std::vector<std::string> &startControllers,
                const std::vector<std::string> &stopControllers);

        // Call to spin first to pass current time to step
        void spin();

        void createTrajectoryExecutor(bool isHand);

        // Set the placement of the human in the plane.
        void setPlacementXYZ(const Eigen::Vector3d &placement);

    private:

        bool mSimulation;

        std::string mArmTrajControllerName;
        std::string mHandTrajControllerName;

        dart::dynamics::SkeletonPtr mHumanSkeleton;

        // Soft velocity and acceleration limits
        Eigen::VectorXd mSoftVelocityLimits;
        Eigen::VectorXd mSoftAccelerationLimits;
        Eigen::VectorXd mDefaultVelocityLimits;
        Eigen::VectorXd mDefaultAccelerationLimits;

        /// ROS node associated with this robot
        std::unique_ptr<::ros::NodeHandle> mNode;

        // Ros controller service client.
        std::unique_ptr<::ros::ServiceClient> mControllerServiceClient;

        // Ros joint state client.
        std::unique_ptr<aikido::control::ros::RosJointStateClient> mJointStateClient;

        std::string mEndEffectorName;

        /// Human's right arm
        aikido::robot::RobotPtr mRightArm;

        aikido::robot::RobotPtr mHandRobot;

        /// For trajectory executions
        std::unique_ptr<aikido::common::ExecutorThread> mThread;

        ros::Publisher mPub;

        std::shared_ptr<HumanHand> mHand;
    };

} // namespace human

#include "detail/Human-impl.h"
#include "HumanHand.h"

#endif // LIBHUMAN_HUMAN_HPP_
