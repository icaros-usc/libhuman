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
#define DEFAULT_CONF_OBJ_NS "adaConf"
#define DEFAULT_ARM_TRAJ_CTRL "trajectory_controller"
#define DEFAULT_HAND_TRAJ_CTRL "hand_controller"
#define DEFAULT_EE_NAME "end_effector"

#define DEFAULT_HUMAN_URDF "package://libhuman/robot/human_short.urdf"
#define DEFAULT_HUMAN_SRDF "package://libhuman/robot/human_short.srdf"


//        const std::chrono::milliseconds threadExecutionCycle{10};

        // Expose base class functions
//        using aikido::robot::Robot::getMetaSkeleton;
//        using aikido::robot::Robot::getStateSpace;

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
                const dart::common::Uri &humanUrdfUri = dart::common::Uri(""),
                const dart::common::Uri &humanSrdfUri = dart::common::Uri(""),
                aikido::planner::WorldPtr env = aikido::planner::World::create(),
                const std::string confNamespace = DEFAULT_CONF_OBJ_NS,
                const std::chrono::milliseconds threadCycle = DEFAULT_THREAD_CYCLE,

//                std::string modelSrc,
//                aikido::common::RNG::result_type rngSeed = std::random_device{}(),
//                const std::string &endEffectorName = "RHand3",
//                const std::string &armTrajectoryExecutorName = "rewd_trajectory_controller",
                const ::ros::NodeHandle *node = nullptr,
                aikido::common::RNG::result_type rngSeed = std::random_device{}(),
                const dart::common::ResourceRetrieverPtr &retriever
                = std::make_shared<aikido::io::CatkinResourceRetriever>());

//        Human(aikido::planner::WorldPtr env,
//              bool simulation,
//              std::string name,
//              const Eigen::Isometry3d &transform,
//              bool vis,
//              std::string modelSrc,
//              aikido::common::RNG::result_type rngSeed = std::random_device{}(),
//              const std::string &endEffectorName = "RHand3",
//              const std::string &armTrajectoryExecutorName = "rewd_trajectory_controller",
//              const ::ros::NodeHandle *node = nullptr,
//              const dart::common::ResourceRetrieverPtr &retriever
//              = std::make_shared<aikido::io::CatkinResourceRetriever>());

        virtual ~Human();

        /// Creates and returns a trajectory executor.
//        std::shared_ptr<aikido::control::TrajectoryExecutor> createTrajectoryExecutor();

//        /// \copydoc ConcreteRobot::postProcessPath.
//        template<typename PostProcessor>
//        aikido::trajectory::UniqueSplinePtr postProcessPath(
//                const aikido::trajectory::Trajectory *path,
//                const aikido::constraint::TestablePtr &constraint,
//                const typename PostProcessor::Params &postProcessorParams,
//                const Eigen::VectorXd &velocityLimits = Eigen::Vector6d::Zero(),
//                const Eigen::VectorXd &accelerationLimits = Eigen::Vector6d::Zero()) {
//            // Don't plan above 70% hard velocity limit
//            // Unless requested explicitly
//            const double DEFAULT_LIMITS_BUFFER = 0.7;
//
//            bool velLimitsInvalid
//                    = (velocityLimits.squaredNorm() == 0.0)
//                      || velocityLimits.size() != getVelocityLimits().size();
//            auto sentVelocityLimits = velLimitsInvalid
//                                      ? DEFAULT_LIMITS_BUFFER * getVelocityLimits()
//                                      : velocityLimits;
//
//            bool accLimitsInvalid
//                    = (accelerationLimits.squaredNorm() == 0.0)
//                      || accelerationLimits.size() != getAccelerationLimits().size();
//            auto sentAccelerationLimits
//                    = accLimitsInvalid ? DEFAULT_LIMITS_BUFFER * getAccelerationLimits()
//                                       : accelerationLimits;
//
//            return mHuman->postProcessPath<PostProcessor>(
//                    sentVelocityLimits,
//                    sentAccelerationLimits,
//                    path,
//                    constraint,
//                    postProcessorParams);
//        }

        void step(const std::chrono::system_clock::time_point &timepoint) override;

        /// Opens Ada's hand
        std::future<void> openHand();

        /// Closes Ada's hand
        std::future<void> closeHand();

        /// Get Body Node of End Effector
        dart::dynamics::BodyNodePtr getEndEffectorBodyNode();

        // Documentation inherited.
//        std::future<void> executeTrajectory(
//                const aikido::trajectory::TrajectoryPtr &trajectory) const override;

        // Documentation inherited.
//        boost::optional<Eigen::VectorXd> getNamedConfiguration(
//                const std::string &name) const override;

        // Documentation inherited.
//        void setNamedConfigurations(
//                std::unordered_map<std::string, const Eigen::VectorXd>
//                namedConfigurations) override;

        // Documentation inherited.
//        std::string getName() const override;

        // Documentation inherited.
//        dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const override;

        // Documentation inherited.
//        aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr getStateSpace()
//        const override;

        // Documentation inherited.
//        void setRoot(Robot *robot) override;

        // Documentation inherited.
//        void step(const std::chrono::system_clock::time_point &timepoint) override;

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

        // Utility function to (re)-create and set trajectory executors
//        void createTrajectoryExecutor(bool isHand);

//
//
//        /// Compute IK with left arm.
//        std::vector<std::pair<Eigen::VectorXd, double>> computeLeftIK(
//                const Eigen::Isometry3d &target,
//                const int numSol,
//                const aikido::constraint::TestablePtr constraint);
//
//        // Compute IK with right arm.
//        std::vector<std::pair<Eigen::VectorXd, double>> computeRightIK(
//                const Eigen::Isometry3d &target,
//                const int numSol,
//                const aikido::constraint::TestablePtr constraint);
//
//        /// Sample a TSR with left arm.
//        std::vector<std::pair<Eigen::VectorXd, double>> sampleLeftTSR(
//                std::shared_ptr<aikido::constraint::dart::TSR> &tsr,
//                const int numSamples,
//                const aikido::constraint::TestablePtr constraint);

        /// Sample a TSR with right arm.
//        std::vector<std::pair<Eigen::VectorXd, double>> sampleRightTSR(
//                std::shared_ptr<aikido::constraint::dart::TSR> &tsr,
//                const int numSamples,
//                const aikido::constraint::TestablePtr constraint);

        // Set the placement of the human in the plane.
        void setPlacementXYZ(const Eigen::Vector3d &placement);

//        // Set the full pose of the human.
//        void setPlacementPose(const Eigen::Isometry3d &pose);
//
//        aikido::trajectory::TrajectoryPtr planRightArmToTSR(const aikido::constraint::dart::TSR &tsr,
//                                                            const aikido::constraint::dart::CollisionFreePtr &collisionFree,
//                                                            double timelimit, size_t maxNumTrials,
//                                                            const aikido::distance::ConfigurationRankerPtr &ranker);
//
//        aikido::trajectory::TrajectoryPtr planRightArmToTSR(std::shared_ptr<aikido::constraint::dart::TSR> &tsr,
//                                                            const aikido::constraint::dart::CollisionFreePtr &collisionFree,
//                                                            double timelimit, size_t maxNumTrials,
//                                                            const aikido::distance::ConfigurationRankerPtr &ranker);

    private:
        /// Schema description for named configurations YAML file.
        ///
        /// Maps a configuration name (string) to a configuration
//        using ConfigurationMap = std::unordered_map<std::string, Eigen::VectorXd>;
//
//        /// Initialize an arm.
//        ///
//        /// \param[in] armName Name of the arm, either "left" or "right"
//        /// \param[in] retriever Resource retriever to resolve URIs
//        void configureArm(
//                const std::string &armName,
//                const dart::common::ResourceRetrieverPtr &retriever);

        /// Constructs the arm and the hand, and the manipulator.
        /// \param armName Name of the arm.
        /// \param retriever Resource retriever.
        /// \param executor Trajectory executor for the arm.
        /// \param collisionDetector Collision detector for the manipulator.
        /// \param selfCollisionFilter self collision filter for the manipulator.
//        aikido::robot::ConcreteManipulatorPtr configureRightArm(const std::string &armName,
//                                                                const dart::common::ResourceRetrieverPtr &retriever,
//                                                                const aikido::control::TrajectoryExecutorPtr &executor,
//                                                                dart::collision::CollisionDetectorPtr collisionDetector,
//                                                                const std::shared_ptr<dart::collision::BodyNodeCollisionFilter> &
//                                                                selfCollisionFilter);

//        // Private helper for common IK logic between left/right arm.
//        std::vector<std::pair<Eigen::VectorXd, double>> computeIK(
//                const Eigen::Isometry3d &target,
//                const int numSol,
//                const dart::dynamics::InverseKinematicsPtr &ik,
//                const std::shared_ptr<aikido::constraint::Sampleable> &ikSeedSampler,
//                const dart::dynamics::MetaSkeletonPtr &arm,
//                const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
//                const dart::dynamics::BodyNodePtr &hand,
//                const aikido::constraint::TestablePtr constraint);

        // Helper for the above for just a single solution. Returns the solution along
//        // with its SE(3) error.
//        std::pair<Eigen::VectorXd, double> computeSingleIK(
//                const Eigen::Isometry3d &target,
//                const dart::dynamics::InverseKinematicsPtr &ik,
//                const std::shared_ptr<aikido::constraint::SampleGenerator> &ikSeedGenerator,
//                const dart::dynamics::MetaSkeletonPtr &arm,
//                const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
//                const dart::dynamics::BodyNodePtr &hand);

        // Sample IK from the given TSR. Helper used for this between left/right arms.
//        std::vector<std::pair<Eigen::VectorXd, double>> sampleTSR(
//                std::shared_ptr<aikido::constraint::dart::TSR> &tsr,
//                const int numSamples,
//                const dart::dynamics::InverseKinematicsPtr &ik,
//                const std::shared_ptr<aikido::constraint::Sampleable> &ikSeedSampler,
//                const dart::dynamics::MetaSkeletonPtr &arm,
//                const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
//                const dart::dynamics::BodyNodePtr &hand,
//                const aikido::constraint::TestablePtr constraint);

        // Helper that in-place sorts IK solutions on their pose error and filters on
        // collision constraint (if given).
//        void filterSortSolutions(
//                std::vector<std::pair<Eigen::VectorXd, double>> &solutionsAndErrors,
//                const aikido::constraint::TestablePtr constraint,
//                aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace);

        /// mHuman is a wrapper around the meta skeleton
//        aikido::robot::ConcreteRobotPtr mHuman;

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


        /// Trajectory executor
//        std::shared_ptr<aikido::control::TrajectoryExecutor> mTrajectoryExecutor;

        // Name of the End Effector in the URDF
        // might differ for different human configurations
        std::string mEndEffectorName;
//        std::string mHandBaseName;
//        std::string mArmBaseName;
//        std::string mArmEndName;

//        // The hand
//        HumanHandPtr mHand;
//
//        // Correction transform to place human "right side up".
//        Eigen::Isometry3d mCorrectionTransform;
//
//
//
//        std::string mArmTrajectoryExecutorName;
//
//        /// Random generator
//        aikido::common::RNGWrapper<std::mt19937> mRng;
//
//        /// World that human is in.
//        aikido::planner::WorldPtr mWorld;
//
//        /// Underlying robot skeleton
//        dart::dynamics::SkeletonPtr mRobotSkeleton;
//
//        /// Robot planning state space
//        aikido::statespace::dart::MetaSkeletonStateSpacePtr mSpace;
//
//        // MetaSkeleton for human's torso. Used mainly for IK collision checking.
//        dart::dynamics::MetaSkeletonPtr mTorso;
//
//        /// Human's left arm
//        dart::dynamics::MetaSkeletonPtr mLeftArm;
//        aikido::statespace::dart::MetaSkeletonStateSpacePtr mLeftArmSpace;
//        dart::dynamics::BodyNodePtr mLeftHand;
//        dart::dynamics::InverseKinematicsPtr mLeftIk;

        /// Human's right arm
        aikido::robot::RobotPtr mRightArm;

        aikido::robot::RobotPtr mHandRobot;

//        aikido::statespace::dart::MetaSkeletonStateSpacePtr mRightArmSpace;
////  dart::dynamics::MetaSkeletonPtr mRightArm;
////  aikido::statespace::dart::MetaSkeletonStateSpacePtr mRightArmSpace;
//        dart::dynamics::BodyNodePtr mRightHand;
//        dart::dynamics::InverseKinematicsPtr mRightIk;

        /// Human concrete robot
//        aikido::robot::ConcreteRobotPtr mRobot;

        /// For trajectory executions
        std::unique_ptr<aikido::common::ExecutorThread> mThread;

        ros::Publisher mPub;

        std::shared_ptr<HumanHand> mHand;

//        aikido::trajectory::TrajectoryPtr planToTSR(const aikido::statespace::dart::MetaSkeletonStateSpacePtr &space,
//                                                    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
//                                                    const dart::dynamics::BodyNodePtr &bn,
//                                                    const aikido::constraint::dart::TSRPtr &tsr,
//                                                    const aikido::constraint::dart::CollisionFreePtr &collisionFree,
//                                                    double timelimit, size_t maxNumTrials,
//                                                    const aikido::distance::ConfigurationRankerPtr &ranker);
//
    };

} // namespace human

#include "detail/Human-impl.h"
#include "HumanHand.h"

#endif // LIBHUMAN_HUMAN_HPP_
