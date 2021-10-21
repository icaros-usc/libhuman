#include "Human.hpp"
#include "util.h"

#include <cassert>
#include <mutex>
#include <stdexcept>
#include <string>

#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/common/RNG.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/robot/util.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <dart/common/Timer.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <controller_manager_msgs/SwitchController.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#undef dtwarn
#define dtwarn (::dart::common::colorErr("Warning", __FILE__, __LINE__, 33))

#undef dtinfo
#define dtinfo (::dart::common::colorMsg("Info", 32))

namespace human {

    namespace internal {

        dart::dynamics::BodyNodePtr getBodyNodeOrThrow(
                const dart::dynamics::MetaSkeletonPtr &skeleton,
                const std::string &bodyNodeName) {
            auto bodyNode = skeleton->getBodyNode(bodyNodeName);

            if (!bodyNode) {
                std::stringstream message;
                message << "Bodynode [" << bodyNodeName << "] does not exist in skeleton.";
                throw std::runtime_error(message.str());
            }

            return bodyNode;
        }

        inline const dart::common::Uri getDartURI(
                const dart::common::Uri providedUri,
                const std::string confNamespace,
                const std::string key,
                const std::string defaultUri) {
            if (providedUri.toString() != "file://") {
                return providedUri;
            }

            // Get Default from Parameter Server
            std::string uri = "";
            ros::param::param<std::string>(
                    "/" + confNamespace + "/" + key, uri, defaultUri);
            return dart::common::Uri(uri);
        }

    } // namespace internal

    using dart::collision::CollisionGroup;
    using dart::dynamics::Chain;
    using dart::dynamics::SkeletonPtr;
    using dart::dynamics::BodyNodePtr;
    using dart::dynamics::InverseKinematics;
    using dart::dynamics::InverseKinematicsPtr;

    using aikido::constraint::dart::CollisionFree;
    using aikido::constraint::dart::CollisionFreePtr;
    using aikido::constraint::dart::createSampleableBounds;
    using aikido::constraint::dart::TSRPtr;
    using aikido::constraint::Sampleable;
    using aikido::constraint::SampleGenerator;
    using aikido::constraint::TestablePtr;
//using aikido::robot::ConcreteManipulatorPtr;
    using aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr;
    using aikido::statespace::dart::MetaSkeletonStateSaver;
    using aikido::statespace::dart::MetaSkeletonStateSpace;
    using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
    using aikido::trajectory::TrajectoryPtr;
    using aikido::trajectory::UniqueSplinePtr;

    dart::common::Uri defaultPRLUrdfUri{"package://libhuman/robot/man1.urdf"};
    dart::common::Uri defaultPRLSrdfUri{"package://libhuman/robot/man1.srdf"};
    dart::common::Uri defaultVisPRLUrdfUri{"package://libhuman/robot/vis_man1.urdf"};
    dart::common::Uri defaultVisPRLSrdfUri{"package://libhuman/robot/vis_man1.srdf"};
    dart::common::Uri defaultICAROSUrdfUri{"package://libhuman/robot/human_short.urdf"};
    dart::common::Uri defaultICAROSSrdfUri{"package://libhuman/robot/human_short.srdf"};
    dart::common::Uri defaultVisICAROSUrdfUri{"package://libhuman/robot/vis_human_short.urdf"};
    dart::common::Uri defaultVisICAROSSrdfUri{"package://libhuman/robot/vis_human_short.srdf"};
    const dart::common::Uri namedConfigurationsUri{"TODO"};


// Arm trajectory controllers that are meant to be used by the human.
// Needs to be consistent with the configurations in human_launch.
// Right now there is no such a thing.
    const std::vector<std::string> availableArmTrajectoryExecutorNames{
            "trajectory_controller",
            "rewd_trajectory_controller",
            "move_until_touch_topic_controller"};

    namespace {


        double computeSE3Distance(
                const Eigen::Isometry3d &firstPose,
                const Eigen::Isometry3d &secondPose
        ) {
            double conversionRatioFromRadiusToMeter = 0.17;

            // Borrowed from VFP, should do the same thing as PrPy.
            return aikido::planner::vectorfield::computeGeodesicDistance(
                    firstPose, secondPose, conversionRatioFromRadiusToMeter);
        }
    } // ns

//==============================================================================
    Human::Human(
            bool simulation,
            const dart::common::Uri &humanUrdfUri,
            const dart::common::Uri &humanSrdfUri,
            aikido::planner::WorldPtr env,
            const std::string confNamespace,
            const std::chrono::milliseconds threadCycle,
            const ::ros::NodeHandle *node,
            aikido::common::RNG::result_type rngSeed,
            const dart::common::ResourceRetrieverPtr &retriever)
            : aikido::robot::ros::RosRobot(
            internal::getDartURI(humanUrdfUri, confNamespace, "default_urdf", DEFAULT_HUMAN_URDF),
            internal::getDartURI(humanSrdfUri, confNamespace, "default_srdf", DEFAULT_HUMAN_SRDF),
            "human",
            retriever),

              mSimulation(simulation) {
        // MetaSkeleton loaded by RosRobot Constructor
        // Set up other args
        setWorld(env);
        setRNG(std::make_unique<aikido::common::RNGWrapper<std::mt19937>>(rngSeed));

        if (!node) {
            mNode = std::make_unique<::ros::NodeHandle>();
        } else {
            mNode = std::make_unique<::ros::NodeHandle>(*node);
        }

        double limit;
        mNode->param<double>("/" + confNamespace + "/default_accel_lim", limit, 0);
        mSoftAccelerationLimits = mDefaultAccelerationLimits
                = (limit != 0)
                  ? Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
                  : mMetaSkeleton->getAccelerationUpperLimits();
        mNode->param<double>("/" + confNamespace + "/default_vel_lim", limit, 0);
        mSoftVelocityLimits = mDefaultVelocityLimits
                = (limit != 0)
                  ? Eigen::VectorXd::Constant(mMetaSkeleton->getNumDofs(), limit)
                  : mMetaSkeleton->getVelocityUpperLimits();

        // Create sub-robot (arm)
        std::vector<std::string> armNodes;
        mNode->getParam("/" + confNamespace + "/arm", armNodes);
        if (armNodes.size() < 2) {
            std::stringstream message;
            message << "Configuration [/" << confNamespace << "/arm] is required.";
            throw std::runtime_error(message.str());
        }
        auto armBase = internal::getBodyNodeOrThrow(mMetaSkeleton, armNodes[0]);
        auto armEnd = internal::getBodyNodeOrThrow(mMetaSkeleton, armNodes[1]);
        auto arm = dart::dynamics::Chain::create(armBase, armEnd, "adaArm");
        mRightArm = registerSubRobot(arm, "humanRightArm");
        if (!mRightArm) {
            throw std::runtime_error("Could not create arm");
        }

        // Register initial End Effector Node
        std::string endEffector;
        mNode->param<std::string>(
                "/" + confNamespace + "/end_effector", mEndEffectorName, DEFAULT_EE_NAME);
        auto handEnd = internal::getBodyNodeOrThrow(mMetaSkeleton, mEndEffectorName);

        // Create sub-robot (hand)
        std::string handBaseName;
        if (!mNode->getParam("/" + confNamespace + "/hand_base", handBaseName)) {
            std::stringstream message;
            message << "Configuration [/" << confNamespace
                    << "/hand_base] is required.";
            throw std::runtime_error(message.str());
        }
        auto handBase = internal::getBodyNodeOrThrow(mMetaSkeleton, handBaseName);
        auto hand = dart::dynamics::Branch::create(
                dart::dynamics::Branch::Criteria(handBase), "adaHand");
        mHandRobot = registerSubRobot(hand, "adaHand");
        if (!mHandRobot) {
            throw std::runtime_error("Could not create hand");
        }

        // Create Trajectory Executors
        // Should not execute trajectories on whole arm by default
        setTrajectoryExecutor(nullptr);

        // Arm Trajectory Executor
        mNode->param<std::string>(
                "/" + confNamespace + "/arm_controller",
                mArmTrajControllerName,
                DEFAULT_ARM_TRAJ_CTRL);
        createTrajectoryExecutor(false);

        // Hand Trajectory Executor
        mNode->param<std::string>(
                "/" + confNamespace + "/hand_controller",
                mHandTrajControllerName,
                DEFAULT_HAND_TRAJ_CTRL);
        createTrajectoryExecutor(true);

        // Load the named configurations if available
        std::string nameConfigs;
        if (mNode->getParam("/" + confNamespace + "/named_configs", nameConfigs)) {
            auto rootNode = aikido::io::loadYAML(nameConfigs, retriever);
            if (rootNode["hand"]) {
                mHandRobot->setNamedConfigurations(
                        aikido::robot::util::parseYAMLToNamedConfigurations(
                                rootNode["hand"]));
            }
            if (rootNode["arm"]) {
                mHandRobot->setNamedConfigurations(
                        aikido::robot::util::parseYAMLToNamedConfigurations(rootNode["arm"]));
            }
        }

        // Use limits to set default postprocessor
        setDefaultPostProcessor(
                mSoftVelocityLimits, mSoftAccelerationLimits, KunzParams());

        // Create joint state updates
        if (!mSimulation) {
            // Real Robot, create state client
            mControllerServiceClient = std::make_unique<::ros::ServiceClient>(
                    mNode->serviceClient<controller_manager_msgs::SwitchController>(
                            "controller_manager/switch_controller"));
            mJointStateClient
                    = std::make_unique<aikido::control::ros::RosJointStateClient>(
                    mMetaSkeleton->getBodyNode(0)->getSkeleton(),
                    *mNode,
                    "/joint_states",
                    1);
        } else {
            // Simulation, create state publisher
            mPub = mNode->advertise<sensor_msgs::JointState>("joint_states", 5);
        }

        // Create Inner AdaHand
        mHand = std::make_shared<HumanHand>(this, handBase, handEnd);

        // Start driving self-thread
        mThread = std::make_unique<aikido::common::ExecutorThread>(
                std::bind(&Human::spin, this), threadCycle);
    }


    Human::~Human() {
        mThread->stop();
    }


    void Human::step(const std::chrono::system_clock::time_point &timepoint) {
        if (!mThread || !mThread->isRunning())
            return;

        Robot::step(timepoint);

        if (!mSimulation && mJointStateClient) {
            // Spin joint state client
            mJointStateClient->spin();

            // Lock Skeleton
            std::lock_guard<std::mutex> lock(getRootSkeleton()->getMutex());

            // Get most recent joint states
            try {
                mMetaSkeleton->setPositions(
                        mJointStateClient->getLatestPosition(*mMetaSkeleton));
            }
            catch (const std::exception &e) {
                dtwarn << "Issue reading joints: " << e.what() << std::endl;
            }
        } else {
            // Lock Skeleton
            std::lock_guard<std::mutex> lock(getRootSkeleton()->getMutex());

            // Publish joint states to /joint_states
            sensor_msgs::JointState state;
            state.header.stamp = ros::Time::now();
            for (auto joint : mMetaSkeleton->getDofs()) {
                state.name.push_back(joint->getName());
                state.position.push_back(joint->getPosition());
                state.velocity.push_back(joint->getVelocity());
                state.effort.push_back(joint->getForce());
            }
            mPub.publish(state);
        }
    }

    //==============================================================================
    aikido::robot::RobotPtr Human::getRightArm() {
        return mRightArm;
    }

    aikido::robot::ConstRobotPtr Human::getRightArm() const {
        return mRightArm;
    }

//==============================================================================
    aikido::robot::RobotPtr Human::getRightHandRobot() {
        return mHandRobot;
    }

//========================================================================================
    aikido::robot::ConstRobotPtr Human::getRightHandRobot() const {
        return mHandRobot;
    }

    //========================================================================================
//==============================================================================
    std::shared_ptr<Human::HumanHand> Human::getHand() {
        return mHand;
    }

//==============================================================================
    void Human::spin() {
        if (mThread->isRunning()) {
            step(std::chrono::system_clock::now());
        }
    }

    //==================================================

    aikido::trajectory::TrajectoryPtr Human::computeArmJointSpacePath(
            const std::vector<std::pair<double, Eigen::VectorXd>> &waypoints,
            const aikido::constraint::dart::CollisionFreePtr &collisionFree,
            const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
            trajPostProcessor) {
        auto stateSpace
                = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
                        mRightArm->getMetaSkeletonClone().get());

        std::shared_ptr<aikido::statespace::GeodesicInterpolator> interpolator
                = std::make_shared<aikido::statespace::GeodesicInterpolator>(stateSpace);
        std::shared_ptr<aikido::trajectory::Interpolated> traj
                = std::make_shared<aikido::trajectory::Interpolated>(
                        stateSpace, interpolator);

        for (auto &waypoint : waypoints) {
            auto state = stateSpace->createState();
            try {
                stateSpace->convertPositionsToState(waypoint.second, state);
            }
            catch (const std::exception &e) {
                dtwarn << "Cannot convert configuration to robot state: " << e.what()
                       << std::endl;
                return nullptr;
            }
            traj->addWaypoint(waypoint.first, state);
        }

        // Postprocess if enabled or provided
        auto postprocessor
                = (trajPostProcessor)
                  ? trajPostProcessor
                  : ((mEnablePostProcessing) ? mRightArm->getDefaultPostProcessor()
                                             : nullptr);
        if (traj && postprocessor) {
            return postprocessor->postprocess(
                    *traj, *(cloneRNG().get()), collisionFree);

            // Else return raw path
        }
        return traj;
    }


    //==============================================================================
    bool Human::startTrajectoryControllers() {
        return switchControllers(
                std::vector<std::string>{mArmTrajControllerName, mHandTrajControllerName},
                std::vector<std::string>());
    }


    bool Human::stopTrajectoryControllers() {
        cancelAllTrajectories();
        return switchControllers(
                std::vector<std::string>(),
                std::vector<std::string>{mArmTrajControllerName,
                                         mHandTrajControllerName});
    }


    //==============================================================================
    Eigen::VectorXd Human::getVelocityLimits(bool armOnly) const {
        return (armOnly) ? mSoftVelocityLimits.segment(
                0, mRightArm->getMetaSkeleton()->getNumDofs())
                         : mSoftVelocityLimits;
    }

//==============================================================================
    Eigen::VectorXd Human::getAccelerationLimits(bool armOnly) const {
        return (armOnly) ? mSoftAccelerationLimits.segment(
                0, mRightArm->getMetaSkeleton()->getNumDofs())
                         : mSoftAccelerationLimits;
    }

    void Human::createTrajectoryExecutor(bool isHand) {
        std::string controller
                = isHand ? mHandTrajControllerName : mArmTrajControllerName;
        aikido::robot::RobotPtr subrobot = isHand ? mHandRobot : mRightArm;
        using aikido::control::KinematicSimulationTrajectoryExecutor;
        using aikido::control::ros::RosTrajectoryExecutor;

        if (mSimulation) {
            subrobot->setTrajectoryExecutor(
                    std::make_shared<KinematicSimulationTrajectoryExecutor>(
                            subrobot->getMetaSkeleton()->getBodyNode(0)->getSkeleton()));
        } else {
            std::string serverName = controller + "/follow_joint_trajectory";
            auto exec = std::make_shared<RosTrajectoryExecutor>(
                    *mNode,
                    serverName,
                    DEFAULT_ROS_TRAJ_INTERP_TIME,
                    DEFAULT_ROS_TRAJ_GOAL_TIME_TOL);
            subrobot->setTrajectoryExecutor(exec);
        }
    }

    //==============================================================================
    bool Human::switchControllers(
            const std::vector<std::string> &startControllers,
            const std::vector<std::string> &stopControllers) {
        if (!mNode)
            throw std::runtime_error("Ros node has not been instantiated.");

        if (!mControllerServiceClient)
            throw std::runtime_error("ServiceClient not instantiated.");

        controller_manager_msgs::SwitchController srv;
        // First try stopping the started controllers
        // Avoids us falsely detecting a failure if already started
        srv.request.stop_controllers = startControllers;
        srv.request.strictness
                = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
        mControllerServiceClient->call(srv); // Don't care about response code

        // Actual command
        srv.request.start_controllers = startControllers;
        srv.request.stop_controllers = stopControllers;
        srv.request.strictness
                = controller_manager_msgs::SwitchControllerRequest::STRICT;

        return mControllerServiceClient->call(srv) && srv.response.ok;
    }

    std::future<void> Human::openHand() {
        return mHand->executePreshape("open");
    }

    std::future<void> Human::closeHand() {
        return mHand->executePreshape("closed");
    }

    dart::dynamics::BodyNodePtr Human::getEndEffectorBodyNode() {
        return internal::getBodyNodeOrThrow(mMetaSkeleton, mEndEffectorName);
    }

} // ns

