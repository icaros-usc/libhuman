#include <iostream>
#include <Eigen/Dense>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pr_tsr/can.hpp>

#include "Human.hpp"

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

using dart::dynamics::SkeletonPtr;

void waitForUser(const std::string &msg) {
    std::cout << msg;
    std::cin.get();
}

const SkeletonPtr makeBodyFromURDF(
        const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
        const std::string &uri,
        const Eigen::Isometry3d &transform) {
    dart::utils::DartLoader urdfLoader;
    const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

    if (!skeleton)
        throw std::runtime_error("unable to load '" + uri + "'");

    dynamic_cast<dart::dynamics::FreeJoint *>(skeleton->getJoint(0))
            ->setTransform(transform);
    return skeleton;
}

int main(int argc, char **argv) {
    ROS_INFO("Starting ROS node.");
    ros::init(argc, argv, "simple_trajectories");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
    bool IS_SIM = true;
    // Create AIKIDO World

    // Load the human.
    ROS_INFO("Loading Human.");
    std::string modelSrc = "icaros";
    human::Human human(true);
    auto env = human.getWorld();

    // Set pose of human to "face" table.
    Eigen::Isometry3d humanPose = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d humanRot;
    humanRot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
               * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    humanPose.linear() = humanRot;

    // Start Visualization Topic
    static const std::string topicName = topicName + "/simple_trajectories";

    // Start the RViz viewer.
    ROS_INFO_STREAM(
            "Starting viewer. Please subscribe to the '"
                    << topicName
                    << "' InteractiveMarker topic in RViz.");
    aikido::rviz::InteractiveMarkerViewer viewer(topicName, baseFrameName, env);

    // Add Human to the viewer.
    viewer.setAutoUpdate(true);

    // Add a loaded table to the scene.
    const std::string tableURDFUri(
            "package://pr_assets/data/furniture/uw_demo_table.urdf");

    double tableHeight = 0.716475;

    Eigen::Isometry3d tablePose = Eigen::Isometry3d::Identity();
    tablePose.translation() = Eigen::Vector3d(0.6, 0.0, -tableHeight);
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    tablePose.linear() = rot;

    // Load table
    const auto resourceRetriever
            = std::make_shared<aikido::io::CatkinResourceRetriever>();
    SkeletonPtr table = makeBodyFromURDF(resourceRetriever, tableURDFUri, tablePose);

    human.getWorld()->addSkeleton(table);

    // Load cans on table.
    const std::string sodaName{"can"};
    const std::string sodaURDFUri("package://pr_assets/data/objects/can.urdf");

    std::vector<Eigen::Isometry3d> sodaPoses;

    for (std::size_t i = 0; i < 3; ++i) {
        auto pose = Eigen::Isometry3d::Identity();
        pose.translation()
                = tablePose.translation() + Eigen::Vector3d(i * 0.05, i * 0.10, 0.73);
        sodaPoses.push_back(pose);
    }

    std::vector<SkeletonPtr> sodaSkeletons;
    std::vector<std::shared_ptr<aikido::constraint::dart::TSR>> sodaTSRs;
    for (const auto &pose : sodaPoses) {
        auto soda = makeBodyFromURDF(resourceRetriever, sodaURDFUri, pose);
        soda->setName(sodaName);
        human.getWorld()->addSkeleton(std::move(soda));

        auto sodaTSR = std::make_shared<aikido::constraint::dart::TSR>(
                pr_tsr::getDefaultCanTSR());
        sodaTSR->mT0_w = pose;

        Eigen::Isometry3d graspTransform = Eigen::Isometry3d::Identity();
        Eigen::Matrix3d rot;
        rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
        graspTransform.linear() = rot;
        sodaTSR->mTw_e.matrix() = graspTransform.matrix();

        sodaSkeletons.push_back(soda);
        sodaTSRs.push_back(sodaTSR);
    }

    Eigen::VectorXd positions0 = human.getRightArm()->getCurrentConfiguration();
    Eigen::VectorXd positions1(7);
    positions1 << positions0;
    Eigen::VectorXd positions2(7);
    positions2 << positions0;
    Eigen::VectorXd positions3(7);
    positions3 << positions0;
    positions1(0) = positions1(0) + 0.5;
    positions1(2) = positions1(2) + 0.2;
    positions2(0) = positions2(0) + 1.5;
    positions2(2) = positions2(2) + 0.4;
    positions3(0) = positions3(0) - 0.4;
    positions3(2) = positions3(2) + 0.6;

    std::cout << "Good so far?" << std::endl;

    auto traj = human.getRightArm()->planToConfiguration(positions3);
    std::vector<std::pair<double, Eigen::VectorXd>> waypoints_rev;
    waypoints_rev.emplace_back(std::pair<double, Eigen::VectorXd>{0, positions3});
    waypoints_rev.emplace_back(std::pair<double, Eigen::VectorXd>{0, positions2});
    waypoints_rev.emplace_back(std::pair<double, Eigen::VectorXd>{0, positions1});
    waypoints_rev.emplace_back(std::pair<double, Eigen::VectorXd>{0, positions0});
    auto traj_rev = human.computeArmJointSpacePath(waypoints_rev);

    waitForUser("Press [ENTER] to execute");

    auto future = human.getRightArm()->executeTrajectory(traj);

    if (!future.valid())
    {
        std::__throw_future_error(0);
    }
    future.wait();
    // Throw any exceptions
    future.get();

    waitForUser("Press [ENTER] to execute");

    future = human.getRightArm()->executeTrajectory(traj_rev);

    if (!future.valid())
    {
        std::__throw_future_error(0);
    }
    future.wait();
    // Throw any exceptions
    future.get();

    waitForUser("Press [ENTER] to exit: ");

    return 0;
}
