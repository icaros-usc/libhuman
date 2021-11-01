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

void waitForUser(const std::string& msg)
{
    std::cout << msg;
    std::cin.get();
}

const SkeletonPtr makeBodyFromURDF(
        const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
        const std::string& uri,
        const Eigen::Isometry3d& transform)
{
    dart::utils::DartLoader urdfLoader;
    const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

    if (!skeleton)
        throw std::runtime_error("unable to load '" + uri + "'");

    dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
            ->setTransform(transform);
    return skeleton;
}

int main(int argc, char** argv)
{
    ROS_INFO("Starting ROS node.");
    ros::init(argc, argv, "simple_trajectories");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
    bool IS_SIM=true;
    // Create AIKIDO World
    if (!ros::isShutDown())
    {
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

//  human.setPlacementPose(humanPose);

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

        for (std::size_t i = 0; i < 3; ++i)
        {
            auto pose = Eigen::Isometry3d::Identity();
            pose.translation()
                    = tablePose.translation() + Eigen::Vector3d(i * 0.05, i * 0.10, 0.73);
            sodaPoses.push_back(pose);
        }

        std::vector<SkeletonPtr> sodaSkeletons;
        std::vector<std::shared_ptr<aikido::constraint::dart::TSR>> sodaTSRs;
        for (const auto& pose : sodaPoses)
        {
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
//        it seems that in py file that collision is not used.
//        // Create collision constraint
//        aikido::constraint::TestablePtr selfCollConstraint =
//              human.getSelfCollisionConstraint(
//                      human.getRightArmSpace(), human.getRightArm()->getMetaSkeleton());

    }





//  // Sample from TSR.
//  std::vector<std::pair<Eigen::VectorXd, double>> tsrSamples
//    = human.sampleRightTSR(sodaTSRs.at(0), 30, selfCollConstraint);
//
//  // Set sample.
//  human.getRightArm()->getMetaSkeleton()->setPositions(tsrSamples.at(0).first);
//
//  auto testState = human.getRightArmSpace()->createState();
//  human.getRightArmSpace()
//    ->convertPositionsToState(tsrSamples.at(0).first, testState);
//
//  if (selfCollConstraint->isSatisfied(testState))
//  {
//    std::cout << "" << std::endl;
//    std::cout << "GOOD FINAL STATE" << std::endl;
//    std::cout << tsrSamples.at(0).first.transpose() << std::endl;
//    std::cout << "FOUND " << tsrSamples.size() << " SOLS" << std::endl;
//  }

    waitForUser("Press [ENTER] to exit: ");

    return 0;
}
