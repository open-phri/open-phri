#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>

using namespace phri;
using namespace std;

TEST_CASE("Frame transformations") {
    AffineTransform tcp_in_world;
    tcp_in_world.linear() =
        Eigen::AngleAxisd(M_PI / 2, Vector3d::UnitX()).toRotationMatrix();

    FrameAdapter::setTransform(FrameAdapter::world(), tcp_in_world,
                               FrameAdapter::frame("tcp"));

    // std::cout << "World -> TCP: " << FrameAdapter::getTransform(

    Pose tcp_pose(FrameAdapter::frame("tcp"));
    tcp_pose.translation() = Vector3d(1., 2., 3);
    auto world_pose1 = FrameAdapter::transform(tcp_pose, FrameAdapter::world());
    auto world_pose2 = tcp_pose.transform(FrameAdapter::world());

    std::cout << "Pose @ TCP: " << tcp_pose << std::endl;
    std::cout << "Pose @ World: " << world_pose2 << std::endl;

    Twist tcp_twist(FrameAdapter::frame("tcp"));
    tcp_twist.translation() = Vector3d(1., 2., 3.);
    tcp_twist.rotation() = Vector3d(4., 5., 6.);
    auto world_twist1 =
        FrameAdapter::transform(tcp_twist, FrameAdapter::world());
    auto world_twist2 = tcp_twist.transform(FrameAdapter::world());

    std::cout << "Twist @ TCP: " << tcp_twist << std::endl;
    std::cout << "Twist @ World: " << world_twist2 << std::endl;
}