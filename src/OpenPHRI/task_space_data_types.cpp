#include <OpenPHRI/task_space_data_types.h>

namespace phri {

/***			Pose			***/
Pose::Pose() :
	Pose(
		std::make_shared<phri::Vector3d>(phri::Vector3d::Zero()),
		std::make_shared<Eigen::Quaterniond>(Eigen::Quaterniond::Identity()))
{
}

Pose::Pose(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<Eigen::Quaterniond> orientation) :
	translation_(translation),
	orientation_(orientation)
{
}

Pose::Pose(phri::Vector3d translation, Eigen::Quaterniond orientation)  :
	Pose(
		std::make_shared<phri::Vector3d>(translation),
		std::make_shared<Eigen::Quaterniond>(orientation))
{
}

Pose::Pose(phri::Vector3d translation, phri::Vector3d euler_angles) :
	Pose(
		translation,
		Eigen::AngleAxisd(euler_angles.x(), Eigen::Vector3d::UnitX())       // Roll
		* Eigen::AngleAxisd(euler_angles.y(), Eigen::Vector3d::UnitY())     // Pitch
		* Eigen::AngleAxisd(euler_angles.z(), Eigen::Vector3d::UnitZ()))    // Yaw
{
}

Pose::Pose(phri::AffineTransform transformation) :
	Pose(
		transformation.translation(),
		static_cast<Eigen::Quaterniond>(transformation.rotation()))
{
}

std::shared_ptr<const phri::Vector3d> Pose::translation() const {
	return translation_;
}
std::shared_ptr<const Eigen::Quaterniond> Pose::orientation() const {
	return orientation_;
}
std::shared_ptr<phri::Vector3d> Pose::translation() {
	return translation_;
}
std::shared_ptr<Eigen::Quaterniond> Pose::orientation() {
	return orientation_;
}

phri::Vector6d Pose::getErrorWith(const Pose& other) const {
	phri::Vector6d error;
	error.block<3,1>(0,0) = *translation() - *other.translation();
	error.block<3,1>(3,0) = orientation()->getAngularError(*other.orientation());
	return error;
}

Pose::operator phri::AffineTransform() const {
	phri::AffineTransform transform;
	transform.setIdentity();
	transform.translate(*translation());
	transform.rotate(*orientation());
	return transform;
}

/***			Twist			***/
Twist::Twist() {

}
Twist::Twist(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<Eigen::Quaterniond> rotation) {

}
Twist::Twist(phri::Vector3d translation, phri::Vector3d rotation) {

}
Twist::Twist(phri::Vector6d twist) {

}

std::shared_ptr<const phri::Vector3d> Twist::translation() const {

}
std::shared_ptr<const Eigen::Quaterniond> Twist::rotation() const {

}
std::shared_ptr<phri::Vector3d> Twist::translation() {

}
std::shared_ptr<Eigen::Quaterniond> Twist::rotation() {

}

Twist::operator phri::Vector6d() const {

}

/***			Acceleration			***/
Acceleration::Acceleration() {

}
Acceleration::Acceleration(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<Eigen::Quaterniond> rotation) {

}
Acceleration::Acceleration(phri::Vector3d translation, phri::Vector3d rotation) {

}
Acceleration::Acceleration(phri::Vector6d acceleration) {

}

std::shared_ptr<const phri::Vector3d> Acceleration::translation() const {

}
std::shared_ptr<const Eigen::Quaterniond> Acceleration::rotation() const {

}
std::shared_ptr<phri::Vector3d> Acceleration::translation() {

}
std::shared_ptr<Eigen::Quaterniond> Acceleration::rotation() {

}

Acceleration::operator phri::Vector6d() const {

}


} // namespace phri
