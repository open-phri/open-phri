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

const phri::Vector3d& Pose::translation() const {
	return *translation_;
}
const Eigen::Quaterniond& Pose::orientation() const {
	return *orientation_;
}
phri::Vector3d& Pose::translation() {
	return *translation_;
}
Eigen::Quaterniond& Pose::orientation() {
	return *orientation_;
}

std::shared_ptr<const phri::Vector3d> Pose::translationPtr() const {
	return translation_;
}
std::shared_ptr<const Eigen::Quaterniond> Pose::orientationPtr() const {
	return orientation_;
}
std::shared_ptr<phri::Vector3d> Pose::translationPtr() {
	return translation_;
}
std::shared_ptr<Eigen::Quaterniond> Pose::orientationPtr() {
	return orientation_;
}

phri::Vector6d Pose::getErrorWith(const Pose& other) const {
	phri::Vector6d error;
	error.block<3,1>(0,0) = translation() - other.translation();
	error.block<3,1>(3,0) = orientationPtr()->getAngularError(other.orientation());
	return error;
}

Pose& Pose::integrate(const Twist& twist, double sample_time) {
	translation() += twist.translation()*sample_time;
	orientation().integrate(twist.rotation(), sample_time);
	return *this;
}


Pose::operator phri::AffineTransform() const {
	phri::AffineTransform transform;
	transform.setIdentity();
	transform.translate(translation());
	transform.rotate(orientation());
	return transform;
}

/***			Twist			***/
Twist::Twist() :
	Twist(
		std::make_shared<phri::Vector3d>(phri::Vector3d::Zero()),
		std::make_shared<phri::Vector3d>(phri::Vector3d::Zero()))
{
}
Twist::Twist(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<phri::Vector3d> rotation) :
	translation_(translation),
	rotation_(rotation)
{
}
Twist::Twist(phri::Vector3d translation, phri::Vector3d rotation) :
	Twist(
		std::make_shared<phri::Vector3d>(translation),
		std::make_shared<phri::Vector3d>(rotation))
{
}
Twist::Twist(phri::Vector6d twist) :
	Twist(
		twist.block<3,1>(0,0),
		twist.block<3,1>(3,0))
{
}

const phri::Vector3d& Twist::translation() const {
	return *translation_;
}
const phri::Vector3d& Twist::rotation() const {
	return *rotation_;
}
phri::Vector3d& Twist::translation() {
	return *translation_;
}
phri::Vector3d& Twist::rotation() {
	return *rotation_;
}

std::shared_ptr<const phri::Vector3d> Twist::translationPtr() const {
	return translation_;
}
std::shared_ptr<const phri::Vector3d> Twist::rotationPtr() const {
	return rotation_;
}
std::shared_ptr<phri::Vector3d> Twist::translationPtr() {
	return translation_;
}
std::shared_ptr<phri::Vector3d> Twist::rotationPtr() {
	return rotation_;
}

Twist& Twist::integrate(const Acceleration& acceleration, double sample_time) {
	translation() += acceleration.translation()*sample_time;
	rotation() += acceleration.rotation()*sample_time;
	return *this;
}

Twist::operator phri::Vector6d() const {
	phri::Vector6d twist;
	twist.block<3,1>(0,0) = translation();
	twist.block<3,1>(3,0) = rotation();
	return twist;
}

/***			Acceleration			***/
Acceleration::Acceleration() :
	Acceleration(
		std::make_shared<phri::Vector3d>(phri::Vector3d::Zero()),
		std::make_shared<phri::Vector3d>(phri::Vector3d::Zero()))
{
}
Acceleration::Acceleration(std::shared_ptr<phri::Vector3d> translation, std::shared_ptr<phri::Vector3d> rotation) :
	translation_(translation),
	rotation_(rotation)
{
}
Acceleration::Acceleration(phri::Vector3d translation, phri::Vector3d rotation) :
	Acceleration(
		std::make_shared<phri::Vector3d>(translation),
		std::make_shared<phri::Vector3d>(rotation))
{
}
Acceleration::Acceleration(phri::Vector6d acceleration) :
	Acceleration(
		acceleration.block<3,1>(0,0),
		acceleration.block<3,1>(3,0))
{
}

const phri::Vector3d& Acceleration::translation() const {
	return *translation_;
}
const phri::Vector3d& Acceleration::rotation() const {
	return *rotation_;
}
phri::Vector3d& Acceleration::translation() {
	return *translation_;
}
phri::Vector3d& Acceleration::rotation() {
	return *rotation_;
}

std::shared_ptr<const phri::Vector3d> Acceleration::translationPtr() const {
	return translation_;
}
std::shared_ptr<const phri::Vector3d> Acceleration::rotationPtr() const {
	return rotation_;
}
std::shared_ptr<phri::Vector3d> Acceleration::translationPtr() {
	return translation_;
}
std::shared_ptr<phri::Vector3d> Acceleration::rotationPtr() {
	return rotation_;
}

Acceleration::operator phri::Vector6d() const {
	phri::Vector6d acceleration;
	acceleration.block<3,1>(0,0) = translation();
	acceleration.block<3,1>(3,0) = rotation();
	return acceleration;
}


} // namespace phri
