#include <velocity_constraint.h>

using namespace ADamp;
using namespace Eigen;

/***		Constructor & destructor		***/
VelocityConstraint::VelocityConstraint(
	std::shared_ptr<Matrix6d> tool_base_transform,
	std::shared_ptr<Matrix6d> damping_matrix,
	std::shared_ptr<Vector6d> reference_velocity,
	std::shared_ptr<Vector6d> force,
	std::shared_ptr<double> maximum_velocity) :
	tool_base_transform_(tool_base_transform),
	damping_matrix_(damping_matrix),
	reference_velocity_(reference_velocity),
	force_(force),
	maximum_velocity_(maximum_velocity)
{

}

/***		Algorithm		***/
double VelocityConstraint::compute() {
	double alpha = 1.;
	Vector3d feq = force_->block<3,1>(0,0);

	if(feq.norm() > 1e-3) {
		Matrix3d R_b_t = tool_base_transform_->block<3,3>(0,0);
		Vector3d Yfeq = R_b_t * damping_matrix_->block<3,3>(0,0).inverse() * force_->block<3,1>(0,0);
		Vector3d vr = R_b_t * reference_velocity_->block<3,1>(0,0);
		double Yfeq_sq = Yfeq.squaredNorm();
		double Yfeq_dot_vr = Yfeq.dot(vr);
		double vm = (*maximum_velocity_);
		double vr_sq = vr.squaredNorm();

		alpha = (-Yfeq_dot_vr + sqrt(Yfeq_dot_vr*Yfeq_dot_vr - Yfeq_sq*(vr_sq - vm*vm))) / Yfeq_sq;
	}

	return alpha;
}
