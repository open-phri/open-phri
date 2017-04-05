#undef NDEBUG

#include <RSCL/RSCL.h>
#include <iostream>

using namespace RSCL;
using namespace std;

int main(int argc, char const *argv[]) {

	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity() * 100.);
	auto safety_controller = SafetyController(damping_matrix);
	safety_controller.setVerbose(true);

	auto tcp_velocity = safety_controller.getTCPVelocity();

	auto rob_pos = make_shared<Vector6d>(Vector6d::Zero());
	auto potential_field_generator = make_shared<PotentialFieldGenerator>(rob_pos);
	potential_field_generator->setVerbose(true);

	safety_controller.addForceGenerator("potential field", potential_field_generator);

	auto obs_pos = make_shared<Vector6d>(Vector6d::Zero());
	auto obstacle = make_shared<PotentialFieldObject>(
		PotentialFieldType::Repulsive,
		make_shared<double>(10.),   // gain
		make_shared<double>(0.2),   // threshold distance
		obs_pos);

	auto tgt_pos = make_shared<Vector6d>(Vector6d::Zero());
	auto target = make_shared<PotentialFieldObject>(
		PotentialFieldType::Attractive,
		make_shared<double>(10.),  // gain
		make_shared<double>(std::numeric_limits<double>::infinity()),   // threshold distance
		tgt_pos);

	// Step #1 : addObject
	bool ok;
	ok = potential_field_generator->addObject("obstacle", obstacle);
	assert_msg("Step #1", ok == true);

	// Step #2 : re-addObject, force=false
	ok = potential_field_generator->addObject("obstacle", obstacle);
	assert_msg("Step #2", ok == false);

	// Step #3 : re-addObject, force=true
	ok = potential_field_generator->addObject("obstacle", obstacle, true);
	assert_msg("Step #3", ok == true);

	// Step #4 : removeObject
	ok = potential_field_generator->removeObject("obstacle");
	assert_msg("Step #4", ok == true);

	// Step #5 : re-removeObject
	ok = potential_field_generator->removeObject("obstacle");
	assert_msg("Step #5", ok == false);

	// Step #6 : getObject
	potential_field_generator->addObject("obstacle", obstacle);
	auto obj = potential_field_generator->getObject("obstacle");
	assert_msg("Step #6", obj == obstacle);

	// Step #7 : 1 obstacle > threshold distance
	(*obs_pos)(0) = 0.3;
	safety_controller.updateTCPVelocity();
	assert_msg("Step #7", tcp_velocity->isZero());

	// Step #8 : 1 obstacle < threshold distance
	(*obs_pos)(0) = 0.1;
	safety_controller.updateTCPVelocity();
	assert_msg("Step #8", tcp_velocity->dot(*obs_pos) < 0.);

	// Step #9 : 1 target
	ok = potential_field_generator->removeObject("obstacle");
	potential_field_generator->addObject("target", target);
	(*tgt_pos)(0) = 0.1;
	(*tgt_pos)(1) = 0.2;
	safety_controller.updateTCPVelocity();
	assert_msg("Step #9", tcp_velocity->dot(*tgt_pos) > 0.);

	return 0;
}
