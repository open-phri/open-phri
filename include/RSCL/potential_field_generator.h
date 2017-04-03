#pragma once

#include <RSCL/force_generator.h>
#include <RSCL/definitions.h>
#include <map>

namespace RSCL {

enum class PotentialFieldType {
	Attractive,
	Repulsive
};

struct PotentialFieldObject {
	PotentialFieldObject(
		PotentialFieldType type,
		doubleConstPtr gain,
		doubleConstPtr threshold_distance,
		Vector6dConstPtr object_position) :
		type(type),
		gain(gain),
		threshold_distance(threshold_distance),
		object_position(object_position)
	{

	}

	PotentialFieldType type;
	doubleConstPtr gain;
	doubleConstPtr threshold_distance;
	Vector6dConstPtr object_position;
};

using PotentialFieldObjectPtr = std::shared_ptr<PotentialFieldObject>;
using PotentialFieldObjectConstPtr = std::shared_ptr<const PotentialFieldObject>;


class PotentialFieldGenerator : public ForceGenerator {
public:
	PotentialFieldGenerator() = default;
	PotentialFieldGenerator(Vector6dConstPtr robot_position);
	~PotentialFieldGenerator() = default;

	void setVerbose(bool on);

	virtual Vector6d compute() override;

	bool addObject(const std::string& name, PotentialFieldObjectPtr object, bool force = false);
	bool removeObject(const std::string& name);
	PotentialFieldObjectPtr getObject(const std::string& name);

private:
	Vector6dConstPtr robot_position_;

	std::map<std::string, PotentialFieldObjectPtr> objects_;

	bool verbose_;
};

using PotentialFieldGeneratorPtr = std::shared_ptr<PotentialFieldGenerator>;
using PotentialFieldGeneratorConstPtr = std::shared_ptr<const PotentialFieldGenerator>;

} // namespace RSCL
