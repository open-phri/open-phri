#ifndef CONSTRAINT_H
#define CONSTRAINT_H

namespace ADamp {

class Constraint {
public:
	Constraint() = default;
	virtual ~Constraint() = default;

	virtual double compute() = 0;
};

}

#endif /* CONSTRAINT_H */
