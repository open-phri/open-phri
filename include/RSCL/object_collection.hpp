#pragma once

#include <map>
#include <iostream>
#include <typeinfo>
#include <memory>
#include <cxxabi.h>

namespace RSCL {

template<typename T>
class ObjectCollection {
public:
	ObjectCollection() : verbose_(false) {
	}

	virtual ~ObjectCollection() = default;

	void setVerbose(bool on) {
		if(on) {
			class_name_ = type(*this);
			collection_name_ = "Object";
		}
		verbose_ = on;
	}

	void setVerbose(bool on, const std::string& class_name, const std::string& collection_name) {
		if(on) {
			class_name_ = class_name;
			collection_name_ = collection_name;
		}
		verbose_ = on;
	}

	bool addObject(const std::string& name, T object, bool force = false) {
		if((objects_.find(name) != objects_.end())and not force) {
			if(verbose_) {
				std::cerr << "In " << class_name_ << "::add" << collection_name_ << ": an object called \"" << name << "\" already exists. Not replaced (force = false)" << std::endl;
			}
			return false;
		}
		objects_[name] = object;
		return true;
	}

	bool removeObject(const std::string& name) {
		auto obj = objects_.find(name);
		if(obj == objects_.end()) {
			if(verbose_) {
				std::cerr << "In " << class_name_ << "::remove" << collection_name_ << ": no object called \"" << name << "\"" << std::endl;
			}
			return false;
		}
		objects_.erase(obj);
		return true;
	}

	T getObject(const std::string& name) {
		T obj;
		auto elem = objects_.find(name);
		if(elem != objects_.end()) {
			obj = elem->second;
		}
		else if(verbose_) {
			std::cerr << "In " << class_name_ << "::get" << collection_name_ << ": no object called \"" << name << "\"" << std::endl;
		}
		return obj;
	}

	using iterator = typename std::map<std::string,T>::iterator;
	using const_iterator = typename std::map<std::string,T>::const_iterator;

	iterator begin() {
		return objects_.begin();
	}

	const_iterator begin() const {
		return objects_.begin();
	}

	iterator end() {
		return objects_.end();
	}

	const_iterator end() const {
		return objects_.end();
	}

protected:
	std::map<std::string, T> objects_;

	bool verbose_;
	std::string class_name_;
	std::string collection_name_;
private:
	std::string demangle(const char* name) {
		int status = -4; // some arbitrary value to eliminate the compiler warning

		std::unique_ptr<char, void (*)(void*)> res {
			abi::__cxa_demangle(name, NULL, NULL, &status),
			std::free
		};

		return (status==0) ? res.get() : name;
	}

	template<typename TT>
	std::string type(const TT& t) {
		return demangle(typeid(t).name());
	}
};

} // namespace RSCL
