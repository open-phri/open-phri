/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of RSCL <https://gite.lirmm.fr/navarro/RSCL>.
 *
 *  RSCL is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  RSCL is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with RSCL.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file object_collection.hpp
 * @author Benjamin Navarro
 * @brief Definition of the ObjectCollection class
 * @date April 2014
 * @ingroup RSCL
 */

#pragma once

#include <map>
#include <iostream>
#include <typeinfo>
#include <memory>
#include <cxxabi.h>

namespace RSCL {

/** @brief Manage a collection of objects.
 *  @details Provides add/get/remove object methods and begin/end methods to iterate through the managed objects
 */
template<typename T>
class ObjectCollection {
public:
	ObjectCollection() : verbose_(false) {
	}

	virtual ~ObjectCollection() = default;

	/**
	 * @brief Set the verbosity on or off. If on, error messages can be printed when calling the add/get/remove methods.
	 * @details This method is to be used by derived classes. The derived class name will be retreived and used in the messages.
	 * @param on True to enable verbosity, false to disable it.
	 */
	void setVerbose(bool on) {
		if(on) {
			class_name_ = type(*this);
			collection_name_ = "Object";
		}
		verbose_ = on;
	}

	/**
	 * @brief Set the verbosity on or off. If on, error messages can be printed when calling the add/get/remove methods.
	 * @details This method is to be used instanciated as a attribute is other classes. The instanciating class can provide its name and the collection name to customize the messages.
	 * message example: "In Foo::addBar: ..." with class_name=Foo and collection_name=Bar
	 * @param on True to enable verbosity, false to disable it.
	 * @param class_name The name of the class wrapping this one.
	 * @param collection_name The name of the collection used by the wrapping class.
	 */
	void setVerbose(bool on, const std::string& class_name, const std::string& collection_name = "Object") {
		if(on) {
			class_name_ = class_name;
			collection_name_ = collection_name;
		}
		verbose_ = on;
	}

	/**
	 * @brief Add a new object to the collection.
	 * @param name The name given to the object. Must be unique. Used to latter get/remove the object.
	 * @param object The object to add.
	 * @param force [optional] If true, any object with the same name will be overriden.
	 * @return true if the object has successfully been added to the collection, false otherwise.
	 */
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

	/**
	 * @brief Remove an object from the collection.
	 * @param name The name of the object
	 * @return true if the object has successfully been removed from the collection, false otherwise.
	 */
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

	/**
	 * @brief Retrieve a object from the collection.
	 * @param name The name given to the object to retreive.
	 * @return A pointer to the object. Store a null pointer if the object doesn't exist.
	 */
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

	/**
	 * @brief Provide an iterator to the first element of the collection
	 * @return The iterator
	 */
	iterator begin() {
		return objects_.begin();
	}

	/**
	 * @brief Provide a const iterator to the first element of the collection
	 * @return The iterator
	 */
	const_iterator begin() const {
		return objects_.begin();
	}

	/**
	 * @brief Provide an iterator to the last element of the collection
	 * @return The iterator
	 */
	iterator end() {
		return objects_.end();
	}

	/**
	 * @brief Provide a const iterator to the last element of the collection
	 * @return The iterator
	 */
	const_iterator end() const {
		return objects_.end();
	}

protected:
	std::map<std::string, T> objects_;

private:
	bool verbose_;
	std::string class_name_;
	std::string collection_name_;

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
