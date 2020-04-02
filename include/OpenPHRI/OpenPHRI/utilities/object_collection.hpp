/*      File: object_collection.hpp
 *       This file is part of the program open-phri
 *       Program description : OpenPHRI: a generic framework to easily and
 * safely control robots in interactions with humans Copyright (C) 2017 -
 * Benjamin Navarro (LIRMM). All Right reserved.
 *
 *       This software is free software: you can redistribute it and/or modify
 *       it under the terms of the LGPL license as published by
 *       the Free Software Foundation, either version 3
 *       of the License, or (at your option) any later version.
 *       This software is distributed in the hope that it will be useful,
 *       but WITHOUT ANY WARRANTY without even the implied warranty of
 *       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *       LGPL License for more details.
 *
 *       You should have received a copy of the GNU Lesser General Public
 * License version 3 and the General Public License version 3 along with this
 * program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file itemect_collection.hpp
 * @author Benjamin Navarro
 * @brief Definition of the ObjectCollection class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/detail/universal_wrapper.hpp>

#include <map>
#include <iostream>
#include "demangle.h"
#include "exceptions.h"

namespace phri {

/** @brief Manage a collection of items.
 *  @details Provides add/get/remove methods and begin/end methods to iterate
 * through the managed items
 */
template <typename T> class ObjectCollection {
public:
    ObjectCollection() : verbose_(false) {
    }

    virtual ~ObjectCollection() = default;

    /**
     * @brief Set the verbosity on or off. If on, error messages can be printed
     * when calling the add/get/remove methods.
     * @details This method is to be used by derived classes. The derived class
     * name will be retreived and used in the messages.
     * @param on True to enable verbosity, false to disable it.
     */
    void setVerbose(bool on) {
        if (on) {
            class_name_ = phri::getTypeName(*this);
            collection_name_ = "Object";
        }
        verbose_ = on;
    }

    /**
     * @brief Set the verbosity on or off. If on, error messages can be printed
     * when calling the add/get/remove methods.
     * @details This method is to be used if instanciated as a attribute in
     * other classes. The instanciating class can provide its name and the
     * collection name to customize the messages. message example: "In
     * Foo::addBar: ..." with class_name=Foo and collection_name=Bar
     * @param on True to enable verbosity, false to disable it.
     * @param class_name The name of the class wrapping this one.
     * @param collection_name The name of the collection used by the wrapping
     * class.
     */
    void setVerbose(bool on, const std::string& class_name,
                    const std::string& collection_name = "Object") {
        if (on) {
            class_name_ = class_name;
            collection_name_ = collection_name;
        }
        verbose_ = on;
    }

    /**
     * @brief Add a new item to the collection.
     * @param name The name given to the item. Must be unique. Used to latter
     * get/remove the item.
     * @param item The item to add.
     * @param force [optional] If true, any item with the same name will be
     * overriden.
     * @return true if the item has successfully been added to the collection,
     * false otherwise.
     */
    template <typename ItemT>
    bool add(const std::string& name, ItemT&& item, bool force = false) {
        if (not force and (items_.find(name) != items_.end())) {
            if (verbose_) {
                std::cerr << "In phri::" << class_name_ << "::add"
                          << collection_name_ << ": an item called \"" << name
                          << "\" already exists. Not replaced (force = false)"
                          << std::endl;
            }
            return false;
        }
        detail::UniversalWrapper<T> wrapper{std::forward<ItemT>(item)};
        items_.emplace(std::make_pair(name, std::move(wrapper)));
        return true;
    }

    /**
     * @brief Remove an item from the collection.
     * @param name The name of the item
     * @return true if the item has successfully been removed from the
     * collection, false otherwise.
     */
    virtual bool remove(const std::string& name) {
        auto item = items_.find(name);
        if (item == items_.end()) {
            throw std::domain_error(OPEN_PHRI_ERROR("No item called " + name));
        }
        items_.erase(item);
        return true;
    }

    /**
     * @brief Remove all items from the collection.
     */
    virtual void removeAll() {
        // items_.clear(); Don't do that, the erase method can be overrided, and
        // for a good reason!
        for (const auto& item : items_) {
            remove(item.first);
        }
    }

    /**
     * @brief Retrieve a item from the collection.
     * @param name The name given to the item to retreive.
     * @return A pointer to the item. Store a null pointer if the item doesn't
     * exist.
     */
    virtual T& get(const std::string& name) {
        auto elem = items_.find(name);
        if (elem != items_.end()) {
            return elem->second.ref();
        } else {
            throw std::domain_error(OPEN_PHRI_ERROR("No item called " + name));
        }
    }

    /**
     * @brief Retrieve a item from the collection.
     * @param name The name given to the item to retreive.
     * @return A pointer to the item. Store a null pointer if the item doesn't
     * exist.
     */
    virtual const T& get(const std::string& name) const {
        auto elem = items_.find(name);
        if (elem != items_.end()) {
            return elem->second.cref();
        } else {
            throw std::domain_error(OPEN_PHRI_ERROR("No item called " + name));
        }
    }

    using iterator =
        typename std::map<std::string, detail::UniversalWrapper<T>>::iterator;
    using const_iterator =
        typename std::map<std::string,
                          detail::UniversalWrapper<T>>::const_iterator;

    /**
     * @brief Provide an iterator to the first element of the collection
     * @return The iterator
     */
    iterator begin() {
        return items_.begin();
    }

    /**
     * @brief Provide a const iterator to the first element of the collection
     * @return The iterator
     */
    const_iterator begin() const {
        return items_.begin();
    }

    /**
     * @brief Provide an iterator to the last element of the collection
     * @return The iterator
     */
    iterator end() {
        return items_.end();
    }

    /**
     * @brief Provide a const iterator to the last element of the collection
     * @return The iterator
     */
    const_iterator end() const {
        return items_.end();
    }

protected:
    std::map<std::string, detail::UniversalWrapper<T>> items_;

private:
    bool verbose_;
    std::string class_name_;
    std::string collection_name_;
};

} // namespace phri
