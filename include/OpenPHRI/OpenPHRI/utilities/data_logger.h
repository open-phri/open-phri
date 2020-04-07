/*      File: data_logger.h
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
 * @file data_logger.h
 * @author Benjamin Navarro
 * @brief Definition of the DataLogger class
 * @date June 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/type_aliases.h>
#include <OpenPHRI/robot.h>
#include <memory>
#include <fstream>
#include <sstream>
#include <map>

namespace phri {

class SafetyController;

class DataLogger {
    struct external_data;
    template <typename T> struct external_data_t;

public:
    /**
     * Creates a new data logger
     * @param directory            Directory in which log files will be stored
     * @param time                 A shared pointer to the current time
     * @param create_gnuplot_files Create a file to easily open logs in gnuplot
     * (default = false)
     * @param delay_disk_write     Log data in RAM and write it to the disk when
     * writeStoredDataToDisk is called or on destruction (default = false)
     */
    DataLogger(const std::string& directory, std::shared_ptr<const double> time,
               bool create_gnuplot_files = false,
               bool delay_disk_write = false);
    ~DataLogger();

    /**
     * Log all the data related to a SafetyController (inputs, constraints,
     * intermediate computation)
     * @param controller Pointer to a SafetyController object
     */
    void logSafetyControllerData(SafetyController* controller);
    /**
     * Log all the data related to a Robot (all fields present in the structure)
     * @param robot A shared pointer to a Robot object
     */
    void logRobotData(Robot const* robot);

    /**
     * Log any array of data
     * @param data_name  The name given to this data (used for file name)
     * @param data       A pointer to an array of data to log
     * @param data_count The number of values in the array
     */
    template <typename T>
    void logExternalData(const std::string& data_name, const T* data,
                         size_t data_count) {
        external_data_[&createLog(data_name, data_count)] =
            std::make_unique<external_data_t<T>>(data, data_count);
    }

    /**
     * Reset the data logger back to its initial state (no controller, robot or
     * external data to log)
     */
    void reset();

    /**
     * Log all the given data
     */
    void process();

    /**
     * Shortcut for process
     */
    void operator()();

    /**
     * Write all previously saved data to the disk. Called during destruction if
     * delay_disk_write was set to true during construction.
     */
    void writeStoredDataToDisk();

    /**
     * Close all the currently open files. Called during destruction.
     */
    void closeFiles();

private:
    std::ofstream& createLog(const std::string& data_name, size_t data_count);
    [[nodiscard]] std::ostream& getStream(std::ofstream& file);
    void logData(std::ofstream& file, const double* data, size_t data_count);
    void logData(std::ofstream& file, const external_data& data);

    SafetyController* controller_;
    std::string directory_;
    std::shared_ptr<const double> time_;
    bool create_gnuplot_files_;
    bool delay_disk_write_;

    struct external_data {
        external_data() = default;
        virtual ~external_data() = default;
        const void* data;
        size_t size;

        virtual void write(std::ostream& stream) const = 0;
    };

    template <typename T>
    struct external_data_t : virtual public external_data {
        external_data_t(const T* data, size_t size) {
            this->data = static_cast<const void*>(data);
            this->size = size;
        }

        void write(std::ostream& stream) const override {
            auto ptr = static_cast<const T*>(data);
            for (size_t i = 0; i < size - 1; ++i) {
                stream << ptr[i] << '\t';
            }
            stream << ptr[size - 1] << '\n';
        }
    };

    std::map<std::string, std::ofstream> log_files_;
    std::map<std::ofstream*, std::stringstream> stored_data_;
    std::map<std::ofstream*, std::unique_ptr<external_data>> external_data_;
};

} // namespace phri
