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
 * @file data_logger.h
 * @author Benjamin Navarro
 * @brief Definition of the DataLogger class
 * @date June 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/safety_controller.h>
#include <fstream>
#include <sstream>

namespace RSCL {

class DataLogger {
public:
	DataLogger(
		const SafetyController& controller,
		const std::string& directory,
		doubleConstPtr time,
		bool create_gnuplot_files = false,
		bool delay_disk_write = false);
	~DataLogger();

	void logRobotData(RobotConstPtr robot);
	void logExternalData(const std::string& data_name, const double* data, size_t data_count);
	void process();
	void operator()();

private:
	std::ofstream& createLog(const std::string& data_name, size_t data_count);
	std::ostream& getStream(std::ofstream& file);
	void logData(std::ofstream& file, const double* data, size_t data_count);

	const SafetyController& controller_;
	std::string directory_;
	doubleConstPtr time_;
	bool create_gnuplot_files_;
	bool delay_disk_write_;

	RobotConstPtr robot_;
	struct external_data_t {
		const double* data;
		size_t size;
	};

	std::map<std::string, std::ofstream> log_files_;
	std::map<std::ofstream*, std::stringstream> stored_data_;
	std::map<std::ofstream*, external_data_t> external_data_;
};

} // namespace RSCL
