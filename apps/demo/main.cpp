/*      File: main.cpp
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

#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <list>

#include <OpenPHRI/OpenPHRI.h>
#include <OpenPHRI/drivers/vrep_driver.h>

#include "state_machine.h"

using namespace phri;

double SAMPLE_TIME = 0.005;

constexpr bool USE_LASER_SCANNER = false;

bool _stop = false;

void sigint_handler(int sig) {
    _stop = true;
}

int main(int argc, char const* argv[]) {

    /***				Robot				***/
    auto robot = std::make_shared<Robot>(
        "LBR4p", // Robot's name, must match V-REP model's name
        7);      // Robot's joint count

    /***				V-REP driver				***/
    VREPDriver driver(robot, SAMPLE_TIME, "", "", -1000);

    driver.start();

    std::shared_ptr<LaserScannerDetector> laser_detector;
    std::shared_ptr<const VectorXd> laser_data;
    if (USE_LASER_SCANNER) {
        laser_data = driver.initLaserScanner("Hokuyo");
        laser_detector = std::make_shared<LaserScannerDetector>(
            laser_data, 270. * M_PI / 180., 0.2, 3.);
    }

    /***			Controller configuration			***/
    auto safety_controller = SafetyController(robot);

    auto state_machine = StateMachine(robot, safety_controller, laser_detector,
                                      argc > 1); // skip teaching

    Clock clock(SAMPLE_TIME);
    DataLogger logger("/tmp", clock.getTime(),
                      true,  // create gnuplot files
                      true); // delay disk write

    logger.logSafetyControllerData(&safety_controller);
    logger.logRobotData(robot);

    int fsm_states[2];
    logger.logExternalData("fsm state", fsm_states, 2);
    double operator_distance;
    logger.logExternalData("operator distance", &operator_distance, 1);
    double sep_dist_vlim;
    logger.logExternalData("sep dist vlim", &sep_dist_vlim, 1);
    double t_avg = 0.;
    logger.logExternalData("tavg_fsm_and_controller", &t_avg, 1);

    auto update_external_data = [&state_machine, &fsm_states,
                                 &operator_distance, &sep_dist_vlim]() {
        fsm_states[0] = static_cast<int>(state_machine.getTeachState());
        fsm_states[1] = static_cast<int>(state_machine.getReplayState());
        operator_distance = state_machine.getOperatorDistance();
        sep_dist_vlim = state_machine.getSeparationDistanceVelocityLimitation();
    };

    signal(SIGINT, sigint_handler);

    driver.enableSynchonous(true);
    size_t count = 10;
    while (not(driver.read() and
               (not laser_data or laser_data->size() == 1080) and
               count-- == 0) and
           not _stop) {
        driver.nextStep();
    }

    if (laser_detector) {
        laser_detector->init();
    }
    state_machine.init();

    if (not _stop)
        std::cout << "Starting main loop\n";

    while (not _stop) {
        if (driver.read()) {

            auto t_start = std::chrono::high_resolution_clock::now();
            _stop |= state_machine.compute();
            safety_controller.compute();
            auto t_end = std::chrono::high_resolution_clock::now();

            t_avg =
                (1e6 * 0.01 *
                     std::chrono::duration<double>(t_end - t_start).count() +
                 0.99 * t_avg);

            if (not driver.send()) {
                std::cerr << "Can'send robot data to V-REP" << std::endl;
            }

            // std::cout << "t_avg (us): " << t_avg*1e6 << std::endl;

            clock();
            update_external_data();
            logger();
        } else {
            std::cerr << "Can't get robot data from V-REP" << std::endl;
        }

        driver.nextStep();
    }

    driver.enableSynchonous(false);
    driver.stop();

    return 0;
}
