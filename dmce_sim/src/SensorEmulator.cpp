#include "dmce_sim/SensorEmulator.hpp"

namespace dmce {
    // 1. Updated Constructor to accept and initialize commsRange
    SensorEmulator::SensorEmulator(const double range, const unsigned int rayCount, const OccupancyMap map, const double commsRange)
        : sensorRange_(range), commsRange_(commsRange), rayCount_(rayCount), groundTruthMap_(map)
    {
        if (sensorRange_ < 0) {
            throw std::invalid_argument("Sensor range must be non-negative.");
        }
        if (commsRange_ < 0) {
            throw std::invalid_argument("Communication range must be non-negative.");
        }
    }

    bool SensorEmulator::castRay_(
        const grid_map::Position& from,
        const grid_map::Position& to,
        std::vector<double>& x_pos,
        std::vector<double>& y_pos,
        std::vector<float>& occ) const
    {
        try {
            auto it = groundTruthMap_.getLineIterator(from, to);
            grid_map::Position cellPos;
            bool stop = false;
            if (it.isPastEnd()) {
                return false;
            }
            do {
                groundTruthMap_.getPosition(*it, cellPos);
                x_pos.push_back(cellPos.x());
                y_pos.push_back(cellPos.y());
                occ.push_back(groundTruthMap_.getOccupancy(*it));
                stop = groundTruthMap_.isOccupied(*it);
                ++it;
                stop |= it.isPastEnd();
            } while (!stop);
        } catch (std::invalid_argument e) {
            return false;
        }
        return true;
    }

    dmce_msgs::RobotMapUpdate SensorEmulator::getMapUpdate(const grid_map::Position& pos) {
        std::vector<double> x_pos;
        std::vector<double> y_pos;
        std::vector<float> occ;

        double angleIncrement = 2 * M_PI / rayCount_;
        double actualRange = sensorRange_ - groundTruthMap_.getResolution();
        for (unsigned int i = 0; i < rayCount_; i++) {
            double angle = i * angleIncrement;
            auto rayTarget = pos;
            rayTarget.x() += std::cos(angle) * actualRange;
            rayTarget.y() += std::sin(angle) * actualRange;
            castRay_(pos, rayTarget, x_pos, y_pos, occ);
        }

        dmce_msgs::RobotMapUpdate update;
        update.length = x_pos.size();
        update.values = occ;
        update.x_positions = x_pos;
        update.y_positions = y_pos;

        return update;
    }

    // 2. Added new method to check both distance and line-of-sight
    bool SensorEmulator::canCommunicate(const grid_map::Position& p1, const grid_map::Position& p2, bool restrictComms) const {
        // Enforce maximum communication range
        double distance = (p1 - p2).norm();
        if (distance > commsRange_) {
            return false; 
        }

        // Enforce line-of-sight if the restrictComms flag is true
        if (restrictComms) {
            try {
                auto it = groundTruthMap_.getLineIterator(p1, p2);
                for (; !it.isPastEnd(); ++it) {
                    if (groundTruthMap_.isOccupied(*it)) {
                        return false; // A wall/obstacle blocks the signal
                    }
                }
            } catch (std::invalid_argument e) {
                return false; // Iterator failed (e.g., positions out of map bounds)
            }
        }

        // Within range and (if required) line of sight is clear
        return true;
    }
};