#pragma once

#include <ros/ros.h>
#include "grid_map_core/GridMap.hpp"

// #include "dmce/RobotMap.hpp"
#include "dmce_core/OccupancyMap.hpp"

#include "dmce_msgs/RobotMapUpdate.h"

namespace dmce {
	class SensorEmulator {
		double sensorRange_;
		double commsRange_;
		unsigned int rayCount_;

		OccupancyMap groundTruthMap_;

		bool castRay_(
			const grid_map::Position& from,
			const grid_map::Position& to,
			std::vector<double>& pos_x,
			std::vector<double>& pos_y,
			std::vector<float>& occ) const;

	public:
		/**
		 * Constructor.
		 * @param range [m] Effective maximum sensor range.
		 * @param rayCount [-] Number of emulated lidar rays to cast.
		 * @param map The map object to be used as ground truth.
		 */
		SensorEmulator(const double range = 1, const unsigned int rayCount = 144, const OccupancyMap map = OccupancyMap(), const double commsRange = 20.0);

		/**
		 * Given a position, emulates a sensor sweep and returns
		 * a RobotMapUpdate message containing the revealed cells.
		 */
		dmce_msgs::RobotMapUpdate getMapUpdate(const grid_map::Position& pos);

		/**
         * Checks if two positions can communicate based on distance and line-of-sight.
         */
        bool canCommunicate(const grid_map::Position& p1, const grid_map::Position& p2, bool restrictComms) const;

		SensorEmulator(const SensorEmulator&) = default;
		SensorEmulator& operator=(const SensorEmulator&) = default;
		SensorEmulator(SensorEmulator&&) = default;
		SensorEmulator& operator=(SensorEmulator&&) = default;
	};
};
