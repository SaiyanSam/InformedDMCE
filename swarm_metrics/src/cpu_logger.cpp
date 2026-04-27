#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include <iostream>
#include <memory>

class CpuLogger {
public:
    CpuLogger() : nh_(), pnh_("~") {
        pnh_.param<int>("nRobots", n_robots_, 1);
        pnh_.param<std::string>("plannerType", planner_type_, "dmcts");
        
        // Match the parameter naming convention from overlap_logger
        nh_.param<double>("/logger/logFrequency", log_frequency_, 1.0);

        // List of process keywords to track. Can override in launch file.
        std::vector<std::string> default_procs = {
            "FrontierClusterServer",
            "PlannerServer",
            "RobotMapServer",
            "PathFindingServer",
            "NavigationServer",
            "SensorEmulatorServer",
            "ConnectivityServer",
            "GlobalMapServer",
            "LoggingServer",
            "ClockServer",
            "rviz",
            "RobotMarkerServer",
            "overlap_logger",
            "cpu_logger",
            "rosmaster"
        };
        pnh_.param("process_keywords", process_keywords_, default_procs);

        openLog();
        start_time_ = ros::Time::now().toSec();
        timer_ = nh_.createTimer(ros::Duration(1.0 / log_frequency_), &CpuLogger::timerCb, this);
        
        ROS_INFO("[CpuLogger] Started logging CPU & Load at %.1f Hz", log_frequency_);
    }

    ~CpuLogger() {
        if (log_.is_open()) {
            log_.close();
        }
    }

private:
    void openLog() {
        // Matches the directory structure from your overlap_logger
        std::string base_dir = ros::package::getPath("dmce_sim") + "/logs/" + 
                               std::to_string(n_robots_) + "robots/" + planner_type_ + "/";
        std::filesystem::create_directories(base_dir);

        int file_index = 1;
        while (true) {
            std::stringstream ss;
            ss << "cpu_load_" << std::setfill('0') << std::setw(4) << file_index << ".csv";
            log_path_ = base_dir + ss.str();
            if (!std::filesystem::exists(log_path_)) {
                break;
            }
            file_index++;
        }

        log_.open(log_path_, std::ios::out);

        // Write Headers
        log_ << "time_s";
        for (const auto& kw : process_keywords_) {
            log_ << "," << kw << "_%";
        }
        log_ << ",other_%,load_avg\n";
        log_.flush();
    }

    std::map<std::string, double> getCpuUsages() {
        std::map<std::string, double> usages;
        for (const auto& kw : process_keywords_) usages[kw] = 0.0;
        usages["other"] = 0.0;

        // Force terminal width to 512 columns so ROS process names are never truncated.
        // Run 'top' twice with a 0.1s delay. The SECOND output is the instantaneous CPU usage.
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("env COLUMNS=512 top -b -n 2 -d 0.1 -c", "r"), pclose);
        if (!pipe) {
            ROS_ERROR("[CpuLogger] popen() failed to execute top command!");
            return usages;
        }

        char line[1024];
        int batch = 0;
        while (fgets(line, sizeof(line), pipe.get()) != nullptr) {
            std::string sline(line);
            
            // Count how many times we see the header to know when the second batch starts
            if (sline.find("PID USER") != std::string::npos) {
                batch++;
                continue;
            }
            
            // Skip everything until we are in the second (instantaneous) batch
            if (batch < 2) continue;

            // Trim leading whitespace
            size_t start = sline.find_first_not_of(" \t");
            if (start == std::string::npos) continue;
            sline = sline.substr(start);

            std::stringstream ss(sline);
            std::string pid, user, pr, ni, virt, res, shr, s, cpu, mem, time;
            
            // Extract the standard 'top' columns
            if (!(ss >> pid >> user >> pr >> ni >> virt >> res >> shr >> s >> cpu >> mem >> time)) {
                continue; // Not a valid process line
            }
            
            // The rest of the line is the actual command running
            std::string cmd;
            std::getline(ss, cmd); 

            double cpu_val = 0.0;
            try {
                cpu_val = std::stod(cpu);
            } catch (...) { continue; }

            if (cpu_val <= 0.01) continue; // Ignore idle processes

            bool matched = false;
            for (const auto& kw : process_keywords_) {
                if (cmd.find(kw) != std::string::npos) {
                    usages[kw] += cpu_val;
                    matched = true;
                    break;
                }
            }
            
            if (!matched && cpu_val > 1.0) { 
                usages["other"] += cpu_val;
            }
        }
        return usages;
    }

    void timerCb(const ros::TimerEvent& event) {
        double current_time = ros::Time::now().toSec();
        double elapsed_t = current_time - start_time_;

        // 1. Get the CPU usage array
        std::map<std::string, double> current_usages = getCpuUsages();

        // 2. Read the 1-minute load average from the OS
        double load_avg = 0.0;
        std::ifstream load_file("/proc/loadavg");
        if (load_file.is_open()) {
            load_file >> load_avg; 
            load_file.close();
        }

        // 3. Write everything to the CSV
        log_ << std::fixed << std::setprecision(2) << elapsed_t;
        for (const auto& kw : process_keywords_) {
            log_ << "," << current_usages[kw];
        }
        log_ << "," << current_usages["other"] << "," << load_avg << "\n";
        log_.flush();
    }

    ros::NodeHandle nh_, pnh_;
    ros::Timer timer_;
    std::ofstream log_;

    std::string log_path_;
    std::vector<std::string> process_keywords_;
    
    double log_frequency_;
    double start_time_;
    int n_robots_;
    std::string planner_type_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cpu_logger");
    CpuLogger node;
    ros::spin();
    return 0;
}
