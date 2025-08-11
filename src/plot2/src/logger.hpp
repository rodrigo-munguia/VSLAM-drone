#ifndef LOGGER_TYPES_H
#define LOGGER_TYPES_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <vector>
#include <filesystem>  // For std::filesystem::create_directory

class Logger {
public:
    Logger(const std::string& base_filename) 
        : start_time(std::chrono::steady_clock::now()) {
        namespace fs = std::filesystem;

        // Create logs directory if it doesn't exist
        fs::create_directory("logs");

        // Generate label_time at construction (once)
        auto now = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&t_c);

        std::ostringstream time_stream;
        time_stream << std::put_time(&tm, "%Y%m%d_%H%M%S");
        std::string label_time = time_stream.str();

        // Final filename
        filename = "logs/" + base_filename + "_" + label_time + ".csv";

        // Open the file in write mode and write the header
        std::ofstream file(filename, std::ios::out);
        if (file.is_open()) {
            file << "time,X_gt,X_e,Y_gt,Y_e,Z_gt,Z_e\n";
        }
    }

    ~Logger() = default;

    void log(double X_gt, double X_e, double Y_gt, double Y_e, double Z_gt, double Z_e) {
        using namespace std::chrono;
        auto current_time = steady_clock::now();
        auto elapsed = duration_cast<milliseconds>(current_time - start_time).count();

        // Calculate seconds and centiseconds
        int ss = static_cast<int>(elapsed / 1000);
        int cc = static_cast<int>((elapsed % 1000) / 10);

        // Format time
        std::ostringstream ss_stream;
        ss_stream << std::setw(2) << std::setfill('0') << ss << '.'
                  << std::setw(2) << std::setfill('0') << cc;
        std::string label_time = ss_stream.str();

        // Build log line
        std::ostringstream log_stream;
        log_stream << label_time << ','
                   << std::fixed << std::setprecision(6)
                   << X_gt << ',' << X_e << ',' << Y_gt << ',' << Y_e << ','
                   << Z_gt << ',' << Z_e << '\n';

        // Append to the file
        std::ofstream file(filename, std::ios::app);
        if (file.is_open()) {
            file << log_stream.str();
        }
    }

private:
    std::string filename;
    std::chrono::steady_clock::time_point start_time;
};

#endif
