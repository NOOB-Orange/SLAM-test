// SPDX-License-Identifier: GPL-3.0-or-later
// Headless sequential monocular input. All geometry is estimated by ORB-SLAM3.
#include "System.h"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

struct InputFrame { long index; double time; std::string path; };

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "Usage: mono_video vocabulary settings dataset output realtime[0|1]\n";
        return 2;
    }
    try {
        const std::string dataset = argv[3], output = argv[4];
        const bool realtime = std::stoi(argv[5]) != 0;
        std::ifstream manifest(dataset + "/frames.tsv");
        if (!manifest) throw std::runtime_error("Cannot open frames.tsv");
        std::string header, relative;
        std::getline(manifest, header);
        std::vector<InputFrame> frames;
        long index; double timestamp;
        while (manifest >> index >> timestamp >> relative) {
            if (!std::isfinite(timestamp) || (!frames.empty() && timestamp <= frames.back().time))
                throw std::runtime_error("Timestamps must increase strictly");
            frames.push_back({index, timestamp, dataset + "/" + relative});
        }
        if (frames.empty()) throw std::runtime_error("No frames in manifest");
        std::ofstream states(output + "/tracking.csv");
        if (!states) throw std::runtime_error("Cannot write tracking.csv");
        states << "frame_index,timestamp,state,tracking_ms\n" << std::setprecision(17);
        ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);
        for (size_t i = 0; i < frames.size(); ++i) {
            const auto& frame = frames[i];
            cv::Mat image = cv::imread(frame.path, cv::IMREAD_COLOR); // BGR, Camera.RGB=0
            if (image.empty()) throw std::runtime_error("Cannot read " + frame.path);
            const auto started = std::chrono::steady_clock::now();
            slam.TrackMonocular(image, frame.time);
            const double seconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            states << frame.index << ',' << frame.time << ',' << slam.GetTrackingState()
                   << ',' << 1000*seconds << '\n';
            if (i % 100 == 0 || i+1 == frames.size()) {
                states.flush();
                std::cout << "[video] " << i+1 << '/' << frames.size()
                          << " state=" << slam.GetTrackingState() << std::endl;
            }
            // Give mapping and loop-closing threads time even on fast hardware.
            if (realtime && i+1 < frames.size()) {
                const double delay = frames[i+1].time-frame.time-seconds;
                if (delay > 0) std::this_thread::sleep_for(std::chrono::duration<double>(delay));
            }
        }
        states.close();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        slam.Shutdown();
        // The upstream Shutdown currently does not join workers: this method waits first.
        slam.ExportMonocularReplay(output);
        std::cout << "[video] Export complete: " << output << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[video] " << error.what() << '\n';
        return 1;
    }
}
