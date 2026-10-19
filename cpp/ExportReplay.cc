// SPDX-License-Identifier: GPL-3.0-or-later
// Compiled into the upstream System target by scripts/patch_upstream.py.
#include "System.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <stdexcept>
#include <tuple>

namespace ORB_SLAM3 {
namespace {
std::ofstream csv(const std::string& path, const std::string& header) {
    std::ofstream file(path);
    if (!file) throw std::runtime_error("Cannot write " + path);
    file << header << '\n' << std::setprecision(17);
    return file;
}
void poseValues(std::ostream& out, const Sophus::SE3f& Tcw) {
    const auto q = Tcw.unit_quaternion();
    const auto t = Tcw.translation();
    out << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z()
        << ',' << t.x() << ',' << t.y() << ',' << t.z();
}
}

void System::ExportMonocularReplay(const std::string& directory) {
    if (mSensor != MONOCULAR || !isShutDown())
        throw std::runtime_error("Call Shutdown on a pure monocular system before exporting");
    const auto deadline = std::chrono::steady_clock::now()+std::chrono::minutes(10);
    while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA()) {
        if (std::chrono::steady_clock::now() > deadline)
            throw std::runtime_error("Workers did not stop within 10 min; no inconsistent snapshot exported");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (mptLocalMapping && mptLocalMapping->joinable()) mptLocalMapping->join();
    if (mptLoopClosing && mptLoopClosing->joinable()) mptLoopClosing->join();

    auto mapsFile = csv(directory+"/maps.csv", "map_id,keyframes,points");
    auto keysFile = csv(directory+"/keyframes.csv", "id,map_id,timestamp,qw,qx,qy,qz,tx,ty,tz");
    auto pointsFile = csv(directory+"/points.csv", "id,map_id,x,y,z,birth_time,observations");
    auto obsFile = csv(directory+"/observations.csv", "point_id,map_id,timestamp,u,v");
    auto posesFile = csv(directory+"/poses.csv", "timestamp,map_id,qw,qx,qy,qz,tx,ty,tz,lost");
    std::set<Map*> validMaps;
    auto maps = mpAtlas->GetAllMaps();
    std::sort(maps.begin(), maps.end(), [](Map* a, Map* b) { return a->GetId() < b->GetId(); });
    for (Map* map : maps) {
        if (!map || map->IsBad()) continue;
        validMaps.insert(map);
        size_t nKeys = 0, nPoints = 0;
        auto keys = map->GetAllKeyFrames();
        std::sort(keys.begin(), keys.end(), KeyFrame::lId);
        for (KeyFrame* key : keys) {
            if (!key || key->isBad()) continue;
            keysFile << key->mnId << ',' << map->GetId() << ',' << key->mTimeStamp << ',';
            poseValues(keysFile, key->GetPose()); keysFile << '\n';
            ++nKeys;
        }
        auto points = map->GetAllMapPoints();
        std::sort(points.begin(), points.end(), [](MapPoint* a, MapPoint* b) { return a->mnId < b->mnId; });
        for (MapPoint* point : points) {
            if (!point || point->isBad()) continue;
            const Eigen::Vector3f pos = point->GetWorldPos();
            if (!pos.allFinite()) continue;
            std::vector<std::tuple<double, float, float>> observations;
            for (const auto& observation : point->GetObservations()) {
                KeyFrame* key = observation.first;
                const int index = std::get<0>(observation.second);
                if (!key || key->isBad() || key->GetMap() != map || index < 0 ||
                    static_cast<size_t>(index) >= key->mvKeys.size()) continue;
                const auto pixel = key->mvKeys[index].pt; // ORIGINAL distorted coordinates
                observations.emplace_back(key->mTimeStamp, pixel.x, pixel.y);
            }
            if (observations.empty()) continue;
            std::sort(observations.begin(), observations.end());
            pointsFile << point->mnId << ',' << map->GetId() << ',' << pos.x() << ',' << pos.y()
                       << ',' << pos.z() << ',' << std::get<0>(observations.front())
                       << ',' << observations.size() << '\n';
            // Up to three retained keyframe images supply persistent RGB colors offline.
            for (size_t k = 0; k < std::min<size_t>(3, observations.size()); ++k) {
                const auto& obs = observations[k];
                obsFile << point->mnId << ',' << map->GetId() << ',' << std::get<0>(obs)
                        << ',' << std::get<1>(obs) << ',' << std::get<2>(obs) << '\n';
            }
            ++nPoints;
        }
        mapsFile << map->GetId() << ',' << nKeys << ',' << nPoints << '\n';
    }
    const size_t size = mpTracker->mlRelativeFramePoses.size();
    if (mpTracker->mlpReferences.size() != size || mpTracker->mlFrameTimes.size() != size ||
        mpTracker->mlbLost.size() != size) throw std::runtime_error("Inconsistent tracker history sizes");
    auto ref = mpTracker->mlpReferences.begin();
    auto stamp = mpTracker->mlFrameTimes.begin();
    auto lost = mpTracker->mlbLost.begin();
    for (auto relative = mpTracker->mlRelativeFramePoses.begin();
         relative != mpTracker->mlRelativeFramePoses.end(); ++relative, ++ref, ++stamp, ++lost) {
        // Reset/lost records may refer to discarded keyframes: skip before dereferencing.
        if (*lost) continue;
        KeyFrame* key = *ref;
        Sophus::SE3f chain;
        std::set<KeyFrame*> visited;
        while (key && key->isBad()) {
            if (!visited.insert(key).second) throw std::runtime_error("Cyclic culled-keyframe parent tree");
            chain = chain*key->mTcp;
            key = key->GetParent();
        }
        if (*lost || !key || !validMaps.count(key->GetMap())) continue;
        // Keep the ORIGINAL map reference for BOTH map points and camera poses.
        // A separate first-keyframe-origin normalization here would misalign them.
        const Sophus::SE3f Tcw = (*relative)*chain*key->GetPose();
        if (!Tcw.matrix().allFinite()) continue;
        posesFile << *stamp << ',' << key->GetMap()->GetId() << ',';
        poseValues(posesFile, Tcw); posesFile << ",0\n";
    }
}
}
