#pragma once
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include "observation.h"
#include "instance.h"

namespace VBEE
{
    class Manager
    {
    private:
        std::map<unsigned long, Instance> mpInstances;
        std::vector<std::pair<unsigned long, unsigned long>> pendingMerges;
        std::mutex mMergeMutex;
        std::mutex mInstancesMutex;
        std::map<unsigned long, float> mPECache;
        std::map<unsigned long, float> mPrevPESnapshot;
        std::vector<unsigned long> mCurrentUpdateIds;
        std::map<unsigned long, float> mDeltaCache;
        std::vector<int> ransac_iteration_counts;
        std::vector<std::vector<int>> vbee_stats;
        bool mDeltaCacheDirty = true;

        bool inUse;
        bool weightRansac;
    public:
        Manager(bool inUse, bool weightRansac) : inUse(inUse), weightRansac(weightRansac) {};
        ~Manager() = default;

        std::unordered_set<unsigned long> update(const std::vector<Observation>& observations, float th_delete = 0.1f);
        void AlertMerge(unsigned long from, unsigned long to);
        std::map<unsigned long, float> getAllPE();
        std::map<unsigned long, float> getAllDeltas();
        float getPE(unsigned long mpId);
        float getPS(unsigned long mpId, const Eigen::Vector3f& position);

        // Stats collection functions
        void addRansacIterationsCount(int count)
        {
            ransac_iteration_counts.push_back(count);
        }

        void saveVBEEStats(const std::string& filepath)
        {
            std::ofstream f(filepath);
            for (size_t i = 0; i < ransac_iteration_counts.size(); ++i)
            {
                if (i > 0) f << ",";
                f << ransac_iteration_counts[i];
            }
            f << "\n";
            for (const auto& row : vbee_stats)
            {
                for (size_t i = 0; i < row.size(); ++i)
                {
                    if (i > 0) f << ",";
                    f << row[i];
                }
                f << "\n";
            }
        }

        bool isInUse() const { return inUse; }
        bool isWeightRansac() const { return weightRansac; }
    };
}