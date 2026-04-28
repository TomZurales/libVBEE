#pragma once
#include <map>
#include <mutex>
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
        bool mDeltaCacheDirty = true;
    public:
        Manager();
        ~Manager() = default;

        std::unordered_set<unsigned long> update(const std::vector<Observation>& observations, float th_delete = 0.1f);
        void AlertMerge(unsigned long from, unsigned long to);
        std::map<unsigned long, float> getAllPE();
        std::map<unsigned long, float> getAllDeltas();
    };
}