#include "vbee_manager.h"
#include "observability_models/discrete_boundary.h"

#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <map>
#include <memory>
#include <unordered_set>
#include <vector>

std::unordered_set<unsigned long> VBEE::Manager::update(const std::vector<Observation>& observations, float th_delete)
{
    std::cout << "Updating VBEE with " << observations.size() << " observations." << std::endl;

    std::vector<std::pair<unsigned long, unsigned long>> merges;
    {
        std::lock_guard<std::mutex> lock(mMergeMutex);
        merges.swap(pendingMerges);
    }

    std::map<unsigned long, std::vector<Observation>> by_mp;
    for(const auto& obs : observations)
        by_mp[obs.mpId].push_back(obs);

    {
        std::lock_guard<std::mutex> lock(mInstancesMutex);

        mPrevPESnapshot.clear();
        mCurrentUpdateIds.clear();
        for(const auto& [mpId, obsList] : by_mp)
        {
            mCurrentUpdateIds.push_back(mpId);
            if(mpInstances.count(mpId) > 0)
                mPrevPESnapshot[mpId] = mpInstances[mpId].getPe();
            else
                mpInstances[mpId] = Instance(std::make_shared<DiscreteBoundary>(5, 0.5f), 0.01, 0.001, 10.0f);
            mpInstances[mpId].step(obsList);
        }
        for(const auto& [from, to] : merges)
        {
            if(mpInstances.count(from) > 0 && mpInstances.count(to) > 0)
            {
                mpInstances[to].merge(mpInstances[from]);
                mpInstances.erase(from);
            }
        }

        mPECache.clear();
        std::unordered_set<unsigned long> toDelete;
        std::vector<int> bins(20, 0);
        for(const auto& [id, instance] : mpInstances)
        {
            float pe = instance.getPe();
            mPECache[id] = pe;
            if(pe < th_delete)
            {
                toDelete.insert(id);
            }
            int bin = static_cast<int>((1.0f - pe) / 0.05f);
            if(bin > 19) bin = 19;
            ++bins[bin];
        }
        for(unsigned long id : toDelete)
            mpInstances.erase(id);
        bins.push_back(toDelete.size());
        bins.push_back(merges.size());
        vbee_stats.push_back(bins);
        mDeltaCacheDirty = true;

        return toDelete;
    }
}

// Ransac weighting functions
float VBEE::Manager::getPE(unsigned long mpId)
{
    std::lock_guard<std::mutex> lock(mInstancesMutex);
    auto it = mpInstances.find(mpId);
    if(it != mpInstances.end())
        return it->second.getPe();
    return 0.9f;
}

float VBEE::Manager::getPS(unsigned long mpId, const Eigen::Vector3f& position)
{
    std::lock_guard<std::mutex> lock(mInstancesMutex);
    auto it = mpInstances.find(mpId);
    if(it != mpInstances.end())
    {
        float psge = it->second.query(position);
        float pe = it->second.getPe();

        return (psge * pe) + ((1.0f - pe) * 0.001f); // P(S) calculated from P(S|E) and P(E)
    }
    return 0.5f;
}

void VBEE::Manager::AlertMerge(unsigned long from, unsigned long to)
{
    std::lock_guard<std::mutex> lock(mMergeMutex);
    pendingMerges.emplace_back(from, to);
}

// Diagnostic functions
std::map<unsigned long, float> VBEE::Manager::getAllDeltas()
{
    std::lock_guard<std::mutex> lock(mInstancesMutex);
    if(mDeltaCacheDirty)
    {
        mDeltaCache.clear();
        for(unsigned long id : mCurrentUpdateIds)
        {
            auto prevIt = mPrevPESnapshot.find(id);
            auto instIt = mpInstances.find(id);
            if(prevIt != mPrevPESnapshot.end() && instIt != mpInstances.end())
                mDeltaCache[id] = instIt->second.getPe() - prevIt->second;
        }
        mDeltaCacheDirty = false;
    }
    return mDeltaCache;
}

std::map<unsigned long, float> VBEE::Manager::getAllPE()
{
    std::lock_guard<std::mutex> lock(mInstancesMutex);
    return mPECache;
}

