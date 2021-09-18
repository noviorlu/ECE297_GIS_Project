//
// Created by cheny811 on 2021-03-27.
//
#include "m3.h"
#include "m1.h"
#include "NaviHelper.h"
#include <queue>

// Seperate from major Dyjstra Method
// Can be run only SegsInfoList is loaded
double computePathTravelTime(const std::vector<StreetSegmentIdx>& path, const double turn_penalty){
    double totalTurnPenalty = 0;
    if(path.size() > 2){

        // Calculate all turning Penalties
        for(int i=0 ; i < path.size()-1 ; i++ ){
            
            // determined by different streetID(of streetSegment)
            if(SegsInfoList[path[i]].segInfo.streetID != SegsInfoList[path[i+1]].segInfo.streetID){
                totalTurnPenalty+=turn_penalty;
            }
        }
    }
    // Sum Segment travelTime
    double totalTraveTime = 0;
    for(auto curStSegId : path){
        totalTraveTime += SegsInfoList[curStSegId].time;
    }

    return totalTraveTime + totalTurnPenalty;
}

bool NaviInfoHelper(const IntersectionIdx intersect_id_start,
                    const IntersectionIdx intersect_id_destination,
                    std::vector<IntersectNaviInfo>& IntersectNaviInfoList,
                    const double turn_penalty);

std::vector<StreetSegmentIdx> findPathBetweenIntersections(
        const IntersectionIdx intersect_id_start,
        const IntersectionIdx intersect_id_destination,
        const double turn_penalty){

    std::vector<IntersectNaviInfo> IntersectNaviInfoList(getNumIntersections());

    std::vector<StreetSegmentIdx> path;
    bool pathExist = NaviInfoHelper(intersect_id_start, intersect_id_destination, IntersectNaviInfoList, turn_penalty);
    if(pathExist) path = backTracing(intersect_id_start, intersect_id_destination, IntersectNaviInfoList);

    return path;
}

// Dijstra + AStar method, using waveFont techique
bool NaviInfoHelper(
        const IntersectionIdx intersect_id_start,
        const IntersectionIdx intersect_id_destination,
        std::vector<IntersectNaviInfo>& IntersectNaviInfoList,
        const double turn_penalty){
    
    // Lambda function used for pirorty queue comparsion
    // proceed faster route infront
    auto cmp = [](WaveElem lhs, WaveElem rhs){
        return (lhs.travelTime) > (rhs.travelTime);
    };
    std::priority_queue<WaveElem, std::vector<WaveElem>, decltype(cmp)> WaveFront(cmp);

    // Push the Starting point into WaveFont to deal
    WaveFront.push(WaveElem(intersect_id_start, -1, 0));

    // Start to Wave Out until it reaches all possible vertices
    while(!WaveFront.empty()){
        WaveElem currWave = WaveFront.top();
        WaveFront.pop();

        IntersectionIdx currIntersectId = currWave.IntersectId;

        //Determine if there's better route to reach this node, check next if not
        //Since it is been sorted by Priority queue, case "return without finding
        //the shortest edge cloest to Destination" will not occur
        if(currWave.travelTime < IntersectNaviInfoList[currIntersectId].bestTime){
            
            // Renew information saved in temperary storage Graph(represented by vector)
            IntersectNaviInfoList[currIntersectId].reachingEdge = currWave.reachingEdge;
            IntersectNaviInfoList[currIntersectId].bestTime = currWave.travelTime;

            // if Find destination then return
            if(currIntersectId == intersect_id_destination){
                return true;
            }

            // m1 Function returns edge of current node
            auto tempStSegsList = findStreetSegmentsOfIntersection(currIntersectId);

            // each outEdge of currNode
            for(auto currStSegsId : tempStSegsList){
                //m1 structure geting all Info of current Segment
                //used for Legality Check
                const StrSeg_Info & curSegInfo = SegsInfoList[currStSegsId];

                IntersectionIdx toIntersect;

                // Check From and To & ONEWAY
                if(curSegInfo.segInfo.oneWay){
                    if(curSegInfo.segInfo.to == currIntersectId){
                        continue;
                    }else{
                        toIntersect = curSegInfo.segInfo.to;
                    }
                }else{
                    if(curSegInfo.segInfo.to == currIntersectId){
                        toIntersect = curSegInfo.segInfo.from;
                    }else{
                        toIntersect = curSegInfo.segInfo.to;
                    }
                }
                // Travel back to an Node been waveFonted needs to jump(cycle in graph)
                if(IntersectNaviInfoList[toIntersect].isTravel) continue;

                // New Node hasnot been traveled, need to push an New WaveElem 
                double curTravelTime;
                // If this is a turning edge
                if(currIntersectId != intersect_id_start &&
                SegsInfoList[currStSegsId].segInfo.streetID !=
                SegsInfoList[IntersectNaviInfoList[currIntersectId].reachingEdge].segInfo.streetID){
                    curTravelTime = IntersectNaviInfoList[currIntersectId].bestTime + curSegInfo.time + turn_penalty;
                    WaveFront.push(WaveElem(toIntersect, currStSegsId, curTravelTime));
                }//Not an turning edge
                else{
                    curTravelTime = IntersectNaviInfoList[currIntersectId].bestTime + curSegInfo.time;
                    WaveFront.push(WaveElem(toIntersect, currStSegsId, curTravelTime));
                }
            }//set Node is Traveled
            IntersectNaviInfoList[currIntersectId].isTravel = true;
        }
    }
    return false;
}

std::vector<StreetSegmentIdx> backTracing(const IntersectionIdx intersect_id_start,
                                          const IntersectionIdx intersect_id_destination,
                                          const std::vector<IntersectNaviInfo>& IntersectNaviInfoList){
    
    // Result saved in Path, don't need to Reverse whole list when finish
    // Maybe Stack is better for use
    std::deque<StreetSegmentIdx> path;

    // Destination "TO" Intersection
    auto curIntersectId = intersect_id_destination;

    auto previousEdge = IntersectNaviInfoList[curIntersectId].reachingEdge;

    while(previousEdge != -1){
        path.push_front(previousEdge);

        // Check the "FROM" Intersection
        auto fromIntersect = SegsInfoList[previousEdge].segInfo.from;
        auto toIntersect = SegsInfoList[previousEdge].segInfo.to;

        if(toIntersect == curIntersectId){
            curIntersectId = fromIntersect;
        }else{
            curIntersectId = toIntersect;
        }

        // check "FROM" Intersection's reachingEdge
        previousEdge = IntersectNaviInfoList[curIntersectId].reachingEdge;
    }

    return std::vector<StreetSegmentIdx>(path.begin(), path.end());
}