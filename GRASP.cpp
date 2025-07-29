#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <random>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <unordered_set>
#include <queue>
#include <map>
#include <unordered_map>
#include <cstdlib>

using namespace std;

// Structure to represent a customer with their details
struct Customer {
    int id;
    double x, y;
    int demand;
    double serviceTime;
    double earliest;
    double latest;
};

// Structure to represent a route with customer IDs, distances, and times
struct Route {
    vector<int> customerIds;
    double totalDistance;
    vector<double> arrivalTimes;
    vector<double> departureTimes;
    int load; // Total demand of customers in the route
    Route() : totalDistance(0.0), load(0) {}
};

// Structure of problem data
struct ProblemData {
    vector<Customer> customers;
    int vehicleCapacity;
    int maxVehicles;
    double maxRouteDuration;
    vector<vector<double>> distanceMatrix; // Precomputed distance matrix
    unordered_map<int, int> idToIndex; // Map of customer ID to index in customers vector
};

// Reads problem instance and precomputes distances
ProblemData readInstance(const string &filename) {
    ProblemData data;
    ifstream infile(filename);
    if (!infile) {
        cerr << "Cannot open file: " << filename << endl;
        exit(1);
    }
    string line;

    while (getline(infile, line)) {
        if (line.find("CUST NO.") != string::npos) {
            getline(infile, line); // Skip the empty line
            break;
        } else if (line.find("NUMBER") != string::npos) {
            getline(infile, line);
            istringstream iss(line);
            iss >> data.maxVehicles >> data.vehicleCapacity;
        }
    }

    while (getline(infile, line)) {
        istringstream issCust(line);
        Customer cust;
        if (issCust >> cust.id >> cust.x >> cust.y >> cust.demand >> cust.earliest >> cust.latest >> cust.serviceTime) {
            data.customers.push_back(cust);
        } else {
            break;
        }
    }
    data.maxRouteDuration = data.customers[0].latest;

    for (size_t i = 0; i < data.customers.size(); ++i) {
        data.idToIndex[data.customers[i].id] = i;
    }

    size_t n = data.customers.size();
    data.distanceMatrix.resize(n, vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (i == j) {
                data.distanceMatrix[i][j] = 0.0;
            } else {
                double dx = data.customers[i].x - data.customers[j].x;
                double dy = data.customers[i].y - data.customers[j].y;
                data.distanceMatrix[i][j] = sqrt(dx * dx + dy * dy);
            }
        }
    }

    infile.close();
    return data;
}

bool isRouteFeasible(const Route &route, const ProblemData &data) {
    int capacityUsed = 0;
    double currentTime = 0.0;
    int currentIndex = 0; // Start at depot

    for (int custId : route.customerIds) {
        int index = data.idToIndex.at(custId);
        capacityUsed += data.customers[index].demand;
        if (capacityUsed > data.vehicleCapacity) return false;

        double travelTime = data.distanceMatrix[currentIndex][index];
        double arrivalTime = currentTime + travelTime;
        double serviceStartTime = max(arrivalTime, data.customers[index].earliest);
        if (serviceStartTime > data.customers[index].latest) return false;
        currentTime = serviceStartTime + data.customers[index].serviceTime;
        currentIndex = index;
    }
    double returnTravelTime = data.distanceMatrix[currentIndex][0];
    currentTime += returnTravelTime;
    if (currentTime > data.maxRouteDuration) return false;
    return true;
}

bool isSolutionFeasible(const vector<Route>& routes, const ProblemData& data) {
    for (const auto& route : routes) {
        if (!isRouteFeasible(route, data)) return false;
    }
    vector<bool> visited(data.customers.size(), false);
    visited[0] = true;
    for (const auto& route : routes) {
        for (int custId : route.customerIds) {
            int index = data.idToIndex.at(custId);
            if (visited[index]) return false;
            visited[index] = true;
        }
    }
    for (size_t i = 1; i < visited.size(); ++i) {
        if (!visited[i]) return false;
    }
    return true;
}

void updateRoute(Route& route, const ProblemData& data) {
    route.arrivalTimes.clear();
    route.departureTimes.clear();
    route.load = 0;
    route.totalDistance = 0;
    if (route.customerIds.empty()) return;

    double currentTime = 0.0;
    route.arrivalTimes.push_back(currentTime);
    int currentIndex = 0;

    for (int custId : route.customerIds) {
        int index = data.idToIndex.at(custId);
        double travelTime = data.distanceMatrix[currentIndex][index];
        currentTime += travelTime;
        route.totalDistance += travelTime;
        if (currentTime < data.customers[index].earliest) currentTime = data.customers[index].earliest;
        route.arrivalTimes.push_back(currentTime);
        currentTime += data.customers[index].serviceTime;
        route.departureTimes.push_back(currentTime);
        route.load += data.customers[index].demand;
        currentIndex = index;
    }
    double returnDist = data.distanceMatrix[currentIndex][0];
    route.totalDistance += returnDist;
    currentTime += returnDist;
    route.arrivalTimes.push_back(currentTime);
    route.departureTimes.push_back(currentTime);
}

vector<Route> constructInitialSolution(const ProblemData &data, mt19937& rng, double alpha, int RCL_size) {
    vector<Route> routes;
    vector<bool> visited(data.customers.size(), false);
    visited[0] = true; // Depot is visited
    while (find(visited.begin() + 1, visited.end(), false) != visited.end()) {
        Route route;
        route.arrivalTimes.push_back(0.0);
        route.departureTimes.push_back(0.0);
        double currentTime = 0.0;
        int currentLoad = 0;
        while (true) {
            vector<pair<int, pair<size_t, double>>> candidates; // {customer_index, {best_pos, min_cost}}
            for (size_t i = 1; i < data.customers.size(); i++) {
                if (!visited[i]) {
                    int custId = data.customers[i].id;
                    int index = data.idToIndex.at(custId);
                    double minCost = numeric_limits<double>::max();
                    size_t bestPos = 0;
                    // Check all possible insertion positions
                    for (size_t pos = 0; pos <= route.customerIds.size(); pos++) {
                        Route tempRoute = route;
                        tempRoute.customerIds.insert(tempRoute.customerIds.begin() + pos, custId);
                        updateRoute(tempRoute, data);
                        if (isRouteFeasible(tempRoute, data)) {
                            // Calculate insertion cost (additional distance)
                            double cost = 0.0;
                            if (pos == 0) {
                                // Insert at start: depot -> customer
                                cost += data.distanceMatrix[0][index];
                                if (!route.customerIds.empty()) {
                                    // If there’s a next customer
                                    int nextId = route.customerIds[0];
                                    cost += data.distanceMatrix[index][data.idToIndex.at(nextId)];
                                    cost -= data.distanceMatrix[0][data.idToIndex.at(nextId)];
                                } else {
                                    // Return to depot
                                    cost += data.distanceMatrix[index][0];}
                            } else {
                                // Insert between customers or at end
                                int prevId = route.customerIds[pos - 1];
                                cost += data.distanceMatrix[data.idToIndex.at(prevId)][index];
                                if (pos < route.customerIds.size()) {
                                    int nextId = route.customerIds[pos];
                                    cost += data.distanceMatrix[index][data.idToIndex.at(nextId)];
                                    cost -= data.distanceMatrix[data.idToIndex.at(prevId)][data.idToIndex.at(nextId)];
                                } else {
                                    // Insert at end: customer -> depot
                                    cost += data.distanceMatrix[index][0];
                                    cost -= data.distanceMatrix[data.idToIndex.at(prevId)][0];}}
                            if (cost < minCost) {
                                minCost = cost;
                                bestPos = pos;}}}
                    if (minCost != numeric_limits<double>::max()) {
                        candidates.push_back({static_cast<int>(i), {bestPos, minCost}});}}}
            if (candidates.empty()) break;
            // Sort candidates by insertion cost
            sort(candidates.begin(), candidates.end(),
                 [](const auto& a, const auto& b) { return a.second.second < b.second.second; });
            // Build RCL
            double minCost = candidates[0].second.second;
            double maxCost = candidates.back().second.second;
            double threshold = minCost + alpha * (maxCost - minCost);
            vector<pair<int, pair<size_t, double>>> alphaRcl;
            for (const auto& cand : candidates) {
                if (cand.second.second <= threshold) {
                    alphaRcl.push_back(cand);}}
            if (alphaRcl.empty()) break;
            size_t numCandidates = min<size_t>(RCL_size, alphaRcl.size());
            vector<pair<int, pair<size_t, double>>> finalRcl(alphaRcl.begin(), alphaRcl.begin() + numCandidates);
            uniform_int_distribution<size_t> dist(0, finalRcl.size() - 1);
            auto selected = finalRcl[dist(rng)];
            int nextCustomer = selected.first;
            size_t insertPos = selected.second.first;
            // Insert customer at best position
            route.customerIds.insert(route.customerIds.begin() + insertPos, data.customers[nextCustomer].id);
            updateRoute(route, data);
            visited[nextCustomer] = true;
            currentLoad = route.load;
            currentTime = route.departureTimes.back();}
        if (!route.customerIds.empty()) {
            routes.push_back(route);}}
    return routes;}

pair<int, double> objectiveFunction(const vector<Route> &routes, long long &evalCount) {
    evalCount++;
    int vehicles = 0;
    double totalDistance = 0.0;
    for (const auto &route : routes) {
        if (!route.customerIds.empty()) {
            vehicles++;
            totalDistance += route.totalDistance;
        }
    }
    return {vehicles, totalDistance};
}

bool BetterSolution(const vector<Route> &newRoutes, const vector<Route> &currentRoutes, long long &evalCount) {
    auto [vehiclesNew, distNew] = objectiveFunction(newRoutes, evalCount);
    auto [vehiclesCurrent, distCurrent] = objectiveFunction(currentRoutes, evalCount);
    if (vehiclesNew < vehiclesCurrent) return true;
    if (vehiclesNew > vehiclesCurrent) return false;
    return distNew < distCurrent;
}

void removeRouteMove(vector<Route>& routes, mt19937& rng, const ProblemData& data) {
    if (routes.size() <= 1) return;
    uniform_int_distribution<size_t> routeDist(0, routes.size() - 1);
    size_t routeIdx = routeDist(rng);
    while (routes[routeIdx].customerIds.empty()) routeIdx = routeDist(rng);

    vector<int> customersToRedistribute = routes[routeIdx].customerIds;
    routes.erase(routes.begin() + routeIdx);

    for (int customer : customersToRedistribute) {
        bool inserted = false;
        vector<size_t> routeIndices(routes.size());
        iota(routeIndices.begin(), routeIndices.end(), 0);
        shuffle(routeIndices.begin(), routeIndices.end(), rng);

        for (size_t r : routeIndices) {
            vector<size_t> positions(routes[r].customerIds.size() + 1);
            iota(positions.begin(), positions.end(), 0);
            shuffle(positions.begin(), positions.end(), rng);
            for (size_t pos : positions) {
                routes[r].customerIds.insert(routes[r].customerIds.begin() + pos, customer);
                updateRoute(routes[r], data);
                if (isRouteFeasible(routes[r], data)) {
                    inserted = true;
                    break;
                }
                routes[r].customerIds.erase(routes[r].customerIds.begin() + pos);
                updateRoute(routes[r], data);
            }
            if (inserted) break;
        }
        if (!inserted) {
            Route newRoute;
            newRoute.customerIds.push_back(customer);
            updateRoute(newRoute, data);
            if (isRouteFeasible(newRoute, data)) routes.push_back(newRoute);
        }
    }
    routes.erase(remove_if(routes.begin(), routes.end(),
                           [](const Route& r) { return r.customerIds.empty(); }),
                 routes.end());
}

void relocateCustomer(vector<Route>& routes, mt19937& rng, const ProblemData& data) {
    if (routes.size() < 2) return;
    uniform_int_distribution<size_t> routeDist(0, routes.size() - 1);
    size_t sourceRouteIdx = routeDist(rng);
    while (routes[sourceRouteIdx].customerIds.empty()) sourceRouteIdx = routeDist(rng);

    uniform_int_distribution<size_t> custDist(0, routes[sourceRouteIdx].customerIds.size() - 1);
    size_t custPos = custDist(rng);
    int customer = routes[sourceRouteIdx].customerIds[custPos];

    routes[sourceRouteIdx].customerIds.erase(routes[sourceRouteIdx].customerIds.begin() + custPos);
    updateRoute(routes[sourceRouteIdx], data);

    bool inserted = false;
    vector<size_t> routeIndices(routes.size());
    iota(routeIndices.begin(), routeIndices.end(), 0);
    shuffle(routeIndices.begin(), routeIndices.end(), rng);

    for (size_t r : routeIndices) {
        if (r == sourceRouteIdx) continue;
        vector<size_t> positions(routes[r].customerIds.size() + 1);
        iota(positions.begin(), positions.end(), 0);
        shuffle(positions.begin(), positions.end(), rng);
        for (size_t pos : positions) {
            routes[r].customerIds.insert(routes[r].customerIds.begin() + pos, customer);
            updateRoute(routes[r], data);
            if (isRouteFeasible(routes[r], data)) {
                inserted = true;
                break;
            }
            routes[r].customerIds.erase(routes[r].customerIds.begin() + pos);
            updateRoute(routes[r], data);
        }
        if (inserted) break;
    }
    if (!inserted) {
        Route newRoute;
        newRoute.customerIds.push_back(customer);
        updateRoute(newRoute, data);
        if (isRouteFeasible(newRoute, data)) routes.push_back(newRoute);
    }
    routes.erase(remove_if(routes.begin(), routes.end(),
                           [](const Route& r) { return r.customerIds.empty(); }),
                 routes.end());
}

void twoOpt(vector<Route>& routes, mt19937& rng, const ProblemData& data) {
    if (routes.empty()) return;

    vector<size_t> validRoutes;
    for (size_t idx = 0; idx < routes.size(); ++idx) {
        if (routes[idx].customerIds.size() >= 2) {
            validRoutes.push_back(idx);
        }
    }
    if (validRoutes.empty()) return;

    uniform_int_distribution<size_t> routeDist(0, validRoutes.size() - 1);
    size_t routeIdx = validRoutes[routeDist(rng)];

    Route& route = routes[routeIdx];
    size_t n = route.customerIds.size();

    uniform_int_distribution<size_t> posDist(0, n - 1);
    size_t i = posDist(rng);
    size_t j = posDist(rng);
    size_t attempts = 0;
    const size_t maxAttempts = 100;
    while ((i == j || abs(static_cast<int>(i) - static_cast<int>(j)) < 2) && attempts < maxAttempts) {
        i = posDist(rng);
        j = posDist(rng);
        attempts++;
    }
    if (attempts >= maxAttempts) return;

    if (i > j) swap(i, j);
    vector<int> originalCustomers = route.customerIds;
    reverse(route.customerIds.begin() + i, route.customerIds.begin() + j + 1);
    updateRoute(route, data);

    if (!isRouteFeasible(route, data)) {
        route.customerIds = originalCustomers;
        updateRoute(route, data);
    }
}

void crossExchange(vector<Route>& routes, mt19937& rng, const ProblemData& data) {
    if (routes.size() < 2) return;

    vector<size_t> validRoutes;
    for (size_t idx = 0; idx < routes.size(); ++idx) {
        if (routes[idx].customerIds.size() >= 1) {
            validRoutes.push_back(idx);
        }
    }
    if (validRoutes.size() < 2) return;

    uniform_int_distribution<size_t> routeDist(0, validRoutes.size() - 1);
    size_t routeIdx1 = routeDist(rng);
    size_t routeIdx2 = routeDist(rng);
    size_t attempts = 0;
    const size_t maxAttempts = 100;
    while (routeIdx1 == routeIdx2 && attempts < maxAttempts) {
        routeIdx2 = routeDist(rng);
        attempts++;
    }
    if (attempts >= maxAttempts) return;

    Route& route1 = routes[validRoutes[routeIdx1]];
    Route& route2 = routes[validRoutes[routeIdx2]];

    size_t len1 = route1.customerIds.size();
    size_t len2 = route2.customerIds.size();
    if (len1 < 1 || len2 < 1) return;

    uniform_int_distribution<size_t> posDist1(0, len1 - 1);
    uniform_int_distribution<size_t> posDist2(0, len2 - 1);
    size_t start1 = posDist1(rng);
    size_t start2 = posDist2(rng);

    size_t maxSegmentLen1 = min<size_t>(len1 - start1, len1 / 2);
    size_t maxSegmentLen2 = min<size_t>(len2 - start2, len2 / 2);
    if (maxSegmentLen1 < 1 || maxSegmentLen2 < 1) return;

    uniform_int_distribution<size_t> lenDist1(1, maxSegmentLen1);
    uniform_int_distribution<size_t> lenDist2(1, maxSegmentLen2);
    size_t segmentLen1 = lenDist1(rng);
    size_t segmentLen2 = lenDist2(rng);

    vector<int> segment1(route1.customerIds.begin() + start1, route1.customerIds.begin() + start1 + segmentLen1);
    vector<int> segment2(route2.customerIds.begin() + start2, route2.customerIds.begin() + start2 + segmentLen2);

    vector<int> originalRoute1 = route1.customerIds;
    vector<int> originalRoute2 = route2.customerIds;

    route1.customerIds.erase(route1.customerIds.begin() + start1, route1.customerIds.begin() + start1 + segmentLen1);
    route2.customerIds.erase(route2.customerIds.begin() + start2, route2.customerIds.begin() + start2 + segmentLen2);

    route1.customerIds.insert(route1.customerIds.begin() + start1, segment2.begin(), segment2.end());
    route2.customerIds.insert(route2.customerIds.begin() + start2, segment1.begin(), segment1.end());

    updateRoute(route1, data);
    updateRoute(route2, data);

    if (!isRouteFeasible(route1, data) || !isRouteFeasible(route2, data)) {
        route1.customerIds = originalRoute1;
        route2.customerIds = originalRoute2;
        updateRoute(route1, data);
        updateRoute(route2, data);
    }
}

void mergeRoute(vector<Route>& routes, mt19937& rng, const ProblemData& data) {
    if (routes.size() < 2) return;

    vector<size_t> validRoutes;
    for (size_t idx = 0; idx < routes.size(); ++idx) {
        if (!routes[idx].customerIds.empty()) {
            validRoutes.push_back(idx);
        }
    }
    if (validRoutes.size() < 2) return;

    uniform_int_distribution<size_t> routeDist(0, validRoutes.size() - 1);
    size_t routeIdx1 = routeDist(rng);
    size_t routeIdx2 = routeDist(rng);
    size_t attempts = 0;
    const size_t maxAttempts = 100;
    while (routeIdx1 == routeIdx2 && attempts < maxAttempts) {
        routeIdx2 = routeDist(rng);
        attempts++;
    }
    if (attempts >= maxAttempts) return;

    size_t idx1 = validRoutes[routeIdx1];
    size_t idx2 = validRoutes[routeIdx2];
    if (idx2 < idx1) swap(idx1, idx2);

    Route mergedRoute;
    mergedRoute.customerIds = routes[idx1].customerIds;
    mergedRoute.customerIds.insert(mergedRoute.customerIds.end(),
                                   routes[idx2].customerIds.begin(),
                                   routes[idx2].customerIds.end());
    updateRoute(mergedRoute, data);

    routes.erase(routes.begin() + idx2);
    routes.erase(routes.begin() + idx1);

    if (!isRouteFeasible(mergedRoute, data)) {
        vector<int> customersToRedistribute = mergedRoute.customerIds;
        for (int customer : customersToRedistribute) {
            bool inserted = false;
            vector<size_t> routeIndices(routes.size());
            iota(routeIndices.begin(), routeIndices.end(), 0);
            shuffle(routeIndices.begin(), routeIndices.end(), rng);

            for (size_t r : routeIndices) {
                vector<size_t> positions(routes[r].customerIds.size() + 1);
                iota(positions.begin(), positions.end(), 0);
                shuffle(positions.begin(), positions.end(), rng);
                for (size_t pos : positions) {
                    routes[r].customerIds.insert(routes[r].customerIds.begin() + pos, customer);
                    updateRoute(routes[r], data);
                    if (isRouteFeasible(routes[r], data)) {
                        inserted = true;
                        break;
                    }
                    routes[r].customerIds.erase(routes[r].customerIds.begin() + pos);
                    updateRoute(routes[r], data);
                }
                if (inserted) break;
            }
            if (!inserted) {
                Route newRoute;
                newRoute.customerIds.push_back(customer);
                updateRoute(newRoute, data);
                if (isRouteFeasible(newRoute, data)) routes.push_back(newRoute);
            }
        }
    } else {
        routes.push_back(mergedRoute);
    }
    routes.erase(remove_if(routes.begin(), routes.end(),
                           [](const Route& r) { return r.customerIds.empty(); }),
                 routes.end());
}

void swapCustomers(vector<Route>& routes, mt19937& rng, const ProblemData& data) {
    if (routes.size() < 2) return;

    uniform_int_distribution<size_t> routeDist(0, routes.size() - 1);
    size_t routeIdx1 = routeDist(rng);
    size_t routeIdx2 = routeDist(rng);
    size_t attempts = 0;
    const size_t maxAttempts = 100;
    while ((routeIdx1 == routeIdx2 || routes[routeIdx1].customerIds.empty() || routes[routeIdx2].customerIds.empty()) && attempts < maxAttempts) {
        routeIdx2 = routeDist(rng);
        attempts++;
    }
    if (attempts >= maxAttempts) return;

    Route& route1 = routes[routeIdx1];
    Route& route2 = routes[routeIdx2];

    uniform_int_distribution<size_t> posDist1(0, route1.customerIds.size() - 1);
    uniform_int_distribution<size_t> posDist2(0, route2.customerIds.size() - 1);
    size_t pos1 = posDist1(rng);
    size_t pos2 = posDist2(rng);

    int customer1 = route1.customerIds[pos1];
    int customer2 = route2.customerIds[pos2];
    route1.customerIds[pos1] = customer2;
    route2.customerIds[pos2] = customer1;

    vector<int> originalRoute1 = route1.customerIds;
    vector<int> originalRoute2 = route2.customerIds;

    updateRoute(route1, data);
    updateRoute(route2, data);

    if (isRouteFeasible(route1, data) && isRouteFeasible(route2, data)) {
    } else {
        route1.customerIds = originalRoute1;
        route2.customerIds = originalRoute2;
        updateRoute(route1, data);
        updateRoute(route2, data);
    }
}

vector<Route> localSearch(vector<Route> routes, mt19937& rng, const ProblemData& data, chrono::steady_clock::time_point startTime, int maxTime, long long &totalEvalCount, const vector<double>& neighborhoodWeights) {
    vector<Route> currentRoutes = routes;
    vector<Route> bestRoutes = routes;
    long long localEvalCount = 0;
    auto currentObj = objectiveFunction(currentRoutes, localEvalCount);
    auto bestObj = currentObj;
    totalEvalCount += localEvalCount;
    // Compute total weight sum
    double weightSum = 0.0;
    for (double w : neighborhoodWeights) {
        weightSum += w;}
    if (weightSum == 0.0) {
        cout << "All neighborhood weights are 0, skipping local search." << endl;
        return bestRoutes;}
    // Compute cumulative weights for weighted selection
    vector<double> cumulativeWeights(neighborhoodWeights.size());
    cumulativeWeights[0] = neighborhoodWeights[0];
    for (size_t i = 1; i < neighborhoodWeights.size(); ++i) {
        cumulativeWeights[i] = cumulativeWeights[i - 1] + neighborhoodWeights[i];}
    bool improved = true;
    while (improved) {
        improved = false;
        auto currentTime = chrono::steady_clock::now();
        double elapsedTime = chrono::duration_cast<chrono::seconds>(currentTime - startTime).count();
        if (maxTime > 0 && elapsedTime >= maxTime) {
            totalEvalCount += localEvalCount;
            return bestRoutes;}
        vector<Route> neighbor = currentRoutes;
        uniform_real_distribution<double> weightDist(0.0, weightSum);
        double randomWeight = weightDist(rng);
        int operatorChoice = -1;
        for (size_t i = 0; i < cumulativeWeights.size(); ++i) {
            if (randomWeight <= cumulativeWeights[i]) {
                operatorChoice = i;
                break;}}
        if (operatorChoice == 0) {
            removeRouteMove(neighbor, rng, data);
        } else if (operatorChoice == 1) {
            relocateCustomer(neighbor, rng, data);
        } else if (operatorChoice == 2) {
            twoOpt(neighbor, rng, data);
        } else if (operatorChoice == 3) {
            crossExchange(neighbor, rng, data);
        } else if (operatorChoice == 4) {
            mergeRoute(neighbor, rng, data);
        } else if (operatorChoice == 5) {
            swapCustomers(neighbor, rng, data);}
        if (isSolutionFeasible(neighbor, data)) {
            auto neighborObj = objectiveFunction(neighbor, localEvalCount);
            if (BetterSolution(neighbor, currentRoutes, localEvalCount)) {
                currentRoutes = neighbor;
                currentObj = neighborObj;
                improved = true;
                if (BetterSolution(currentRoutes, bestRoutes, localEvalCount)) {
                    bestRoutes = currentRoutes;
                    bestObj = currentObj;
                } }}}
    totalEvalCount += localEvalCount;
    return bestRoutes;}

vector<Route> grasp(const ProblemData &data, mt19937& rng, int maxIterations, int maxTime, int maxEvaluations) {
    vector<Route> bestRoutes;
    long long totalEvalCount = 0;
    auto bestObj = objectiveFunction(bestRoutes, totalEvalCount);
    int evaluations = 0;
    auto startTime = chrono::steady_clock::now();
    const double alpha = 0.2;
    const int RCL_size = 2;

    // Weights: [removeRouteMove, relocateCustomer, twoOpt, crossExchange, mergeRoute, swapCustomers]
    const vector<double> operator_weights = {1.0, 1.0, 0.0, 0.0, 0.0, 0.0};

    cout << "-----------------------------------------" << endl;
    cout << "GRASP Progress (Alpha = " << alpha << ", RCL_size = " << RCL_size << "):" << endl;
    cout << "Neighborhood Weights: [removeRouteMove: " << operator_weights[0]
         << ", relocateCustomer: " << operator_weights[1]
         << ", twoOpt: " << operator_weights[2]
         << ", crossExchange: " << operator_weights[3]
         << ", mergeRoute: " << operator_weights[4]
         << ", swapCustomers: " << operator_weights[5] << "]" << endl;
    cout << "-----------------------------------------" << endl;
    while (evaluations < maxIterations && (maxEvaluations == 0 || evaluations < maxEvaluations)) {
        auto currentTime = chrono::steady_clock::now();
        double elapsedTime = chrono::duration_cast<chrono::seconds>(currentTime - startTime).count();
        if (maxTime > 0 && elapsedTime >= maxTime) break;
        vector<Route> currentRoutes = constructInitialSolution(data, rng, alpha, RCL_size);
        if (!isSolutionFeasible(currentRoutes, data)) continue;
        auto [initVehicles, initDistance] = objectiveFunction(currentRoutes, totalEvalCount);
        cout << "Initial Solution (Iteration " << evaluations + 1 << "): "
             << "Vehicles: " << initVehicles
             << " | Distance: " << fixed << setprecision(2) << initDistance << endl;
        long long evalBeforeLS = totalEvalCount;
        currentRoutes = localSearch(currentRoutes, rng, data, startTime, maxTime, totalEvalCount, operator_weights);
        long long evalAfterLS = totalEvalCount;
        evaluations++;
        long long evalBefore = totalEvalCount;
        if (bestRoutes.empty() || BetterSolution(currentRoutes, bestRoutes, totalEvalCount)) {
            bestRoutes = currentRoutes;
            bestObj = objectiveFunction(bestRoutes, totalEvalCount);}
        long long evalAfter = totalEvalCount;
        auto [vehicles, distance] = bestObj;
        cout << "GRASP Iteration: " << evaluations
             << " | Vehicles: " << vehicles
             << " | Distance: " << fixed << setprecision(0) << distance
             << " | Elapsed Time: " << fixed << setprecision(2) << elapsedTime << " seconds"
             << " | Evaluations in this iteration: " << (evalAfter - evalBefore + evalAfterLS - evalBeforeLS)
             << " | Total Evaluations: " << totalEvalCount
             << endl;
        currentTime = chrono::steady_clock::now();
        elapsedTime = chrono::duration_cast<chrono::seconds>(currentTime - startTime).count();
        if (maxTime > 0 && elapsedTime >= maxTime) break;}
    cout << "\n-----------------------------------------" << endl;
    cout << "GRASP Finished!" << endl;
    cout << "Total Iterations: " << evaluations << endl;
    cout << "Total Evaluations: " << totalEvalCount << endl;
    cout << "-----------------------------------------" << endl;
    return bestRoutes;}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <instanceFile> <maxTime> <maxEvaluations>" << endl;
        return 1;}
    string instanceFile = argv[1];
    int maxTime = stoi(argv[2]);
    int maxEvaluations = stoi(argv[3]);
    cout << "reading instance file: " << instanceFile << ".." << endl;
    ProblemData data = readInstance(instanceFile);
    if (data.customers.empty()) {
        cerr << "Error reading instance." << endl;
        return 1;}
    cout << "Instance read successfully. Customers: " << data.customers.size() - 1 << endl;
    mt19937 rng(static_cast<unsigned>(time(nullptr)));
    auto mainStartTime = chrono::steady_clock::now();
    int maxIterations = 1000;
    vector<Route> bestRoutes = grasp(data, rng, maxIterations, maxTime, maxEvaluations);
    auto mainEndTime = chrono::steady_clock::now();
    double executionTime = chrono::duration_cast<chrono::duration<double>>(mainEndTime - mainStartTime).count();
    if (bestRoutes.empty() && data.customers.size() > 1) {
        cout << "Final solution is empty." << endl;
    } else if (!bestRoutes.empty() && isSolutionFeasible(bestRoutes, data)) {
        cout << "Final solution is FEASIBLE." << endl;
    } else if (!bestRoutes.empty()) {
        cout << "Final solution is INFEASIBLE!" << endl;}
    int routeNumber = 1;
    for (size_t i = 0; i < bestRoutes.size(); ++i) {
        if (!bestRoutes[i].customerIds.empty()) {
            cout << "Route " << routeNumber++ << ":";
            for (int custId : bestRoutes[i].customerIds) {
                cout << " " << custId;
            }
            cout << endl;}}
    long long dummyEvalCount = 0;
    auto [finalVehicles, finalDistance] = objectiveFunction(bestRoutes, dummyEvalCount);
    cout << "Vehicles: " << finalVehicles << endl;
    cout << "Distance: " << fixed << setprecision(2) << finalDistance << endl;
    cout << "Total Execution Time: " << fixed << setprecision(2) << executionTime << " seconds" << endl;
    return 0;}