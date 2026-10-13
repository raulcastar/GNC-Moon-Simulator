# -*- coding: utf-8 -*-
#=====================DyMu PATH PLANNING library===============================
# Python library containing DyMu (Dynamic Multilayered path planner) functions
#          Authors: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#                   Gonzalo Paz Delgado
#==============================================================================

import logging

import numpy as np
import math
import bisect
import cv2

# Gradient-descent steps without a drop in T before getPathGDM gives up on the
# continuous path and descends cell by cell (getPathDiscrete).
GDM_STALL_STEPS = 40
# Loop iterations between checks of Global_Path_Planner.should_stop.
STOP_CHECK_EVERY = 256


class PlanningCancelled(Exception):
    """Raised inside a search when should_stop() turns true (the plan is no longer wanted)."""

class Global_Path_Planner:
    def __init__(self):
        # The owning node replaces this with its ROS logger (so warnings reach /rosout).
        self.log = logging.getLogger('global_path_planner')
        # fn() -> bool, polled by the long loops: true stops the search with
        # PlanningCancelled (the owning node sets it while a plan is computed).
        self.should_stop = None
        self.rad2deg = 180/np.pi
        self.slopecostLUT = np.zeros((2,5))
        self.slopecostLUT[:][0] = [0, 5, 10, 15, 20]
        self.slopecostLUT[:][1] = [10.0, 12.0, 15.0, 20.0, 40.0]


    # =============================================================================
    #    Eikonal Equation
    # =============================================================================
    def getEikonal(self, Thor,Tver,cost):
        if np.isinf(Thor):
            if np.isinf(Tver):
                return np.inf
            else:
                return Tver+cost
        if np.isinf(Tver):
            return Thor+cost
        else:
            if cost < np.abs(Thor-Tver):
                return np.minimum(Thor,Tver)+cost
            else:
                return .5*(Thor+Tver+math.sqrt(2*np.power(cost,2) - \
                                            np.power(Thor-Tver,2)))


    # =============================================================================
    #    Get von Neumann neighbors from a node
    # =============================================================================
    def getNeighbours(self, nodeTarget, closedMap):
        nList = []
        nTij = nodeTarget[0:2]
        neighbours = [[-1,0],
                    [1,0],
                    [0,-1],
                    [0,1]]
        for ni in neighbours:
            nN = np.add(nTij,ni)
            if closedMap[nN[1], nN[0]] == 0:
                nList.append(nN)
        return nList


    # =============================================================================
    #    Compute values of T of neighbouring nodes
    # =============================================================================
    def updateNode(self, nodeTarget, costMap, Tmap, nbT, nbNodes, closedMap):
        #nList = getNeighbours(nodeTarget, closedMap)
        for i in range(1,4+1):
            if i == 1:
                nodeChild = np.add(nodeTarget,[0,-1])
            elif i == 2:
                nodeChild = np.add(nodeTarget,[0,1])
            elif i == 3:
                nodeChild = np.add(nodeTarget,[-1,0])
            elif i == 4:
                nodeChild = np.add(nodeTarget,[1,0])
                    
            if closedMap[nodeChild[1],nodeChild[0]] == 0:     
                Thor1 = Tmap[nodeChild[1],nodeChild[0]+1]
                Thor2 = Tmap[nodeChild[1],nodeChild[0]-1]
                Thor = np.minimum(Thor1,Thor2)
                Tver1 = Tmap[nodeChild[1]+1,nodeChild[0]]
                Tver2 = Tmap[nodeChild[1]-1,nodeChild[0]]
                Tver = np.minimum(Tver1,Tver2)
                T = self.getEikonal(Thor,Tver,costMap[nodeChild[1],nodeChild[0]])
                if np.isinf(Tmap[nodeChild[1],nodeChild[0]]):
                    nIndex = bisect.bisect_left(nbT,T)
                    nbT.insert(nIndex,T)
                    nbNodes.insert(nIndex, nodeChild)
                    Tmap[nodeChild[1],nodeChild[0]] = T
                else:
                    if T < Tmap[nodeChild[1],nodeChild[0]]:
                        tempT = Tmap[nodeChild[1],nodeChild[0]]
                        nIndex = bisect.bisect_left(nbT,tempT)
                        try:
                            nIndex = next(x for x,n in \
                                    enumerate(nbNodes[nIndex:],nIndex) if \
                                    np.array_equal(nodeChild,nbNodes[x]))
                            del nbT[nIndex]
                            del nbNodes[nIndex]
                        except StopIteration:
                            # Not in the narrow band: nothing to remove. (Deleting at the
                            # bisect index anyway dropped some other node from the front.)
                            self.log.warning(f'Fast marching: node {nodeChild.tolist()} with T '
                                             f'{tempT:.2f} missing from the narrow band')
                        nIndex = bisect.bisect_left(nbT,T)
                        nbT.insert(nIndex,T)
                        nbNodes.insert(nIndex, nodeChild)
                        Tmap[nodeChild[1],nodeChild[0]] = T     
        return Tmap, nbT, nbNodes
        
    # =============================================================================
    #    Get the node in the narrowBand with the lower value of Total Cost T
    # =============================================================================
    def getMinNB(self, nbT,nbNodes):
        nodeTarget = nbNodes.pop(0)
        del nbT[0]
        return nodeTarget, nbT, nbNodes


    # =============================================================================
    #    Compute values of T of neighbouring nodes
    # =============================================================================
    def computeTmap(self, costMap,obstacleMap,goal,start):
        closedMap = np.zeros_like(costMap)
        closedMap[np.where(obstacleMap == 1)] = 1
        Tmap = np.ones_like(costMap)*np.inf
        nbT = []
        nbNodes = []
        Tmap[goal[1],goal[0]] = 0
        closedMap[goal[1],goal[0]] = 1
        nodeTarget = [goal[0],goal[1]]
        [Tmap, nbT, nbNodes] = self.updateNode(nodeTarget, costMap, Tmap, nbT, nbNodes, closedMap)
        #iter = 1
        #size = np.size(costMap)
        if start == None:
            steps = 0
            while nbT:
                steps += 1
                self._check_stop(steps)
                nodeTarget, nbT, nbNodes = self.getMinNB(nbT, nbNodes)
                closedMap[nodeTarget[1],nodeTarget[0]] = 1
                Tmap, nbT, nbNodes = self.updateNode(nodeTarget, costMap, Tmap, nbT, nbNodes, closedMap)
        else:
            steps = 0
            while nbT:
                steps += 1
                self._check_stop(steps)
                nodeTarget, nbT, nbNodes = self.getMinNB(nbT, nbNodes)
                closedMap[nodeTarget[1],nodeTarget[0]] = 1
                Tmap, nbT, nbNodes = self.updateNode(nodeTarget, costMap, Tmap, nbT, nbNodes, closedMap)
                if np.array_equal(nodeTarget,start):
                    break
        return Tmap


    # =============================================================================
    #    Compute path waypoints using the Gradient Descent Method
    #  (Note: Dijkstra is also used for degenerated cases, but should be avoided)
    # =============================================================================
    def getPathGDM(self, totalCostMap,initWaypoint,endWaypoint,tau):
        
    #    G1,G2 = computeGradient(totalCostMap)
        
        gamma = np.empty([0,2])
        gamma = np.vstack((gamma,initWaypoint.T))

        nearN = [0,0]

        # Stall guard: in a one-cell-wide passage the continuous steps cut into
        # impassable cells and the recovery below can oscillate there for
        # thousands of steps. If T has not dropped for GDM_STALL_STEPS steps,
        # descend cell by cell instead (always reaches the goal).
        bestT, sinceBest = np.inf, 0

        for k in range(0,round(15000/tau)):
            self._check_stop(k + 1)
            cx, cy = int(round(gamma[-1,0])), int(round(gamma[-1,1]))
            if 0 <= cy < totalCostMap.shape[0] and 0 <= cx < totalCostMap.shape[1] \
                    and totalCostMap[cy,cx] < bestT:
                bestT, sinceBest = totalCostMap[cy,cx], 0
            else:
                sinceBest += 1
                if sinceBest > GDM_STALL_STEPS:
                    return self.getPathDiscrete(totalCostMap, initWaypoint, endWaypoint)
            dx, dy = self.gradientAt(totalCostMap, gamma[-1,:])
            
            if (np.isnan(dx)) or (np.isnan(dy)):
                try:
                    nearN[0] = int(round(gamma[-1,0]))
                    nearN[1] = int(round(gamma[-1,1]))
                    while np.isinf(totalCostMap[nearN[1],nearN[0]]):
                        gamma = np.delete(gamma,-1,axis=0)
                        nearN[0] = int(round(gamma[-1,0]))
                        nearN[1] = int(round(gamma[-1,1]))
                    
                    if gamma.size > 0:
                        while (np.linalg.norm(gamma[-1,:]-nearN)) < 1:
                            gamma = np.delete(gamma,-1,axis=0)
                            if gamma.size == 0:
                                break
                            
                    gamma = np.vstack((gamma,np.array(nearN).T))
                    # Step towards the lowest of the 4 neighbours (T was
                    # computed 4-connected). This used to add the offsets with
                    # list concatenation (nearN + [0,-1]), so it never looked at
                    # a neighbour and the NaN step went into the path, e.g.
                    # with a rock in the cells beside the rover.
                    currentT = totalCostMap[nearN[1],nearN[0]]
                    rows, cols = totalCostMap.shape
                    for ox, oy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
                        nx_, ny_ = nearN[0] + ox, nearN[1] + oy
                        if 0 <= nx_ < cols and 0 <= ny_ < rows and totalCostMap[ny_, nx_] < currentT:
                            currentT = totalCostMap[ny_, nx_]
                            dx, dy = -ox / tau, -oy / tau
                except Exception as exc:
                    self.log.warning(f'Path extraction stopped at node {nearN} after '
                                     f'{len(gamma)} points ({type(exc).__name__}: {exc}); '
                                     f'returning the partial path')
                    return gamma
                if np.isnan(dx) or np.isnan(dy):
                    # No lower neighbour to step to: walk the cells instead.
                    return self.getPathDiscrete(totalCostMap, initWaypoint, endWaypoint)
                    
            
            if dx == 0 and dy == 0:
                # Flat T here (between blocked cells): no direction to step in.
                return self.getPathDiscrete(totalCostMap, initWaypoint, endWaypoint)
            if np.linalg.norm([dx,dy]) < 0.01:
                dnx = dx/math.sqrt(dx**2+dy**2)
                dny = dy/math.sqrt(dx**2+dy**2)
                add = gamma[-1,:] - np.dot(tau,[dnx,dny])
                gamma = np.vstack((gamma,np.array(add).T))
            else:
                dx = dx/math.sqrt(dx**2+dy**2)
                dy = dy/math.sqrt(dx**2+dy**2)
                add = gamma[-1,:] - np.dot(tau,[dx,dy])
                gamma = np.vstack((gamma,np.array(add).T))
            
            if np.linalg.norm(gamma[-1,:]-endWaypoint)<1.5:
                break
        
        else:
            # Ran out of steps without reaching the goal.
            return self.getPathDiscrete(totalCostMap, initWaypoint, endWaypoint)

        gamma = np.vstack((gamma,np.array(endWaypoint).T))

        return gamma


    # =============================================================================
    #    Fallback path: steepest descent of T over the 4 neighbours, cell by cell.
    #    T was computed 4-connected from the goal, so every reached cell has a
    #    lower neighbour and the walk ends at the goal without crossing obstacles.
    # =============================================================================
    def getPathDiscrete(self, totalCostMap, initWaypoint, endWaypoint):
        rows, cols = totalCostMap.shape
        x, y = int(round(initWaypoint[0])), int(round(initWaypoint[1]))
        gx, gy = int(round(endWaypoint[0])), int(round(endWaypoint[1]))
        path = [[x, y]]
        for step in range(rows * cols):
            self._check_stop(step + 1)
            if (x, y) == (gx, gy):
                return np.array(path, dtype=float)
            best = None
            for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < cols and 0 <= ny < rows and totalCostMap[ny, nx] < totalCostMap[y, x] \
                        and (best is None or totalCostMap[ny, nx] < totalCostMap[best[1], best[0]]):
                    best = (nx, ny)
            if best is None:
                return None
            x, y = best
            path.append([x, y])
        return None


    # =============================================================================
    #    Use this gradient computation instead of np.gradient, since this acknow-
    #    ledges cost discontinuities
    # =============================================================================
    def _check_stop(self, step):
        """Every STOP_CHECK_EVERY steps: PlanningCancelled if should_stop() says so."""
        if step % STOP_CHECK_EVERY == 0 and self.should_stop is not None and self.should_stop():
            raise PlanningCancelled()

    def _cell_gradient(self, cost, j, i):
        """
        Normalised gradient (gnx, gny) of `cost` at cell (row j, column i): the
        per-cell computation of computeGradient, on its own.
        """
        m, n = cost.shape
        if j == 0:
            gy = cost[1,i]-cost[0,i]
        elif j == m-1:
            gy = cost[j,i]-cost[j-1,i]
        elif np.isinf(cost[j+1,i]):
            gy = 0 if np.isinf(cost[j-1,i]) else cost[j,i]-cost[j-1,i]
        elif np.isinf(cost[j-1,i]):
            gy = cost[j+1,i]-cost[j,i]
        else:
            gy = (cost[j+1,i]-cost[j-1,i])/2
        if i == 0:
            gx = cost[j,1]-cost[j,0]
        elif i == n-1:
            gx = cost[j,i]-cost[j,i-1]
        elif np.isinf(cost[j,i+1]):
            gx = 0 if np.isinf(cost[j,i-1]) else cost[j,i]-cost[j,i-1]
        elif np.isinf(cost[j,i-1]):
            gx = cost[j,i+1]-cost[j,i]
        else:
            gx = (cost[j,i+1]-cost[j,i-1])/2
        # Stored as float64 in computeGradient's arrays: the same values here.
        gx, gy = np.float64(gx), np.float64(gy)
        magnitude = math.sqrt(gx**2 + gy**2)
        if magnitude == 0 or math.isinf(magnitude) or math.isnan(magnitude):
            return np.float64(0.0), np.float64(0.0)
        return gx / magnitude, gy / magnitude

    def gradientAt(self, cost, point):
        """
        The normalised gradient of `cost` interpolated at `point` (x, y): what
        interpolatePoint(point, computeGradient(cost, point)[k]) gives, computed
        on the (at most) four cells the interpolation reads instead of two
        full-size map arrays.
        """
        cache = {}

        def cell(k):
            def value(j, i):
                key = (int(j), int(i))
                if key not in cache:
                    cache[key] = self._cell_gradient(cost, key[0], key[1])
                return cache[key][k]
            return value

        return (self._interpolate(point, cost.shape, cell(0)),
                self._interpolate(point, cost.shape, cell(1)))

    def computeGradient(self, cost, point=[]):
        
        m,n = cost.shape
        
        if len(point) == 0:
            jmax,imax = m,n
            jmin,imin = 0,0
        else:
            jmax,imax = min(m,int(point[1])+3),min(n,int(point[0])+3)

            jmin,imin = max(int(0),int(point[1]-3)),max(int(0),int(point[0]-3))
        
        
        Gx = np.zeros_like(cost)
        Gy = np.zeros_like(cost)
        Gnx = np.zeros_like(cost)
        Gny = np.zeros_like(cost)
        
        for i in range(imin,imax):
            for j in range(jmin, jmax):
                if j == 0:
                    Gy[0,i] = cost[1,i]-cost[0,i]
                else:
                    if j == m-1:
                        Gy[j,i] = cost[j,i]-cost[j-1,i]
                    else:
                        if np.isinf(cost[j+1,i]):
                            if np.isinf(cost[j-1,i]):
                                Gy[j,i] = 0
                            else:
                                Gy[j,i] = cost[j,i]-cost[j-1,i]
                        else:
                            if np.isinf(cost[j-1,i]):
                                Gy[j,i] = cost[j+1,i]-cost[j,i]
                            else:
                                Gy[j,i] = (cost[j+1,i]-cost[j-1,i])/2
                
                if i == 0:
                    Gx[j,0] = cost[j,1]-cost[j,0]
                else:
                    if i == n-1:
                        Gx[j,i] = cost[j,i]-cost[j,i-1]
                    else:
                        if np.isinf(cost[j,i+1]):
                            if np.isinf(cost[j,i-1]):
                                Gx[j,i] = 0
                            else:
                                Gx[j,i] = cost[j,i]-cost[j,i-1]
                        else:
                            if np.isinf(cost[j,i-1]):
                                Gx[j,i] = cost[j,i+1]-cost[j,i]
                            else:
                                Gx[j,i] = (cost[j,i+1]-cost[j,i-1])/2
                    
                # OLD BROKEN CODE:
                # Gnx[j,i] = Gx[j,i]/math.sqrt(Gx[j,i]**2+Gy[j,i]**2)
                # Gny[j,i] = Gy[j,i]/math.sqrt(Gx[j,i]**2+Gy[j,i]**2)

                # NEW FIXED CODE:
                # Calculate the magnitude (hypotenuse)
                magnitude = math.sqrt(Gx[j,i]**2 + Gy[j,i]**2)

                # Check for 0, infinity, or NaN
                if magnitude == 0 or math.isinf(magnitude) or math.isnan(magnitude):
                    Gnx[j,i] = 0
                    Gny[j,i] = 0
                else:
                    Gnx[j,i] = Gx[j,i] / magnitude
                    Gny[j,i] = Gy[j,i] / magnitude
                
        
        return Gnx,Gny
            
        
    # =============================================================================
    #    Interpolate the value of a XY point within the discrete map I
    # =============================================================================
    def interpolatePoint(self, point,mapI):
        return self._interpolate(point, mapI.shape, lambda j, i: mapI[j, i])

    def _interpolate(self, point, shape, at):
        """Bilinear value at `point` (x, y) of a map given by at(row, col)."""
        i = np.uint32(np.fix(point[0]))
        j = np.uint32(np.fix(point[1]))
        a = point[0] - i
        b = point[1] - j
        
        
        m,n = np.uint32(shape)
        
        if i == n:
            if j == m:
                I = at(j,i)
            else:
                I = b*at(j+1,i) + (1-b)*at(j,i)
        else:
            if j == m:
                I = a*at(j,i+1) + (1-a)*at(j,i)
            else:
                a00 = at(j,i)
                a10 = at(j,i+1) - at(j,i)
                a01 = at(j+1,i) - at(j,i)
                a11 = at(j+1,i+1) + at(j,i) - at(j,i+1) - at(j+1,i)
                if a == 0:
                    if b == 0:
                        I = a00
                    else:
                        I = a00 + a01*b
                else:
                    if b == 0:
                        I = a00 + a10*a
                    else:
                        I = a00 + a10*a + a01*b + a11*a*b
                        
        return I
    

    def getCost(self, slope):
        if slope >= self.slopecostLUT[0][-1]:
            return 50.0
        index = bisect.bisect_left(self.slopecostLUT[0][:], slope)
        decimalPart = (slope - self.slopecostLUT[0][index-1])/(self.slopecostLUT[0][index] - self.slopecostLUT[0][index-1])
        cost = decimalPart*(self.slopecostLUT[1][index]-self.slopecostLUT[1][index-1]) + self.slopecostLUT[1][index-1]
        return cost


    def getCostMap(self, elevationMap, soilMap, res, slopeThreshold):
        
        dX,dY = np.gradient(elevationMap,*[res, res])
        slopeMap = self.rad2deg*np.arctan(np.sqrt(dX**2+dY**2))
        slopeMap[np.isnan(slopeMap)] = 0
        
        costMap = np.ones_like(elevationMap)
        obstacleMap = np.zeros_like(elevationMap)
        
        obstacleMap[np.where(slopeMap > slopeThreshold)] = 1
        obstacleMap[soilMap == 0] = 1
        
        for i in range(slopeMap.shape[1]):
            for j in range(slopeMap.shape[0]):
                costMap[j][i] = self.getCost(slopeMap[j][i])
        
        costMap[obstacleMap == 1] = 2*slopeThreshold
        
        costMap = cv2.blur(costMap,(3,3))
        
        return costMap, obstacleMap, slopeMap