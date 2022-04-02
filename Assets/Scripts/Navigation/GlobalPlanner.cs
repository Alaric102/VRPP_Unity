using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GlobalPlanner : MonoBehaviour {
    public VoxelMap voxelMap = null;
    public Transform startObj, goalObj, visitedObj;
    private Vector3Int startStateDescrete , goalStateDescrete;
    List<Tuple<float, Vector3Int>> sortedQueue = new List<Tuple<float, Vector3Int>>();
    private Dictionary<Vector3Int, float> visitedStates = new Dictionary<Vector3Int, float>();
    private bool enable = false;
    void Start() {

    }

    // Update is called once per frame
    void Update() {
        if (enable && sortedQueue.Count > 0){
            Vector3Int currentState = sortedQueue[0].Item2;
            sortedQueue.RemoveAt(0);

            if (currentState == goalStateDescrete){
                enable = false;
            }

            List<Vector3Int> actions = GetActionSpace();
            foreach (Vector3Int action in actions) {
                Vector3Int nextState = currentState + action;
                if (voxelMap.IsObstacle(nextState)){
                    continue;
                }

                Vector3 pos = voxelMap.GetContinuousState(nextState);
                Transform newObj = Instantiate(visitedObj, pos, Quaternion.identity, transform);
                newObj.gameObject.name = action.ToString();

                bool isVisited = visitedStates.ContainsKey(nextState);
                float currentCost = visitedStates[currentState];
                if (isVisited){
                    float newCost = currentCost + GetActionCost(action);
                    float prevCost = visitedStates[nextState];
                    if (newCost < prevCost){
                        visitedStates[nextState] = newCost;
                    }
                } else {
                    float nextCost = currentCost + GetActionCost(action);
                    visitedStates.Add(nextState, nextCost);
                    float queueCost = nextCost + GetHeuristics(goalStateDescrete - nextState);
                    Insert(queueCost, nextState); 
                    newObj.gameObject.name += "; " + nextCost.ToString() + ", " + queueCost.ToString();
                }
            }
        }
    }
    private void SetStartState(Vector3 v){
        startStateDescrete = voxelMap.GetDescreteState(v);
    }
    private void SetGoalState(Vector3 v){
        goalStateDescrete = voxelMap.GetDescreteState(v);
    }
    private int FindLowerBound(float cost, int minID, int maxID){
        if (maxID - minID == 1){
            return (sortedQueue[minID].Item1 < cost) ? minID : maxID;
        }
        int meanID = ((int)Mathf.Floor((maxID + minID)/2));
        if (sortedQueue[meanID].Item1 > cost)
            maxID = meanID;
        else
            minID = meanID;
        return FindLowerBound(cost, minID, maxID);
    }
    private void Insert(float cost, Vector3Int v){
        if (sortedQueue.Count == 0)
            sortedQueue.Add(new Tuple<float, Vector3Int>(cost, v));
        else{
            for (int i = 0; i < sortedQueue.Count; ++i){
                if (sortedQueue[i].Item1 > cost){
                    sortedQueue.Insert(i, new Tuple<float, Vector3Int>(cost, v));
                    return;
                }
            }
            // int id = FindLowerBound(cost, 0, sortedQueue.Count);
            // sortedQueue.Insert(id, new Tuple<float, Vector3Int>(cost, v));
        }
    }
    private List<Vector3Int> GetActionSpace(){
        List<Vector3Int> actionSpace = new List<Vector3Int>();
        for (int x = -1; x <= 1; ++x)
            for (int z = -1; z <= 1; ++z){
                if ((x == 0) && (z == 0))
                        continue;
                actionSpace.Add(new Vector3Int(x, 0, z));
            }
        return actionSpace;
    }
    private float GetActionCost(Vector3Int v){
        return (v.y != 0) ? 2.0f : 1.0f;
    }
    private float GetHeuristics(Vector3Int v){
        return v.magnitude;
    }
    public bool StartPlan(Vector3 start, Vector3 goal){
        startStateDescrete = voxelMap.GetDescreteState(start);
        goalStateDescrete = voxelMap.GetDescreteState(goal);

        while (voxelMap.IsObstacle(startStateDescrete))
            startStateDescrete.y += 1;
        while (voxelMap.IsObstacle(goalStateDescrete))
            goalStateDescrete.y += 1;

        Instantiate(startObj, voxelMap.GetContinuousState(startStateDescrete), Quaternion.identity, transform);
        Instantiate(goalObj, voxelMap.GetContinuousState(goalStateDescrete), Quaternion.identity, transform);

        sortedQueue.Clear();
        visitedStates.Clear();
        Insert(0, startStateDescrete);
        visitedStates.Add(startStateDescrete, 0);

        // while (sortedQueue.Count > 0){
        //     Vector3Int currentState = sortedQueue[0].Item2;
        //     sortedQueue.RemoveAt(0);

        //     if (currentState == goalStateDescrete){
        //         enable = false;
        //         return true;
        //     }

        //     List<Vector3Int> actions = GetActionSpace();
        //     foreach (Vector3Int action in actions) {
        //         Vector3Int nextState = currentState + action;
        //         if (voxelMap.IsObstacle(nextState)){
        //             continue;
        //         }

        //         bool isVisited = visitedStates.ContainsKey(nextState);
        //         float currentCost = visitedStates[currentState];
        //         if (isVisited){
        //             float newCost = currentCost + GetActionCost(action);
        //             float prevCost = visitedStates[nextState];
        //             if (newCost < prevCost){
        //                 visitedStates[nextState] = newCost;
        //             }
        //         } else {
        //             float nextCost = currentCost + GetActionCost(action);
        //             visitedStates.Add(nextState, nextCost);
        //             float queueCost = nextCost + GetHeuristics(goalStateDescrete - nextState);
        //             Insert(nextCost, nextState);                    
        //         }
        //     }
        // }
        enable = true;
        return false;
    }

    public void ShowPlan(){
        // voxelMap.ShowMap();
        Instantiate(startObj, voxelMap.GetContinuousState(startStateDescrete), Quaternion.identity, transform);
        Instantiate(goalObj, voxelMap.GetContinuousState(goalStateDescrete), Quaternion.identity, transform);
        foreach (var item in visitedStates){
            Vector3 pos = voxelMap.GetContinuousState(item.Key);
            Instantiate(visitedObj, pos, Quaternion.identity, transform);
        }
    }
}
