using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GlobalPlanner : MonoBehaviour {
    public VoxelMap voxelMap = null;
    public int UpDownTh = 3;
    List<Tuple<float, Vector3Int>> sortedQueue = new List<Tuple<float, Vector3Int>>();
    private Dictionary<Vector3Int, float> visitedStates = new Dictionary<Vector3Int, float>();
    private Dictionary<Vector3Int, Dictionary<Vector3Int, float>> actionMap = 
        new Dictionary<Vector3Int, Dictionary<Vector3Int, float>>();
    private Transform startState, goalState, gPlanState;
    private List<Transform> globalPlanStates = new List<Transform>();
    private class Graph {
        private Dictionary<Vector3Int, Vector3Int> graph = new Dictionary<Vector3Int, Vector3Int>();
        public Graph(Vector3Int root){
            graph.Add(root, root);
        }
        public void Append(Vector3Int node, Vector3Int parent){
            if (graph.ContainsKey(parent)){
                if (graph.ContainsKey(node)){
                    graph[node] = parent;
                } else {
                    graph.Add(node, parent);
                }
            } else {
                Debug.Log("No parent: " + parent + " for node: " + node);
            }
        }
        public List<Vector3Int> GetFullDiscretePath(Vector3Int endState){
            if (!graph.ContainsKey(endState)){
                Debug.Log("No end state in graph: " + endState);
                return new List<Vector3Int>();
            }
            List<Vector3Int> path = new List<Vector3Int>{endState};
            Vector3Int parent = graph[endState];
            while(graph[parent] != parent){
                path.Add(parent);
                parent = graph[parent];
            }
            path.Reverse();
            return path;
        }
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
            sortedQueue.Add(new Tuple<float, Vector3Int>(cost, v));
        }
    }
    private List<Vector3Int> GetActionSpace(){
        List<Vector3Int> actionSpace = new List<Vector3Int>();
        for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y)
                for (int z = -1; z <= 1; ++z){
                    if ((x == 0) && (z == 0) && (y == 0))
                            continue;
                    actionSpace.Add(new Vector3Int(x, y, z));
                }
        return actionSpace;
    }
    public void SetActionCost(Vector3Int action, Vector3Int state, float cost){
        if (actionMap.ContainsKey(state)){
            if (actionMap[state].ContainsKey(action)){
                actionMap[state][action] += cost;
            } else {
                actionMap[state].Add(action, cost);
            }
        } else {
            actionMap.Add(state, new Dictionary<Vector3Int, float>(){
                {action, cost} }
            );
        }
    }
    private float GetActionCost(Vector3Int action, Vector3Int state){
        float additionalCost = 0.0f;
        if (actionMap.ContainsKey(state)){
            var actions = actionMap[state];
            if (actions.ContainsKey(action)){
                additionalCost = actions[action];
            }
        }
        return 1 + additionalCost;
    }
    private float GetHeuristics(Vector3Int v){
        return v.magnitude;
    }
    private Vector3Int GetFeasibleState(Vector3 v){
        Vector3Int vDiscrete = voxelMap.GetDiscreteState(v);
        // while (voxelMap.IsObstacle(vDiscrete))
        //     vDiscrete.y += 1;
        // while (!voxelMap.IsObstacle(vDiscrete + new Vector3Int(0, -1, 0)) && vDiscrete.y >= 0)
        //     vDiscrete += new Vector3Int(0, -1, 0);
        return vDiscrete;
    }
    
    public List<Vector3> GetGlobalPlan(Vector3 start, Vector3 goal){
        Vector3Int startStateDescrete = GetFeasibleState(start);
        Vector3Int goalStateDescrete = GetFeasibleState(goal);

        sortedQueue.Clear();
        visitedStates.Clear();
        Graph graph = new Graph(startStateDescrete);
        Insert(0.0f, startStateDescrete);
        visitedStates.Add(startStateDescrete, 0.0f);

        while (sortedQueue.Count > 0){
            Vector3Int currentState = sortedQueue[0].Item2;
            sortedQueue.RemoveAt(0);

            if (currentState == goalStateDescrete){
                var pathDiscrete = graph.GetFullDiscretePath(currentState);
                return GetContinuousPlan(pathDiscrete);
            }

            List<Vector3Int> actions = GetActionSpace();
            foreach (Vector3Int act in actions) {
                Vector3Int action = act;
                Vector3Int nextState = currentState + action;
                if (voxelMap.IsObstacle(nextState)){
                    visitedStates[nextState] = Mathf.Infinity;

                    {
                        // Vector3Int lifting = new Vector3Int(0, 1, 0);
                        // while (voxelMap.IsObstacle(nextState + lifting) && (Mathf.Abs(lifting.y) < UpDownTh)){
                        //     visitedStates[nextState + lifting] = Mathf.Infinity;
                        //     lifting.y += 1;
                        // }
                        // if (voxelMap.IsObstacle(nextState + lifting)){
                        //     visitedStates[nextState] = Mathf.Infinity;
                        //     continue;
                        // } else {
                        //     if (Mathf.Abs(action.x) == Mathf.Abs(action.z)){
                        //         continue;
                        //     } else {
                        //         action += lifting;
                        //         nextState += lifting;
                        //     }
                        // }
                    }
                }
                
                {
                    // Vector3Int descent = new Vector3Int(0, -1, 0);
                    // while (!voxelMap.IsObstacle(nextState + descent) && (Mathf.Abs(descent.y) < UpDownTh)){
                    //     descent.y -= 1;
                    // }
                    // if (voxelMap.IsObstacle(nextState + descent)){
                    //     descent.y += 1;
                    //     if (descent.y != 0 && (Mathf.Abs(action.x) == Mathf.Abs(action.z))){
                    //         continue;
                    //     } else {
                    //         action += descent;
                    //         nextState += descent;
                    //     }
                    // } else {
                    //     continue;
                    // }
                    // action += voxelMap.GetAction(currentState);
                    // nextState += voxelMap.GetAction(currentState);
                }

                bool isVisited = visitedStates.ContainsKey(nextState);
                float currentCost = visitedStates[currentState];
                if (isVisited){
                    // Additional Cost for action from currentState
                    float newCost = currentCost + GetActionCost(action, currentState);
                    float prevCost = visitedStates[nextState];
                    if (newCost < prevCost){
                        visitedStates[nextState] = newCost;
                        graph.Append(nextState, currentState);
                    }
                } else {
                    float nextCost = currentCost + GetActionCost(action, currentState);
                    // Additional cost from learned cost map
                    nextCost += voxelMap.GetWeigth(nextState);
                    visitedStates.Add(nextState, nextCost);
                    float queueCost = nextCost + GetHeuristics(goalStateDescrete - nextState);
                    Insert(queueCost, nextState);
                    graph.Append(nextState, currentState);
                }
            }
        }
        return new List<Vector3>();
    }
    private List<Vector3> GetContinuousPlan(List<Vector3Int> plan){
        List<Vector3> res = new List<Vector3>();
        foreach (var item in plan)
            res.Add(voxelMap.GetContinuousState(item));
        return res;
    }
    public void SetWieght(Vector3Int vD, float cost){
        voxelMap.SetWeight(vD, cost);
    }
    public void FindAndSetCellWeight(Vector3 pos, float cost){
        voxelMap.SetWeight(voxelMap.GetDiscreteState(pos), cost);
    }
    public void ShowGlobalPlan(List<Vector3> plan){
        gPlanState = transform.GetChild(2);
        gPlanState.gameObject.SetActive(false);

        foreach (var item in globalPlanStates)
            Destroy(item.gameObject);
        globalPlanStates.Clear();

        if (plan.Count == 0)
            return;
        foreach (var item in plan){
            var newObj = Instantiate(gPlanState, item, Quaternion.identity, transform);
            newObj.gameObject.SetActive(true);
            newObj.gameObject.name = "global " + globalPlanStates.Count.ToString();
            globalPlanStates.Add(newObj);
        }
    }
}
