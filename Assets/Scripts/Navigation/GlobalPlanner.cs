using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GlobalPlanner : MonoBehaviour {
    public VoxelMap voxelMap = null;
    public Transform startObj, goalObj, visitedObj;
    public int UpDownTh = 3;
    private Vector3Int startStateDescrete , goalStateDescrete;
    List<Tuple<float, Vector3Int>> sortedQueue = new List<Tuple<float, Vector3Int>>();
    private Dictionary<Vector3Int, float> visitedStates = new Dictionary<Vector3Int, float>();
    private class Graph {
        public Graph(Vector3Int root){
            graph.Add(root, root);
        }
        private Dictionary<Vector3Int, Vector3Int> graph = 
            new Dictionary<Vector3Int, Vector3Int>();
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
        private List<Vector3Int> GetFullPath(Vector3Int endState){
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

        public List<Vector3Int> GetPlan(Vector3Int endState){
            List<Vector3Int> path = GetFullPath(endState);
            if (path.Count == 0){
                return path;
            }
            List<Vector3Int> res = new List<Vector3Int>();
            Vector3Int prevDelta = Vector3Int.zero;
            for (int id = 0; id < path.Count - 1; ++id){
                Vector3Int delta = path[id + 1] - path[id];
                if (delta == prevDelta){
                    continue;
                } else {
                    res.Add(path[id]);
                    prevDelta = delta;
                }
            }
            res.Add(path[path.Count - 1]);
            return res;
        }
    }
    private Graph graph;
    private List<Transform> planStates = new List<Transform>();
    private bool enable = false;
    void Start() {

    }
    void Update() {

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
            for (int y = 0; y <= 0; ++y)
                for (int z = -1; z <= 1; ++z){
                    if ((x == 0) && (z == 0) && (y == 0))
                            continue;
                    actionSpace.Add(new Vector3Int(x, y, z));
                }
        return actionSpace;
    }
    private float GetActionCost(Vector3Int v){
        return (v.y != 0) ? (1 + Mathf.Abs(v.y)) : 1.0f;
        // return v.magnitude;
    }
    private float GetHeuristics(Vector3Int v){
        return v.magnitude;
    }
    public List<Vector3Int> GetGlobalPlan(Vector3 start, Vector3 goal){

        startStateDescrete = voxelMap.GetDescreteState(start);
        goalStateDescrete = voxelMap.GetDescreteState(goal);

        while (voxelMap.IsObstacle(startStateDescrete))
            startStateDescrete.y += 1;
        while (!voxelMap.IsObstacle(startStateDescrete + new Vector3Int(0, -1, 0))){
            startStateDescrete += new Vector3Int(0, -1, 0);
        }

        while (voxelMap.IsObstacle(goalStateDescrete))
            goalStateDescrete.y += 1;
        while (!voxelMap.IsObstacle(goalStateDescrete + new Vector3Int(0, -1, 0))){
            goalStateDescrete += new Vector3Int(0, -1, 0);
        }

        sortedQueue.Clear();
        visitedStates.Clear();
        graph = new Graph(startStateDescrete);
        Insert(0, startStateDescrete);
        visitedStates.Add(startStateDescrete, 0);

        while (sortedQueue.Count > 0){
            Vector3Int currentState = sortedQueue[0].Item2;
            sortedQueue.RemoveAt(0);

            if (currentState == goalStateDescrete){
                return graph.GetPlan(currentState);
            }

            List<Vector3Int> actions = GetActionSpace();
            foreach (Vector3Int act in actions) {
                Vector3Int action = act;
                Vector3Int nextState = currentState + action;
                if (voxelMap.IsObstacle(nextState)){
                    visitedStates[nextState] = Mathf.Infinity;

                    Vector3Int lifting = new Vector3Int(0, 1, 0);
                    while (voxelMap.IsObstacle(nextState + lifting) && (Mathf.Abs(lifting.y) < UpDownTh)){
                        visitedStates[nextState + lifting] = Mathf.Infinity;
                        lifting.y += 1;
                    }
                    if (voxelMap.IsObstacle(nextState + lifting)){
                        visitedStates[nextState] = Mathf.Infinity;
                        continue;
                    } else {
                        if (Mathf.Abs(action.x) == Mathf.Abs(action.z)){
                            continue;
                        } else {
                            action += lifting;
                            nextState += lifting;
                        }
                    }
                }

                Vector3Int descent = new Vector3Int(0, -1, 0);
                while (!voxelMap.IsObstacle(nextState + descent) && (Mathf.Abs(descent.y) < UpDownTh)){
                    descent.y -= 1;
                }
                if (voxelMap.IsObstacle(nextState + descent)){
                    descent.y += 1;
                    if (descent.y != 0 && (Mathf.Abs(action.x) == Mathf.Abs(action.z))){
                        continue;
                    } else {
                        action += descent;
                        nextState += descent;
                    }
                } else {
                    continue;
                }

                bool isVisited = visitedStates.ContainsKey(nextState);
                float currentCost = visitedStates[currentState];
                if (isVisited){
                    float newCost = currentCost + GetActionCost(action);
                    float prevCost = visitedStates[nextState];
                    if (newCost < prevCost){
                        visitedStates[nextState] = newCost;
                        graph.Append(nextState, currentState);
                    }
                } else {
                    float nextCost = currentCost + GetActionCost(action);
                    // Additional cost from cost map
                    nextCost += voxelMap.GetWeigth(nextState);
                    visitedStates.Add(nextState, nextCost);
                    float queueCost = nextCost + GetHeuristics(goalStateDescrete - nextState);
                    Insert(queueCost, nextState);
                    graph.Append(nextState, currentState);
                }
            }
        }
        return new List<Vector3Int>();
    }
    public void ShowPlan(List<Vector3> plan){
        foreach (var item in planStates)
            Destroy(item.gameObject);
        planStates.Clear();
        // planStates.Add(Instantiate(startObj, voxelMap.GetContinuousState(startStateDescrete), Quaternion.identity, transform));
        // planStates.Add(Instantiate(goalObj, voxelMap.GetContinuousState(goalStateDescrete), Quaternion.identity, transform));
        if (plan.Count > 0){
            foreach (var pos in plan){
                Transform newStateObj = Instantiate(visitedObj, pos, Quaternion.identity, transform);
                planStates.Add(newStateObj);
            }
        } else {
            foreach (var pos in visitedStates){
                Transform newStateObj = Instantiate(visitedObj, voxelMap.GetContinuousState(pos.Key), Quaternion.identity, transform);
                newStateObj.gameObject.name = pos.Key.ToString();
                if (pos.Value == Mathf.Infinity)
                    newStateObj.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
                planStates.Add(newStateObj);
            }
        }
    }
    public List<Vector3> ConvertPlanToCont(List<Vector3Int> plan){
        List<Vector3> res = new List<Vector3>();
        foreach (var item in plan){
            res.Add(voxelMap.GetContinuousState(item));
        }
        return res;
    }
    public Vector3Int GetDescrete(Vector3 state){
        return voxelMap.GetDescreteState(state);
    }
    public void SetWieght(Vector3Int vD, float cost){
        voxelMap.SetWeight(vD, cost);
    }
}
