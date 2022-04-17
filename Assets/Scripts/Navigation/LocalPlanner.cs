using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LocalPlanner : MonoBehaviour {
    public VoxelMap voxelMap = null;
    public Transform runnerState = null;
    public float dStep = 0.05f;
    private List<Transform> localStates = new List<Transform>();
    private Dictionary<Vector3, List<Transform>> goalToStates = new Dictionary<Vector3, List<Transform>>();
    private Queue<Transform> toReachStates = new Queue<Transform>();
    private List<Transform> reachedStates = new List<Transform>();
    private Transform currentState, globalStatePrefab;
    private bool isPlanning = false;
    private GlobalPlanner gPlanner = null;
    public Vector3 deviation = Vector3.zero;
    private Vector3 goalPose = Vector3.zero;
    private void ClearStates(){
        ClearStatesToAchieve();
        ClearAcheivedStates();
    }
    private void ClearAcheivedStates(){
        foreach (var item in reachedStates) {
            Destroy(item.gameObject);
        }
        reachedStates.Clear();
    }
    private void ClearStatesToAchieve(){
        while (toReachStates.Count > 0) {
            Destroy(toReachStates.Dequeue().gameObject);
        }
    }
    private void SetStateToAchieve(Vector3 pose){
        Transform newState = Instantiate(globalStatePrefab, pose, Quaternion.identity, transform);
        newState.gameObject.name = voxelMap.GetDiscreteState(pose).ToString() + 
            " to reach " + toReachStates.Count.ToString();
        newState.gameObject.SetActive(true);
        toReachStates.Enqueue(newState);
    }
    private void SetNextStateAchived(){
        var achieved = toReachStates.Dequeue();
        achieved.GetComponent<Renderer>().material.color = Color.green;
        achieved.gameObject.name = "achived " + reachedStates.Count.ToString();
        reachedStates.Add(achieved);
    }
    private Vector3 GetRelevantGoal(Vector3 pose){
        if (toReachStates.Count == 0)
            return currentState.position;
        if ((pose - toReachStates.Peek().position).magnitude < dStep){
            SetNextStateAchived();
            return GetRelevantGoal(pose);
        } else {
            return toReachStates.Peek().position;
        }
    }
    private Quaternion GetDeltaAlignedOrigin(Vector3 delta){
        Vector3 deltaFront = delta.normalized;
        Vector3 deltaUp = Vector3.ProjectOnPlane(Vector3.up, delta).normalized;
        Vector3 deltaRight = Vector3.Cross(deltaUp, deltaFront).normalized;  
        
        Quaternion a = Quaternion.FromToRotation(transform.forward, deltaFront);
        Quaternion b = Quaternion.FromToRotation(a * transform.right, deltaRight);
        Quaternion c = Quaternion.FromToRotation(b * a * transform.up, deltaUp);
        return c * b * a;
    }
    private void DrawDirection(Vector3 pos, Quaternion rot){
        Debug.DrawRay(pos, rot * Vector3.forward * 0.15f, Color.blue);
        Debug.DrawRay(pos, rot * Vector3.right * 0.15f, Color.red);
        Debug.DrawRay(pos, rot * Vector3.up * 0.15f, Color.green);
    }
    private void RequestNewPathAfter(Vector3 lastReached, Vector3 goal){
        ClearStatesToAchieve();
        deviation = Vector3.zero;
        var newAfterPath = gPlanner.GetGlobalPlan(lastReached, goal);
        foreach (var pose in newAfterPath) {
            SetStateToAchieve(pose);
        }
    }
    private Transform ForgetBranch(Vector3 goalKey){
        if (goalToStates.ContainsKey(goalKey)){
            foreach (var item in goalToStates[goalKey]) {
                Destroy(item.gameObject);
            }
            goalToStates.Remove(goalKey);
        }
        var lastReached = reachedStates[reachedStates.Count - 1].position;
        return goalToStates[lastReached][goalToStates[lastReached].Count - 1];
    }

    void Awake(){
        gPlanner = transform.GetComponent<GlobalPlanner>();
        globalStatePrefab = transform.GetChild(2);
    }
    void Update() {
        if (isPlanning){
            Vector3 relevantGoal = GetRelevantGoal(currentState.position);
            Vector3 currentGoalPose = relevantGoal + deviation;
            Vector3 deltaPose = currentGoalPose - currentState.position;
            Quaternion deltaDirection = GetDeltaAlignedOrigin(deltaPose);
            Quaternion deltaRotation = deltaDirection * Quaternion.Inverse(currentState.rotation);
            { // Draw deltaDirection origin
                DrawDirection(currentState.position, deltaDirection);
                Debug.DrawRay(currentState.position, deltaRotation.eulerAngles, Color.white);
                Debug.DrawLine(currentState.position, currentGoalPose, Color.yellow);
                // Debug.Log("deltaPose: " + deltaPose.magnitude + "\n" + 
                //     "deltaRotation.eulerAngles: " + deltaRotation.eulerAngles);
            }
            
            // Calculate mean clamped delta of tarnsition and rotation by each leg
            Robot currentRobot = currentState.GetComponent<Robot>();
            List<Tuple<Vector3, Quaternion>> legMovements = 
                currentRobot.GetLegMovements(deltaPose, deltaRotation);
            Vector3 meanTransition = Vector3.zero;
            Quaternion meanRotation = Quaternion.identity;
            foreach (var item in legMovements) {
                meanTransition = Vector3.Lerp(meanTransition, item.Item1, 0.5f);
                meanRotation = Quaternion.Lerp(meanRotation, item.Item2, 0.5f);
            }
            {
                Debug.DrawRay(currentState.position, meanTransition);
                Debug.DrawRay(currentState.position, meanRotation.eulerAngles);
            }

            // Propagate to next state
            Transform nextState = Instantiate(runnerState, currentState.position + meanTransition, 
                meanRotation * currentState.rotation,  transform);
            Robot nextRobot = nextState.GetComponent<Robot>();

            // Check propagated state
            Vector3 newDeviation = Vector3.ClampMagnitude(nextRobot.IsPropagatableMovement(), dStep);
            if (newDeviation != Vector3.zero){
                deviation += newDeviation;
                Debug.Log("Impossible to propagate.\n" +
                    "newDeviation: " + newDeviation + 
                    ", total deviation: " + deviation);
                Debug.DrawRay(nextState.position, deviation, Color.black);

                if (voxelMap.GetDiscreteState(relevantGoal) != 
                    voxelMap.GetDiscreteState(currentGoalPose)){   
                        Vector3Int additionalAction = voxelMap.GetDiscreteState(currentGoalPose) - 
                            voxelMap.GetDiscreteState(relevantGoal);

                        Debug.Log("Inform gPlanner about large displacement.\n" + 
                            voxelMap.GetDiscreteState(relevantGoal) + " -> " + 
                            voxelMap.GetDiscreteState(currentGoalPose) + "\n" +
                            additionalAction);

                        currentState = ForgetBranch(relevantGoal); currentState.gameObject.SetActive(true);
                        voxelMap.SetWeight(voxelMap.GetDiscreteState(relevantGoal), 1.0f);
                        voxelMap.SetAction(voxelMap.GetDiscreteState(relevantGoal), additionalAction);
                        RequestNewPathAfter(currentState.position, goalPose);
                        // 
                }
                Destroy(nextState.gameObject);
            } else {
                currentState.gameObject.SetActive(false);
                if (goalToStates.ContainsKey(relevantGoal)){
                    goalToStates[relevantGoal].Add(nextState);
                } else {
                    goalToStates.Add(relevantGoal, new List<Transform>{nextState});
                }
                nextState.gameObject.name = voxelMap.GetDiscreteState(relevantGoal).ToString() + 
                    " local " + (goalToStates[relevantGoal].Count - 1).ToString();
                currentState = nextState;
                deviation = Vector3.zero;
            }
            
        }
    }
    public void GetPath(Transform start, Transform goal){
        ClearStates();
        goalPose = goal.position;
        var globalPathToReach = gPlanner.GetGlobalPlan(start.position, goal.position);
        foreach (var pose in globalPathToReach) {
            SetStateToAchieve(pose);
        }
                
        currentState = Instantiate(runnerState, start.position, start.rotation, transform);        
        Vector3 currentGoalPose = GetRelevantGoal(currentState.position);
        goalToStates.Add(currentGoalPose, new List<Transform>{currentState});
        
        currentState.gameObject.name = "local " + (goalToStates[currentGoalPose].Count - 1).ToString();
        isPlanning = true;
    }
}
