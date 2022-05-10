using System;
using System.Diagnostics;
using System.IO;
using System.Collections.Generic;
using UnityEngine;

public class LocalPlanner : MonoBehaviour {
    public VoxelMap voxelMap = null;
    public Transform robotPrefab = null;
    private GlobalPlanner gPlanner = null;
    private float dStep = 0.015f;
    private Dictionary<Vector3, List<Transform>> goalsToStates = new Dictionary<Vector3, List<Transform>>();
    private Queue<Transform> toAcheiveState = new Queue<Transform>();
    private List<Transform> reachedStates = new List<Transform>();
    private Transform currentState = null, startState = null, goalState = null, globalStatePrefab;
    private bool isPlanning = false, isFinished = false;
    private Vector3 deviation = Vector3.zero;
    private List<Vector3> deviations = new List<Vector3>();
    private int backTo = 0;
    private Stopwatch sw = new Stopwatch();
    private string label_ = "ildar";
    private int planningCounter = 0;
    private int replanningCounter = 0;
    private void ClearStates(){ // Clear all local planner states, achieved states and states to acheive
        ClearStatesToAchieve();
        ClearAcheivedStates();
        ClearLocalStates();
    }
    private void ClearLocalStates(){ // Clear all local planner states
        foreach (var goalToState in goalsToStates)
            foreach (var state in goalToState.Value)
                Destroy(state.gameObject);
        goalsToStates.Clear();
    }
    private void ClearAcheivedStates(){ // Clear all acheived states
        foreach (var item in reachedStates)
            Destroy(item.gameObject);
        reachedStates.Clear();
    }
    private void ClearStatesToAchieve(){ // Clear all state to acheive
        while (toAcheiveState.Count > 0)
            Destroy(toAcheiveState.Dequeue().gameObject);
    }
    private void AppendStateToAchieve(Vector3 pose){ // append pose to ToAcheive Queue

        Transform newState = Instantiate(globalStatePrefab, pose, Quaternion.identity, transform);
        toAcheiveState.Enqueue(newState);
        
        newState.gameObject.name = voxelMap.GetDiscreteState(pose).ToString() + 
            " toAcheive " + toAcheiveState.Count.ToString();
        newState.gameObject.SetActive(true);
    }
    private void AppendNextStateAchived(bool isShow = true){ // Pop() first from ToAcheive and Add to Acheived
        if (toAcheiveState.Count == 0)
            return;
        var achieved = toAcheiveState.Dequeue();
        reachedStates.Add(achieved);
        
        if (isShow){
            UnityEngine.Debug.Log("Acheived: " + achieved.gameObject.name);
            achieved.GetComponent<Renderer>().material.color = Color.green;
            achieved.gameObject.name = voxelMap.GetDiscreteState(achieved.position).ToString() + 
                " achived " + reachedStates.Count.ToString();
        }
    }
    private bool GetNewGlobalPlan(Transform start, Transform goal, string label){ // Clear previous ToAchieve states and append new
        ClearStatesToAchieve();
        var plan = gPlanner.GetGlobalPlan(start.position, goal.position, label + "_" + replanningCounter.ToString());
        replanningCounter++;
        if (plan.Count == 0)
            return false;
        foreach (var pose in plan)
            AppendStateToAchieve(pose);
        return true;
    }
    private Vector3 GetRelevantGoal(Transform state){ // Return closest goal and update Acheived and ToAcheive containers
        if (toAcheiveState.Count == 0)
            return goalState.position;
        return toAcheiveState.Peek().position;
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
        UnityEngine.Debug.DrawRay(pos, rot * Vector3.forward * 0.15f, Color.blue);
        UnityEngine.Debug.DrawRay(pos, rot * Vector3.right * 0.15f, Color.red);
        UnityEngine.Debug.DrawRay(pos, rot * Vector3.up * 0.15f, Color.green);
    }
    private Vector3 GetLastReachedGoal(int shift = 0){
        if (reachedStates.Count == 0){
            return startState.position;
        }
        if (reachedStates.Count < (1 + shift)){
            return reachedStates[0].position;
        }
        return reachedStates[reachedStates.Count - (1 + shift)].position;
    }
    private void ForgetBranchToGoal(Vector3 pose){
        if (goalsToStates.ContainsKey(pose)){
            foreach (var item in goalsToStates[pose]) {
                Destroy(item.gameObject);
            }
            goalsToStates.Remove(pose);
        }
    }
    private void ForgetReachedAfter(Vector3 v){
        for (int i = reachedStates.Count - 1; i >= 0; --i) {
            if (reachedStates[i].position == v)
                return;
            else {
                Destroy(reachedStates[i].gameObject);
                ForgetBranchToGoal(reachedStates[i].position);
                reachedStates.RemoveAt(i);
            }
        }
    }
    void Awake(){
        gPlanner = transform.GetComponent<GlobalPlanner>();
        globalStatePrefab = transform.GetChild(2);
        globalStatePrefab.gameObject.SetActive(false);
    }
    void Update() {
        if (isPlanning){
            sw.Start();
            Robot currentRobot = currentState.GetComponent<Robot>();
            Vector3 actualGoal = GetRelevantGoal(currentState);
            Vector3 correctedGoal = actualGoal + deviation;

            UnityEngine.Debug.DrawLine(currentState.position, actualGoal, Color.blue);
            UnityEngine.Debug.DrawLine(currentState.position, correctedGoal, Color.cyan);

            if (voxelMap.GetDiscreteState(actualGoal) != voxelMap.GetDiscreteState(correctedGoal)){  
                // UnityEngine.Debug.Log("Inform gPlanner about large displacement.\n" + 
                //     voxelMap.GetDiscreteState(actualGoal) + " -> " + voxelMap.GetDiscreteState(correctedGoal) );
                
                // Vector3 nextAfterBackUp = GetLastReachedGoal( (backTo == 0) ? 0 : backTo - 1 );
                // Vector3 backUpState = GetLastReachedGoal(backTo++);
                // 
                // gPlanner.SetActionCost(actionFromBackup, voxelMap.GetDiscreteState(backUpState), deviation.magnitude);
                voxelMap.SetWeight(voxelMap.GetDiscreteState(actualGoal), deviation.magnitude);

                Vector3 lastReached = GetLastReachedGoal(0);
                Vector3Int action = voxelMap.GetDiscreteState(actualGoal) - voxelMap.GetDiscreteState(lastReached);
                gPlanner.SetActionCost(action, voxelMap.GetDiscreteState(lastReached), deviation.magnitude);

                Vector3 backUpState = GetLastReachedGoal(backTo++);
                ForgetReachedAfter(backUpState); ForgetBranchToGoal(actualGoal);

                currentState = goalsToStates[backUpState][goalsToStates[backUpState].Count - 1];
                currentState.gameObject.SetActive(true);

                GetNewGlobalPlan(currentState, goalState, label_ + planningCounter.ToString());
                deviation = Vector3.zero;
                return;
            }

            Vector3 deltaPose = correctedGoal - currentState.position;
            if (deltaPose.magnitude < dStep){
                deviation = Vector3.zero;
                if (--backTo < 0) backTo = 0;
                AppendNextStateAchived(false);
            }
            Quaternion deltaDirection = GetDeltaAlignedOrigin(deltaPose);
            Quaternion deltaRotation = deltaDirection * Quaternion.Inverse(currentState.rotation);
          
            if (actualGoal == goalState.position){
                deltaRotation = goalState.rotation * Quaternion.Inverse(currentState.rotation);
                deltaRotation = Quaternion.Euler(wrapAngle(deltaRotation.eulerAngles));
            }
            { // Draw deltaDirection origin
                // DrawDirection(currentState.position, deltaDirection);
                // UnityEngine.Debug.DrawRay(currentState.position, deltaRotation.eulerAngles, Color.white);
                // UnityEngine.Debug.DrawRay(currentState.position, deltaPose, Color.yellow);
                // UnityEngine.Debug.Log("deltaPose: " + deltaPose.magnitude + "\n" + 
                //     "deltaRotation.eulerAngles: " + wrapAngle(deltaRotation.eulerAngles).magnitude);
            }
           

            // Clamp delta of tarnsition and rotation by max leg movements
            List<Tuple<Vector3, Quaternion>> heapMovements = currentRobot.GetHeapMovements(deltaPose, deltaRotation, dStep, true);
            Vector3 deltaTransitionClamped = heapMovements[0].Item1;
            Quaternion deltaRotationClamped = heapMovements[0].Item2;
            for (int i = 1; i < heapMovements.Count; i++){
                deltaTransitionClamped = Vector3.Lerp(deltaTransitionClamped, heapMovements[i].Item1, 0.5f);
                deltaRotationClamped = Quaternion.Lerp(deltaRotationClamped, heapMovements[i].Item2, 0.5f);
            }

            if (deltaTransitionClamped == Vector3.zero && deltaRotationClamped.eulerAngles == Vector3.zero){
                isPlanning = false;
                isFinished = true;
            }

            // Propagate to next state
            Transform nextState = Instantiate(robotPrefab, currentState.position + deltaTransitionClamped, 
                deltaRotationClamped * currentState.rotation,  transform);
            Robot nextRobot = nextState.GetComponent<Robot>();
            nextRobot.activeLeg = currentRobot.activeLeg;

            nextRobot.accDeltaStep = currentRobot.accDeltaStep;
            nextRobot.accActiveDelta = currentRobot.accActiveDelta;

            // Find prime leg directions from last motion
            List<Robot.Potential> lastPotentials =  currentRobot.resolvedPotentials;
            List<Vector3> nextDirections = nextRobot.GetLegDirectionsByMotion(heapMovements, lastPotentials);
            List<Vector3> lastDirections = new List<Vector3>();
            foreach (var item in lastPotentials){
                lastDirections.Add(item.GetPropagation());
            }
            // Check propagated state
            if (!nextRobot.IsPropagatable(nextDirections, lastDirections)){
                Vector3 newDeviation = Vector3.ClampMagnitude(nextRobot.GetDeviationVector(), dStep);
                deviations.Add(newDeviation);
                deviation += newDeviation;
                
                UnityEngine.Debug.DrawRay(nextState.position, deviation, Color.red);
                // UnityEngine.Debug.Log("Impossible to propagate." +
                //     " NewDeviation: " + newDeviation.magnitude + ", total deviation: " + deviation.magnitude);

                // voxelMap.SetWeight(voxelMap.GetDiscreteState(nextState.position), newDeviation.magnitude);

                // nextState is invalid for further propagation
                Destroy(nextState.gameObject);
            } else {
                deviations.Add(Vector3.zero);
                // Append next state to actualGoal
                if (goalsToStates.ContainsKey(actualGoal))
                    goalsToStates[actualGoal].Add(nextState);
                else
                    goalsToStates.Add(actualGoal, new List<Transform>{nextState});

                nextState.gameObject.name = voxelMap.GetDiscreteState(actualGoal).ToString() + 
                    " local " + (goalsToStates[actualGoal].Count - 1).ToString();
                currentState.gameObject.SetActive(false);
                currentState = nextState;
            }
            
            sw.Stop();
        } else if (isFinished){
            isFinished = false;
            sw.Stop();
            UnityEngine.Debug.Log(sw.ElapsedMilliseconds);
            SaveLog(label_, sw.ElapsedMilliseconds);
            // Navigation navigation = transform.GetComponent<Navigation>();
            // navigation.SetStartState(currentState);
        } else if (++planningCounter < 20){
            StartPlanning(startState, goalState);
        }
    }
    public void StartPlanning(Transform start, Transform goal, string label = "ildarTest2"){
        UnityEngine.Debug.Log("started: " + planningCounter);
        ClearStates();
        deviations.Clear();
        sw.Reset();
        replanningCounter = 0;
        backTo = 0;
        label_ = label;

        startState = start;
        goalState = goal;
        if (!GetNewGlobalPlan(start, goal, label_ + planningCounter.ToString())){
            return;
        }
        
        // Init currentState
        if (currentState != null)
            Destroy(currentState.gameObject);
        currentState = Instantiate(robotPrefab, start.position, start.rotation, transform);
        currentState.gameObject.SetActive(true);
        var currentRobot = currentState.GetComponent<Robot>();
        currentRobot.accDeltaStep.Add(Vector3.zero);
        currentRobot.accDeltaStep.Add(Vector3.zero);
        currentRobot.accDeltaStep.Add(Vector3.zero);
        currentRobot.accDeltaStep.Add(Vector3.zero);
        // Resolve leg positions
        currentRobot.activeLeg = -1;

        List<Vector3> lastResolved = new List<Vector3>{-Vector3.up, -Vector3.up, -Vector3.up, -Vector3.up};
        if (!currentRobot.IsPropagatable(lastResolved, lastResolved))
            return;
        List<Robot.Potential> resolvedPotentials = currentRobot.resolvedPotentials;
        currentRobot.activeLeg = 0;

        // Append initial currentState to first relevant goal
        Vector3 actualGoal = GetRelevantGoal(currentState);
        goalsToStates.Add(actualGoal, new List<Transform>{currentState});
        currentState.gameObject.name = voxelMap.GetDiscreteState(actualGoal).ToString() + 
                    " local " + (goalsToStates[actualGoal].Count - 1).ToString();
        
        isPlanning = true;
        isFinished = false;
    }

    public void Replan(Vector3 newPose, Vector3 oldPose){
        voxelMap.SetWeight(voxelMap.GetDiscreteState(oldPose), 10.0f);
        voxelMap.SetWeight(voxelMap.GetDiscreteState(newPose), -10.0f);
        StartPlanning(startState, goalState);
    }
    public void StopPlanning(){
        isPlanning = false;
    }
    public List<Vector3> GetGlobalPlan(){
        List<Vector3> res = new List<Vector3>();
        foreach (var item in reachedStates) {
            res.Add(item.position);
        }
        foreach (var item in toAcheiveState) {
            res.Add(item.position);
        }
        return res;
    }
    private Vector3 wrapAngle(Vector3 r){
        Vector3 res = new Vector3(-180.0f, -180.0f, -180.0f);
        res += r;
        res.x += (res.x > 0.0f) ? -180.0f : +180.0f;
        res.y += (res.y > 0.0f) ? -180.0f : +180.0f;
        res.z += (res.z > 0.0f) ? -180.0f : +180.0f;
        return res;
    }
    private void SaveLog(string label, long time){
        
        string _fullPath = "D:/catkin_ws/src/VRPP_ROS/launch/";
        StreamWriter file = new StreamWriter(_fullPath + label + planningCounter.ToString() + "_local.txt", append: false);
        file.Write("replanningCounter: " + replanningCounter + "\n");
        file.Write("time: " + time + "\n");
        foreach (var goal in goalsToStates){
            foreach (var state in goal.Value) {
                file.Write(FormatVector3(state.position) + ";" + 
                    FormatVector3(state.rotation.eulerAngles) + "\n");
            }
        }
        file.Close();

        file = new StreamWriter(_fullPath + label + planningCounter.ToString() + "_deviations.txt", append: false);
        foreach (var dev in deviations) {
            file.Write(FormatVector3(dev) + "\n");
        }
        file.Close();

        file = new StreamWriter(_fullPath + label + planningCounter.ToString() + "_angles.txt", append: false);;
        foreach (var goal in goalsToStates){
            foreach (var state in goal.Value) {
                Robot robot = state.GetComponent<Robot>();
                var angles = robot.GetAngles();
                string s = angles[0].ToString();
                for (int i = 1; i < angles.Count; i++)
                {
                    s += " " + angles[i].ToString();
                }
                file.Write(s + "\n");
            }
        }
        file.Close();
    }
    private string FormatVector3(Vector3 v){
        return v.x.ToString() + " " + v.y.ToString() + " " + v.z.ToString();
    }
}
