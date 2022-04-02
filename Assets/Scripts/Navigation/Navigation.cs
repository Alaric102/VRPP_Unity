using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Navigation : MonoBehaviour
{
    public SocketBridge socketBridge = null;
    private Transform startState, goalState, obstacleState, requestedState, body;
    private Vector3 startRotation, goalRotation; // Required to store angles in range [-180, 180)
    private LineRenderer globalPlanLine = null;
    private List<Vector3> globalPath = new List<Vector3>();
    private List<Tuple<Vector3, Vector3>> requestedStates = new List<Tuple<Vector3, Vector3>>();
    private float displaysmentUp = 0.22f, displaysmentSlide = 0.22f, displaysmentFront = 0.22f;
    private float maxStepLength = 0.08f, goalDistanceTh = 0.0f;
    private int pathCounter = 0;
    private Vector3 currentPose, currentGoalPose;
    private List<Transform> sampledStates = new List<Transform>();
    void Awake() {
        // find child objects of Navigation
        startState = transform.GetChild(0);
        goalState = transform.GetChild(1);
        requestedState = transform.GetChild(2);
        obstacleState = transform.GetChild(3);

        body = requestedState.GetChild(0);
        globalPlanLine = transform.GetComponent<LineRenderer>();
        obstacleState.gameObject.SetActive(false);
    }
    void Start()
    {
        // requestedState.gameObject.SetActive(false);
        // globalPlanLine.positionCount = 0;
        // // removing after debuggin
        // globalPath.Add(new Vector3 (3.609375f,   0.02846878f, 6.8853152f));
        // globalPath.Add(new Vector3 (3.465f,      0.02846878f, 6.5723464f));
        // globalPath.Add(new Vector3 (3.320625f,   0.02846878f, 6.2593776f));
        // globalPath.Add(new Vector3 (2.8875f,     0.02846878f, 6.2593776f));
        // globalPath.Add(new Vector3 (2.02125f,    0.02846878f, 4.3815648f));
        // globalPath.Add(new Vector3 (1.299375f,   0.02846878f, 4.3815648f));
        // globalPath.Add(new Vector3 (1.010625f,   0.02846878f, 3.7556272f));
        // globalPath.Add(new Vector3 (0.5775f,     0.02846878f, 3.7556272f));
        // globalPath.Add(new Vector3 (0.433125f,   0.20581253f, 3.7556272f));
        // globalPath.Add(new Vector3 (0.433125f,   0.41862503f, 3.4426584f));
        // globalPath.Add(new Vector3 (0.144375f,   0.41862503f, 2.8167208f));
        // globalPath.Add(new Vector3 (0.144375f,   0.20581253f, 2.1907832f));
        // globalPath.Add(new Vector3 (0.144375f,   0.02846878f, 1.8778144f));
        // globalPath.Add(new Vector3 (0.144375f,   0.09940628f, 1.5648456f));
        // globalPath.Add(new Vector3 (-0.28875f,     0.09940628f,  0.6259392f));
        // globalPath.Add(new Vector3 (-0.28875f,     0.02846878f,  0.3129704f));
        // globalPath.Add(new Vector3 (-0.721875f,    0.02846878f, -0.625936f));
        // globalPath.Add(new Vector3 (-0.721875f,    0.09940628f, -0.9389048f));
        // globalPath.Add(new Vector3 (-1.155f,       0.09940628f, -1.8778112f));
        // globalPath.Add(new Vector3 (-1.155f,       0.02846878f, -2.19078f));
        // globalPath.Add(new Vector3 (-2.02125f,     0.02846878f, -4.0685928f));
        // globalPath.Add(new Vector3 (-2.743125f,    0.02846878f, -4.0685928f));
        // globalPath.Add(new Vector3 (-3.75375f,     0.02846878f, -6.2593744f));    
        // for (int i = 0; i < globalPath.Count - 1; i++) {
        //     Vector3 currentGoal = globalPath[i];
        //     Vector3 nextGoal = globalPath[i + 1];
        //     Vector3 delta = globalPath[i + 1] - currentGoal;
        //     Quaternion targetRotation = GetLocalOrigin(delta);

        //     Debug.DrawRay(currentGoal, targetRotation*Vector3.forward * 0.15f, Color.blue);
        //     Debug.DrawRay(currentGoal, targetRotation*Vector3.up * 0.15f, Color.green);
        //     Debug.DrawRay(currentGoal, targetRotation*Vector3.right * 0.15f, Color.red);
            
        //     Vector3 hShift = GetHorizonDisplacement(currentGoal, targetRotation);
        //     globalPath[i] += hShift;
        // }   
        // DrawGlobalPlan();

        // currentPose = startState.position;
    }
    private void DrawGlobalPlan(){
        globalPlanLine.positionCount = globalPath.Count;
        for (int i = 0; i < globalPath.Count; ++i){
            globalPlanLine.SetPosition(i, globalPath[i]);
        }
    }
    private Quaternion GetLocalOrigin(Vector3 delta){
        Vector3 deltaFront = delta.normalized;
        Vector3 deltaUp = Vector3.ProjectOnPlane(Vector3.up, delta).normalized;
        Vector3 deltaRight = Vector3.Cross(deltaUp, deltaFront).normalized;  
        
        Quaternion a = Quaternion.FromToRotation(transform.forward, deltaFront);
        Quaternion b = Quaternion.FromToRotation(a * transform.right, deltaRight);
        Quaternion c = Quaternion.FromToRotation(b * a * transform.up, deltaUp);
        return c * b * a;
    }
    private Vector3 GetNextGlobalGoal(Vector3 currentPose){
        Debug.Log("Distance to goal: " + (globalPath[0] - currentPose).magnitude);
        if ((globalPath[0] - currentPose).magnitude > maxStepLength * 2.0f)
            return globalPath[0];
        else if (globalPath.Count > 1)
            globalPath.RemoveAt(0);
        return globalPath[0];
    }
    private Vector3 GetHorizonDisplacement(Vector3 pos, Quaternion rot){
        List<Vector3> rayDirections = new List<Vector3>();
        for (float angle = 0.0f; angle < 2.0f * Mathf.PI; angle += 0.1f){
            Quaternion yRotation = Quaternion.Euler(0.0f, Mathf.Rad2Deg * angle, 0.0f);
            rayDirections.Add(rot * yRotation * Vector3.forward * 0.3f);
        }

        List<Tuple<Vector3, Vector3>> rayHitData = new List<Tuple<Vector3, Vector3>>(rayDirections.Count);
        foreach (Vector3 dir in rayDirections) {
            // Debug.DrawRay(pos, dir, Color.gray);
            RaycastHit hit;
            if (Physics.Raycast(pos, dir, out hit, dir.magnitude, ~0)){
                rayHitData.Add( new Tuple<Vector3, Vector3>(hit.point, hit.normal) );
            } else {
                rayHitData.Add( new Tuple<Vector3, Vector3>(Vector3.zero, Vector3.zero) );
            }
        }

        List<Vector3> nonZeroDisplacements = new List<Vector3>();
        for (int id = 0; id < rayDirections.Count; ++id){
            Vector3 hitPoint = rayHitData[id].Item1;
            Vector3 hitNormal = rayHitData[id].Item2;
            Vector3 displacement = hitNormal * Mathf.Abs((pos - hitPoint).magnitude - rayDirections[id].magnitude);
            Debug.DrawRay(hitPoint, displacement, Color.white);
            if (displacement != Vector3.zero){
                nonZeroDisplacements.Add(displacement);
            }
        }

        Vector3 res = Vector3.zero;
        if (nonZeroDisplacements.Count > 0){
            foreach (Vector3 v in nonZeroDisplacements)
                res += v;
            res /= nonZeroDisplacements.Count;
        }
        return res;
    }
    void Update() {
        // DrawGlobalPlan();
        // // Get next goal state
        // currentGoalPose = GetNextGlobalGoal(currentPose);

        // // Calculate direction to goal state from current state
        // Vector3 currentGoalDelta = currentGoalPose - currentPose;
        // Quaternion currentTargetRotation = GetLocalOrigin(currentGoalDelta);
        // Vector3 currentMeanPose = currentPose + currentTargetRotation*Vector3.up * displaysmentUp;
        // // Draw directed origin of current mean pose
        // Debug.DrawRay(currentMeanPose, currentTargetRotation*Vector3.forward * 0.15f, Color.gray);
        // Debug.DrawRay(currentMeanPose, currentTargetRotation*Vector3.up * 0.15f, Color.gray);
        // Debug.DrawRay(currentMeanPose, currentTargetRotation*Vector3.right * 0.15f, Color.gray);

        // // Predict next state
        // Vector3 delta = Vector3.ClampMagnitude(currentGoalDelta, maxStepLength*2.0f);
        // Vector3 nextPose = currentPose + delta;
        // Vector3 nextMeanPose  = nextPose + currentTargetRotation*Vector3.up * displaysmentUp;
        // // Draw directed origin of current mean pose and delta
        // Debug.DrawRay(currentMeanPose, delta, Color.red);
        // Debug.DrawRay(nextMeanPose, currentTargetRotation*Vector3.forward * 0.15f, Color.red);
        // Debug.DrawRay(nextMeanPose, currentTargetRotation*Vector3.up * 0.15f, Color.red);
        // Debug.DrawRay(nextMeanPose, currentTargetRotation*Vector3.right * 0.15f, Color.red);

        // // Calculate needed shift
        // Vector3 hShift = GetHorizonDisplacement(nextMeanPose, currentTargetRotation);
        // delta = Vector3.ClampMagnitude(delta + hShift, maxStepLength*2.0f);
        // Quaternion nextTargetRotation = GetLocalOrigin(delta);
        // nextPose = currentPose + delta;
        // nextMeanPose  = nextPose + nextTargetRotation*Vector3.up * displaysmentUp;
        
        // // Draw directed origin of next mean pose and delta
        // Debug.DrawRay(currentMeanPose, delta, Color.green);
        // Debug.DrawRay(nextMeanPose, nextTargetRotation*Vector3.forward * 0.15f, Color.green);
        // Debug.DrawRay(nextMeanPose, nextTargetRotation*Vector3.up * 0.15f, Color.green);
        // Debug.DrawRay(nextMeanPose, nextTargetRotation*Vector3.right * 0.15f, Color.green);
        
        // if (sampledStates.Count > 0){
        //     Destroy(sampledStates[0].gameObject);
        //     sampledStates.RemoveAt(0);
        // }
        // Transform newObj = Instantiate(requestedState, nextMeanPose, nextTargetRotation, transform);
        // newObj.gameObject.SetActive(true);
        // sampledStates.Add(newObj);
        
        // Robot newRobot = newObj.GetComponent<Robot>();
        // Quaternion xRotation = newRobot.GetSurfaceNorm();
        // Vector3 vShift = newRobot.GetSurfaceShift(displaysmentUp);
        // nextTargetRotation = Quaternion.Lerp(nextTargetRotation, nextTargetRotation*xRotation, 0.5f);
        // nextMeanPose += vShift;
        // newObj.position = nextMeanPose;
        // newObj.rotation = nextTargetRotation;

        // Debug.DrawRay(nextMeanPose, nextTargetRotation*Vector3.forward * 0.15f, Color.blue);
        // Debug.DrawRay(nextMeanPose, nextTargetRotation*Vector3.up * 0.15f, Color.blue);
        // Debug.DrawRay(nextMeanPose, nextTargetRotation*Vector3.right * 0.15f, Color.blue);

        // newRobot.GetStableFoot(delta, pathCounter);

        // currentPose = nextPose;
        // pathCounter = (pathCounter + 1) % 4;

    }
    public Transform GetStartState(){ // Get activated star state transform
        startState.gameObject.SetActive(true);
        return startState;
    }
    public Transform GetGoalState(){ // Get activated star state transform
        goalState.gameObject.SetActive(true);
        return goalState;
    }
    public Transform GetObstacleState(){ // Get activated obstacle transform with disabled collider
        obstacleState.gameObject.SetActive(true);
        obstacleState.GetChild(0).GetComponent<Collider>().enabled = false;
        return obstacleState;
    } 
    public void GenerateObstacle(){ // Generate new obstacle from prefab with activated collider
        Transform newObst = Instantiate(obstacleState, obstacleState.position, obstacleState.rotation, transform);
        newObst.GetChild(0).GetComponent<Collider>().enabled = true;
    } 
    public void StartPlanning(){
        // Send start and goal states
        Quaternion q = Quaternion.Euler(startRotation);
        socketBridge.SendStartPoint(startState.position, startRotation);
        q = Quaternion.Euler(goalRotation);
        socketBridge.SendGoalPoint(goalState.position, goalRotation);
        // Request plan
        socketBridge.RequestPlan();
    }
    public void SetGlobalPlan(List<Vector3> path){
        globalPath = path;
    }
    public void CheckStateCollision(Vector3 v){
        Debug.Log("Request: " + v.x.ToString() + ", " + v.y.ToString() + ", " + v.z.ToString());
        // requestedStates.Add(v);
    }
    public void SetActiveStates(bool isActive){
        startState.gameObject.SetActive(isActive);
        goalState.gameObject.SetActive(isActive);
    }
    private Vector3 wrapAngle(Vector3 r){
        Vector3 res = new Vector3(-180.0f, -180.0f, -180.0f);
        res += r;
        if (res.x > 0.0f){
            res.x -= 180.0f;
        } else {
            res.x += 180.0f;
        }

        if (res.y > 0.0f){
            res.y -= 180.0f;
        } else {
            res.y += 180.0f;
        }

        if (res.z > 0.0f){
            res.z -= 180.0f;
        } else {
            res.z += 180.0f;
        }
        return res;
    }
    private Quaternion GetSafetyRotattion(Vector3 meanPose, Quaternion localOrigin){
        Vector3 fromFLHeap = meanPose + localOrigin * Vector3.forward * displaysmentFront - localOrigin * Vector3.right * displaysmentSlide;
        Vector3 fromFRHeap = meanPose + localOrigin * Vector3.forward * displaysmentFront + localOrigin * Vector3.right * displaysmentSlide;
        Vector3 fromRLHeap = meanPose -(localOrigin * Vector3.forward) * displaysmentFront - (localOrigin * Vector3.right) * displaysmentSlide;
        Vector3 fromRRHeap = meanPose -(localOrigin * Vector3.forward) * displaysmentFront + localOrigin * Vector3.right * displaysmentSlide;
        RaycastHit hitFLHeapSafe, hitFRHeapSafe, hitRLHeapSafe, hitRRHeapSafe;
        Quaternion displacement = Quaternion.identity;
        float FLMeanDistance = displaysmentUp;
        float FRMeanDistance = displaysmentUp;
        float RLMeanDistance = displaysmentUp;
        float RRMeanDistance = displaysmentUp;
        if (Physics.Raycast(fromFLHeap, -(localOrigin * Vector3.up), out hitFLHeapSafe, fromFLHeap.magnitude, ~0)){
            Debug.DrawLine(fromFLHeap, hitFLHeapSafe.point, Color.black);
            FLMeanDistance = (hitFLHeapSafe.point - fromFLHeap).magnitude;
        }
        if (Physics.Raycast(fromFRHeap,  -(localOrigin * Vector3.up), out hitFRHeapSafe, fromFRHeap.magnitude, ~0)){
            Debug.DrawLine(fromFRHeap, hitFRHeapSafe.point, Color.black);
            FRMeanDistance = (hitFRHeapSafe.point - fromFRHeap).magnitude;
        }
        if (Physics.Raycast(fromRLHeap,  -(localOrigin * Vector3.up), out hitRLHeapSafe, fromRLHeap.magnitude, ~0)){
            Debug.DrawLine(fromRLHeap, hitRLHeapSafe.point, Color.black);
            RLMeanDistance = (hitRLHeapSafe.point - fromRLHeap).magnitude;
        }
        if (Physics.Raycast(fromRRHeap,  -(localOrigin * Vector3.up), out hitRRHeapSafe, fromRRHeap.magnitude, ~0)){
            Debug.DrawLine(fromRRHeap, hitRRHeapSafe.point, Color.black);
            RRMeanDistance = (hitRRHeapSafe.point - fromRRHeap).magnitude;
        }

        Vector3 newRight_FrontSide = Vector3.right * displaysmentSlide + Vector3.up * (FLMeanDistance - FRMeanDistance);
        Vector3 newRight_RearSide = Vector3.right * displaysmentSlide + Vector3.up * (RLMeanDistance - RRMeanDistance);
        Quaternion rotFront1 = Quaternion.FromToRotation(Vector3.right * displaysmentSlide, newRight_FrontSide);
        Quaternion rotFront2 = Quaternion.FromToRotation(Vector3.right * displaysmentSlide, newRight_RearSide);
        Quaternion a = (rotFront1 * rotFront2).normalized;

        Vector3 newForward_RightSide = Vector3.forward * displaysmentFront + Vector3.up * (RRMeanDistance - FRMeanDistance);
        Vector3 newForward_LeftSide = Vector3.forward * displaysmentFront + Vector3.up * (RLMeanDistance - FLMeanDistance);
        Quaternion rotRight1 = Quaternion.FromToRotation(Vector3.forward * displaysmentFront, newForward_RightSide);
        Quaternion rotRight2 = Quaternion.FromToRotation(Vector3.forward * displaysmentFront, newForward_LeftSide);
        Quaternion b = Quaternion.Slerp(rotRight1, rotRight2, 0.5f);
        return a*b;
    }
}
