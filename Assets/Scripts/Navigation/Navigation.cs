using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Navigation : MonoBehaviour
{
    public SocketBridge socketBridge = null;
    private Transform startState, goalState, requestedState, body;
    private Vector3 startRotation, goalRotation; // Required to store angles in range [-180, 180)
    private LineRenderer globalPlanLine = null;
    private List<Vector3> globalPath = new List<Vector3>();
    private List<Tuple<Vector3, Vector3>> requestedStates = new List<Tuple<Vector3, Vector3>>();
    private float displaysmentUp = 0.25f, displaysmentSlide = 0.22f, displaysmentFront = 0.22f;
    private float normalStepLength = 0.20f;
    public float stepFactor = 1.0f;
    private int pathCounter = 0;
    private Vector3 currentPose, goalPose;
    private List<Transform> sampledStates = new List<Transform>();
    void Awake() {
        startState = transform.GetChild(0);
        goalState = transform.GetChild(1);
        requestedState = transform.GetChild(2);
        body = requestedState.GetChild(0);

        globalPlanLine = transform.GetComponent<LineRenderer>();
    }
    void Start()
    {
        requestedState.gameObject.SetActive(false);
        globalPlanLine.positionCount = 0;
        // removing after debuggin
        globalPath.Add(new Vector3 (3.609375f, 0.02846878f, 6.8853152f));
        globalPath.Add(new Vector3 (3.320625f, 0.02846878f, 6.2593776f));
        globalPath.Add(new Vector3 (2.8875f, 0.02846878f, 6.2593776f));
        globalPath.Add(new Vector3 (2.02125f, 0.02846878f, 4.3815648f));
        globalPath.Add(new Vector3 (1.299375f, 0.02846878f, 4.3815648f));
        globalPath.Add(new Vector3 (1.010625f, 0.02846878f, 3.7556272f));
        globalPath.Add(new Vector3 (0.5775f, 0.02846878f, 3.7556272f));
        globalPath.Add(new Vector3 (0.433125f, 0.20581253f, 3.7556272f));
        globalPath.Add(new Vector3 (0.28875f, 0.41862503f, 3.4426584f));
        globalPath.Add(new Vector3 (0.0f, 0.41862503f, 2.8167208f));
        globalPath.Add(new Vector3 (-0.28875f, 0.20581253f,  2.1907832f));
        globalPath.Add(new Vector3 (-0.433125f, 0.02846878f,  1.8778144f));
        globalPath.Add(new Vector3 (-0.5775f, 0.02846878f,  1.8778144f));
        globalPath.Add(new Vector3 (-1.7325f, 0.02846878f, -0.625936f));
        globalPath.Add(new Vector3 (-1.876875f, 0.09940628f, -0.9389048f));
        globalPath.Add(new Vector3 (-2.31f, 0.09940628f, -1.8778112f));
        globalPath.Add(new Vector3 (-2.454375f, 0.02846878f, -2.19078f));
        globalPath.Add(new Vector3 (-3.75375f, 0.02846878f, -5.0074992f));
        
        currentPose = startState.position;
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
    private Vector3 GetSafetyDisplacement(Vector3 meanPose, Quaternion localOrigin){
        Vector3 fromFLHeap = localOrigin * Vector3.forward * displaysmentFront - localOrigin * Vector3.right * displaysmentSlide;
        Vector3 fromFRHeap = localOrigin * Vector3.forward * displaysmentFront + localOrigin * Vector3.right * displaysmentSlide;
        Vector3 fromRLHeap = -(localOrigin * Vector3.forward) * displaysmentFront - (localOrigin * Vector3.right) * displaysmentSlide;
        Vector3 fromRRHeap = -(localOrigin * Vector3.forward) * displaysmentFront + localOrigin * Vector3.right * displaysmentSlide;
        RaycastHit hitFLHeapSafe, hitFRHeapSafe, hitRLHeapSafe, hitRRHeapSafe;
        Vector3 displacement = Vector3.zero;
        Vector3 projection = Vector3.zero;
        if (Physics.Raycast(meanPose, fromFLHeap, out hitFLHeapSafe, fromFLHeap.magnitude, ~0)){
            // projection = Vector3.Project(meanPose - hitFLHeapSafe.point, -(localOrigin * Vector3.right));
            // Debug.DrawRay(hitFLHeapSafe.point, projection, Color.yellow);
            // displacement += (localOrigin * Vector3.right) * (displaysmentSlide - projection.magnitude);

            Debug.DrawRay(hitFLHeapSafe.point, hitFLHeapSafe.normal, Color.red);
            displacement += hitFLHeapSafe.normal * (hitFLHeapSafe.point - meanPose).magnitude;
            // displacement += hitFLHeapSafe.normal * ((hitFLHeapSafe.point - meanPose).magnitude -
            //     Vector3.Magnitude( new Vector3(displaysmentFront, 0, displaysmentSlide)) );
        }
        if (Physics.Raycast(meanPose, fromFRHeap, out hitFRHeapSafe, fromFLHeap.magnitude, ~0)){
            // projection = Vector3.Project(meanPose - hitFRHeapSafe.point, -(localOrigin * Vector3.right));
            // Debug.DrawRay(hitFRHeapSafe.point, projection, Color.yellow);
            // displacement += (localOrigin * Vector3.right) * (displaysmentSlide - projection.magnitude);

            Debug.DrawRay(hitFRHeapSafe.point, hitFRHeapSafe.normal, Color.red);
            displacement += hitFRHeapSafe.normal * (hitFRHeapSafe.point - meanPose).magnitude;
            // displacement += hitFRHeapSafe.normal * ((hitFRHeapSafe.point - meanPose).magnitude -
            //     Vector3.Magnitude( new Vector3(displaysmentFront, 0, displaysmentSlide)) );
        }
        if (Physics.Raycast(meanPose, fromRLHeap, out hitRLHeapSafe, fromFLHeap.magnitude, ~0)){
            // projection = Vector3.Project(meanPose - hitRLHeapSafe.point, -(localOrigin * Vector3.right));
            // Debug.DrawRay(hitRLHeapSafe.point, projection, Color.yellow);
            // displacement += (localOrigin * Vector3.right) * (displaysmentSlide - projection.magnitude);

            Debug.DrawRay(hitRLHeapSafe.point, hitRLHeapSafe.normal, Color.red);
            displacement += hitRLHeapSafe.normal * (hitRLHeapSafe.point - meanPose).magnitude;
            // displacement += hitRLHeapSafe.normal * ((hitRLHeapSafe.point - meanPose).magnitude -
            //     Vector3.Magnitude( new Vector3(displaysmentFront, 0, displaysmentSlide)) );
        }
        if (Physics.Raycast(meanPose, fromRRHeap, out hitRRHeapSafe, fromFLHeap.magnitude, ~0)){
            // projection = Vector3.Project(meanPose - hitRRHeapSafe.point, -(localOrigin * Vector3.right));
            // Debug.DrawRay(hitRRHeapSafe.point, projection, Color.yellow);
            // displacement += (localOrigin * Vector3.right) * (displaysmentSlide - projection.magnitude);

            Debug.DrawRay(hitRRHeapSafe.point, hitRRHeapSafe.normal, Color.red);
            displacement += hitRRHeapSafe.normal * (hitRRHeapSafe.point - meanPose).magnitude;
            // displacement += hitRRHeapSafe.normal * ((hitRRHeapSafe.point - meanPose).magnitude -
            //     Vector3.Magnitude( new Vector3(displaysmentFront, 0, displaysmentSlide)) );
        }
        // return Vector3.ClampMagnitude(displacement, 1.5f*normalStepLength);
        return displacement;
    }
    private void ProcessRequests(){
        if (globalPath.Count == 0)
            return; 
        Vector3 goalPose = globalPath[0];               // Get next global goal
        Vector3 goalDelta = goalPose - currentPose;     // Calculate delta to Global Goal state
        if (goalDelta.magnitude < normalStepLength){    // If closer than in one step remove achieved global state
            globalPath.RemoveAt(0);
            return;
        }
        Vector3 nextPose = currentPose + goalDelta.normalized * normalStepLength * stepFactor;   // Propagated one step

        Quaternion localOrigin = GetLocalOrigin(goalDelta);
        Vector3 deltaFront = localOrigin * Vector3.forward;
        Vector3 deltaUp = localOrigin * Vector3.up; 
        Vector3 deltaRight = localOrigin * Vector3.right;
        Vector3 nextMeanPose  = nextPose + deltaUp * displaysmentUp;

        // Draw next state origin
        Debug.DrawRay(currentPose, goalDelta, Color.red);
        Debug.DrawRay(nextMeanPose, localOrigin*Vector3.forward * 0.15f, Color.red);
        Debug.DrawRay(nextMeanPose, localOrigin*Vector3.up * 0.15f, Color.red);
        Debug.DrawRay(nextMeanPose, localOrigin*Vector3.right * 0.15f, Color.red);


        Vector3 safetyShift = GetSafetyDisplacement(nextMeanPose, localOrigin);
        Vector3 goalDeltaSafe = goalDelta + safetyShift;
        Quaternion localOriginSafe = GetLocalOrigin(goalDeltaSafe);
        Vector3 nextPoseSafe = currentPose + goalDeltaSafe.normalized * normalStepLength * stepFactor;
        Vector3 nextMeanPoseSafe  = nextPoseSafe + localOriginSafe * Vector3.up * displaysmentUp;

        Debug.DrawRay(currentPose, goalDeltaSafe, Color.green);
        Debug.DrawRay(nextMeanPoseSafe, localOriginSafe*Vector3.forward * 0.15f, Color.green);
        Debug.DrawRay(nextMeanPoseSafe, localOriginSafe*Vector3.up * 0.15f, Color.green);
        Debug.DrawRay(nextMeanPoseSafe, localOriginSafe*Vector3.right * 0.15f, Color.green);

        Debug.Log("Safety: " + safetyShift.magnitude + ", step" + normalStepLength * stepFactor);

        if (sampledStates.Count > 100){
            Destroy(sampledStates[0].gameObject);
            sampledStates.RemoveAt(0);
        }
        Transform newObj = Instantiate(requestedState, nextPoseSafe, localOriginSafe, transform);
        newObj.gameObject.SetActive(true);
        sampledStates.Add(newObj);

        Robot newRobot = newObj.GetComponent<Robot>();
        currentPose = nextPoseSafe;
    }

    void Update()
    {
        DrawGlobalPlan();
        ProcessRequests();
    }
    public void SetStartState(ref Vector3 v, ref Vector3 r){
        startState.position = v;
        startState.rotation = Quaternion.Euler(r);
        startRotation = r;
    } // Set start State from Navigation Menu
    public void SetGoalState(ref Vector3 v, ref Vector3 r){ 
        goalState.position = v;
        goalState.rotation = Quaternion.Euler(r);
        goalRotation = r;
    } // Set goal State from Navigation Menu

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
}
