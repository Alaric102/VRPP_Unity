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
    private List<Vector3> requestedStates = new List<Vector3>();
    public float meanRobotLevel = 0.25f;
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
    }

    private void DrawGlobalPlan(){
        globalPlanLine.positionCount = globalPath.Count;
        for (int i = 0; i < globalPath.Count; ++i){
            globalPlanLine.SetPosition(i, globalPath[i]);
        }
    }

    private void ProcessRequests(){
        if (requestedStates.Count > 0){
            
        }
    }

    void Update()
    {
        DrawGlobalPlan();
        ProcessRequests();
        if(requestedStates.Count > 0){
            // requestedState.gameObject.SetActive(true);
            // Vector3 v = requestedStates[0];
            // requestedStates.RemoveAt(0);
            // requestedState.position = v;
       
            // RaycastHit hitFrontRight, hitFrontLeft, hitRearRight, hitRearLeft;
            // bool isFrontRight = Physics.Raycast(body.position, 
            //     -body.transform.forward + body.transform.up + body.transform.right, 
            //     out hitFrontRight, Mathf.Infinity, ~6);
            // bool isRearRight = Physics.Raycast(body.position, 
            //     -body.transform.forward + body.transform.up - body.transform.right, 
            //     out hitRearRight, Mathf.Infinity, ~6);
            // bool isFronLeft = Physics.Raycast(body.position, 
            //     -body.transform.forward - body.transform.up + body.transform.right, 
            //     out hitFrontLeft, Mathf.Infinity, ~6);
            // bool isRearLeft = Physics.Raycast(body.position, 
            //     -body.transform.forward - body.transform.up - body.transform.right, 
            //     out hitRearLeft, Mathf.Infinity, ~6);
            
            // if (isFrontRight && isRearRight && isFronLeft && isRearLeft){
            //     Debug.DrawRay(body.position, 
            //         -body.transform.forward + body.transform.up + body.transform.right,
            //         Color.yellow);
            //     Debug.DrawRay(hitFrontRight.point, body.transform.forward * meanRobotLevel, Color.red);
            //     Vector3 pointFrontRight = hitFrontRight.point + body.transform.forward * meanRobotLevel;

            //     Debug.DrawRay(body.position, 
            //         -body.transform.forward + body.transform.up - body.transform.right,
            //         Color.yellow);
            //     Debug.DrawRay(hitRearRight.point, body.transform.forward * meanRobotLevel, Color.red);
            //     Vector3 pointRearRight = hitRearRight.point + body.transform.forward * meanRobotLevel;

            //     Debug.DrawRay(body.position, 
            //         -body.transform.forward - body.transform.up + body.transform.right,
            //         Color.yellow);
            //     Debug.DrawRay(hitFrontLeft.point, body.transform.forward * meanRobotLevel, Color.red);
            //     Vector3 pointFrontLeft = hitFrontLeft.point + body.transform.forward * meanRobotLevel;

            //     Debug.DrawRay(body.position, 
            //         -body.transform.forward - body.transform.up - body.transform.right,
            //         Color.yellow);
            //     Debug.DrawRay(hitRearLeft.point, body.transform.forward * meanRobotLevel, Color.red);
            //     Vector3 pointRearLeft = hitRearLeft.point + body.transform.forward * meanRobotLevel;
                
            //     Vector3 middlePoint = (pointFrontRight + pointFrontLeft + pointRearLeft + pointRearRight) / 4.0f;
            //     Debug.DrawLine(pointFrontRight, middlePoint, Color.black);
            //     Debug.DrawLine(pointRearRight, middlePoint, Color.black);
            //     Debug.DrawLine(pointFrontLeft, middlePoint, Color.black);
            //     Debug.DrawLine(pointRearLeft, middlePoint, Color.black);

            //     Vector3 relativeForward = (pointFrontLeft + pointFrontRight) / 2.0f - middlePoint;
            //     Debug.DrawLine(middlePoint, middlePoint + relativeForward, Color.blue);

            //     Vector3 relativeRight = (pointRearRight + pointFrontRight) / 2.0f - middlePoint;
            //     Debug.DrawLine(middlePoint, middlePoint + relativeRight, Color.green);

            //     Vector3 relativeUp = Vector3.Cross(relativeForward, relativeRight) * 10.0f;
            //     Debug.DrawLine(middlePoint, middlePoint + relativeUp, Color.red);

            //     Quaternion rotationForward = Quaternion.FromToRotation(requestedState.forward, relativeForward);
            //     Quaternion rotationUp = Quaternion.FromToRotation(requestedState.up, relativeUp);
            //     Quaternion rotationRight = Quaternion.FromToRotation(requestedState.right, relativeRight);
            //     Quaternion totalRotation = rotationForward * rotationUp * rotationRight;
            //     requestedState.rotation *= totalRotation;
                
            //     Debug.DrawLine(middlePoint, middlePoint + requestedState.forward, Color.blue);
            //     Debug.DrawLine(middlePoint, middlePoint + requestedState.right, Color.green);
            //     Debug.DrawLine(middlePoint, middlePoint + requestedState.up, Color.red);
                
            //     if (transform.childCount < 100){
            //         Instantiate(requestedState, requestedState.position, requestedState.rotation, transform);
            //     }
            //     Debug.Log("Rotation: " + totalRotation.eulerAngles.x.ToString() + ", " + totalRotation.eulerAngles.y.ToString() + ", " + totalRotation.eulerAngles.z.ToString());
                
            //     socketBridge.SendRequestedState(requestedState.position, wrapAngle(requestedState.rotation.eulerAngles), true);
            // } else {
            //     socketBridge.SendRequestedState(requestedState.position, Vector3.zero, false);
            // }
        }
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
        requestedStates.Add(v);
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
