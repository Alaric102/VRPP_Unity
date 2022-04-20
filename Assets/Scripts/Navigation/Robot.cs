using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour {
    [Header("Common parameters")]
    private Transform body;
    private List<float> jointDistances = new List<float>(3);
    private List<Vector3> heapRelativePose = new List<Vector3>(4);
    public float discretization = 0.5f;
    public List<float> maxAngles = new List<float>{90.0f, 180.0f, 180.0f};
    public List<float> minAngles = new List<float>{-90.0f, -180.0f, -180.0f};
    public float minDistance = 0.1053526f, maxDistance = 0.3111828f;
    
    [Header("FL leg variables")]
    private List<Transform> FLjoints = new List<Transform>(4);
    
    [Header("FR leg variables")]
    private List<Transform> FRjoints = new List<Transform>(4);
    
    [Header("RL leg variables")]
    private List<Transform> RLjoints = new List<Transform>(4);
    
    [Header("RR leg variables")]
    private List<Transform> RRjoints = new List<Transform>(4);
    private List<Tuple<Vector3, Vector3>> collisionData = new List<Tuple<Vector3, Vector3>>();
    void Awake() {
        Initialize();
    }
    private void Initialize(){
        body = transform.GetChild(0);
        { // Define FL joints
            FLjoints.Add(body.GetChild(0));
            for (int id = 1; id < FLjoints.Capacity; ++id)
                FLjoints.Add(FLjoints[id - 1].GetChild(0));
        }
        { // Define FR joints
            FRjoints.Add(body.GetChild(1));
            for (int id = 1; id < FRjoints.Capacity; ++id)
                FRjoints.Add(FRjoints[id - 1].GetChild(0));
        }
        { // Define RL joints
            RLjoints.Add(body.GetChild(2));
            for (int id = 1; id < RLjoints.Capacity; ++id)
                RLjoints.Add(RLjoints[id - 1].GetChild(0));
        }
        { // Define RR joints
            RRjoints.Add(body.GetChild(3));
            for (int id = 1; id < RRjoints.Capacity; ++id)
                RRjoints.Add(RRjoints[id - 1].GetChild(0));
        }
        { // Define Relative Joint2 positions
            heapRelativePose = new List<Vector3>{ 
                FLjoints[1].position - transform.position, FRjoints[1].position - transform.position, 
                RLjoints[1].position - transform.position, RRjoints[1].position - transform.position};
        }
        { // Define joints length
            for (int id = 0; id < jointDistances.Capacity; ++id)
                jointDistances.Add(Vector3.Distance(FLjoints[id].position, FLjoints[id + 1].position));
        }
        maxDistance = Mathf.Sqrt( Mathf.Pow(jointDistances[0], 2.0f) + Mathf.Pow((jointDistances[1] + jointDistances[2]), 2.0f)); 
    }
    private float Joint1AngleLeftSide(Vector3 pos, float alpha, float betta){
        float angle = 0.0f;
        if (pos.x > 0.0f)
            angle = Mathf.PI - betta - alpha;
        else if (pos.x > -jointDistances[0])
            angle = betta - alpha;
        else
            angle = -(alpha - betta);
        angle *= Mathf.Rad2Deg;
        return angle;
    }
    private float Joint1AngleRightSide(Vector3 pos, float alpha, float betta){
        float angle = 0.0f;
        if (pos.x < 0.0f)
            angle = Mathf.PI - betta - alpha;
        else if (pos.x < jointDistances[0])
            angle = betta - alpha;
        else
            angle = -(alpha - betta);
        angle *= -Mathf.Rad2Deg;
        return angle;
    }
    private float GetJoint1Angle(Vector3 pos, Func<Vector3, float, float, float> SideFunction){
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y, 2.0f) + Mathf.Pow(pos.x, 2.0f));
        float conjugateJoint1Alpha = Mathf.Acos( Mathf.Abs(jointDistances[0])/pos_xy );
        float conjugateJoint1Betta = Mathf.Acos( Mathf.Abs(pos.x)/pos_xy );
        
        float angle = SideFunction(pos, conjugateJoint1Alpha, conjugateJoint1Betta);

        if (angle < minAngles[0] || angle > maxAngles[0])
            return float.NaN;
        return angle;
    }
    private float GetJoint2Angle(Vector3 pos){
        float angle = 0.0f;
        float pos_xy = Mathf.Sqrt( Mathf.Pow(pos.y, 2.0f) + Mathf.Pow(pos.x, 2.0f) );
        float h = Mathf.Sqrt(Mathf.Pow(pos_xy, 2.0f) - Mathf.Pow(jointDistances[0], 2.0f));
        float pos_joints23 = Mathf.Sqrt( Mathf.Pow(pos.z, 2.0f) + Mathf.Pow(h, 2.0f) );

        float conjugateJoint2 = Mathf.Acos( 
            (Mathf.Pow(jointDistances[2], 2.0f) - Mathf.Pow(jointDistances[1], 2.0f) - Mathf.Pow(pos_joints23, 2.0f))/
            (-2.0f * jointDistances[1] * pos_joints23));
        if (pos.z >= 0)
            conjugateJoint2 += Mathf.Acos( Mathf.Abs(pos.z)/pos_joints23 );
        else
            conjugateJoint2 += Mathf.PI - Mathf.Acos( Mathf.Abs(pos.z)/pos_joints23 );
        angle = -(Mathf.PI - conjugateJoint2) * Mathf.Rad2Deg;

        if (angle < minAngles[1] || angle > maxAngles[1])
            return float.NaN;
        return angle;
    }
    private float GetJoint3Angle(Vector3 pos){
        float pos_xy = Mathf.Sqrt( Mathf.Pow(pos.y, 2.0f) + Mathf.Pow(pos.x, 2.0f) );
        float h = Mathf.Sqrt( Mathf.Pow(pos_xy, 2.0f) - Mathf.Pow(jointDistances[0], 2.0f) );
        float pos_joints23 = Mathf.Sqrt( Mathf.Pow(pos.z, 2.0f) + Mathf.Pow(h, 2.0f) );

        float angle = Mathf.Acos( 
            (Mathf.Pow(pos_joints23, 2.0f) - Mathf.Pow(jointDistances[1], 2.0f) - Mathf.Pow(jointDistances[2], 2.0f))/
            (-2.0f * jointDistances[1] * jointDistances[2])) * Mathf.Rad2Deg;

        if (angle < minAngles[2] || angle > maxAngles[2])
            return float.NaN;
        return angle;
    }
    private List<float> ResolveInverseKinematicsFL(Vector3 pos){
        return new List<float>{ GetJoint1Angle(pos, Joint1AngleLeftSide), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsFR(Vector3 pos){
        return new List<float>{ GetJoint1Angle(pos, Joint1AngleRightSide), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsRL(Vector3 pos){
        return new List<float>{ GetJoint1Angle(pos, Joint1AngleLeftSide), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsRR(Vector3 pos){
        return new List<float>{ GetJoint1Angle(pos, Joint1AngleRightSide), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private Tuple<Vector3, Vector3> IsAnglesInCollision(List<Transform> joints, List<float> angles, bool isShow = false){
        // Check joint0
        joints[0].localRotation = Quaternion.Euler(angles[0], 0, 0);
        var jointCollision = InCollision(joints[0], isShow);
        if ( jointCollision.Item2 != Vector3.zero ){
            return jointCollision;
        }
        // Check joint1
        joints[1].localRotation = Quaternion.Euler(0, angles[1], 0);
        jointCollision = InCollision(joints[1], isShow);
        if ( jointCollision.Item2 != Vector3.zero ){
            return jointCollision;
        }         
        joints[2].localRotation = Quaternion.Euler(0, angles[2], 0);
        return new Tuple<Vector3, Vector3>(Vector3.zero, Vector3.zero);
    }
    private static int CompareByCross(Tuple<Vector3, Vector3> x, Tuple<Vector3, Vector3> y){
        // To sort by Vector3.Cross(norm1, delta1) < Vector3.Cross(norm2, delta2)
        Vector3 crossX = Vector3.Cross(x.Item2, x.Item1);
        Vector3 crossY = Vector3.Cross(y.Item2, y.Item1);
        return crossX.magnitude.CompareTo(crossY.magnitude);
    }
    private List<Tuple<Vector3, Vector3>> GetVectorPotentials(Vector3 pos, bool isShow = false){
        /* Creates a sphere raycast around pos. 
        Saves RaycastHit.point and RaycastHit.norm in List<Tuple<Vector3, Vector3>> rayHitData
        Sorts rayHitData by CompareByCross
        */
        List<Vector3> rayDirections = new List<Vector3>();
        for (float latitude = 0.0f; latitude < 2.0f * Mathf.PI; latitude += discretization){
            for (float longitude = Mathf.PI * 0.25f; longitude < Mathf.PI * 0.75f; longitude += discretization){
                Quaternion yRotation = Quaternion.Euler(Mathf.Rad2Deg * longitude, Mathf.Rad2Deg * latitude, 0.0f);
                rayDirections.Add(yRotation * Vector3.forward);
            }
        }

        List<Tuple<Vector3, Vector3>> rayHitData = new List<Tuple<Vector3, Vector3>>(rayDirections.Count);
        foreach (Vector3 dir in rayDirections) {
            RaycastHit hit;
            if (Physics.Raycast(pos, dir, out hit, Mathf.Infinity, ~(1 << 6))){
                rayHitData.Add( new Tuple<Vector3, Vector3>(hit.point - pos, hit.normal) );
                if (isShow) Debug.DrawLine(pos, hit.point, Color.gray);
            }
        }
        rayHitData.Sort(CompareByCross);
        return rayHitData;
    }
    private List<Tuple<Vector3, List<float>>> FindFeasibleLegSolutions(List<Transform> joints, 
        Func<Vector3, List<float>> InverseFunction, bool isShow = false){
        List<Tuple<Vector3, Vector3>> rayData = GetVectorPotentials(joints[1].position, false);
        List<Tuple<Vector3, List<float>>> feasibleSolutions = new List<Tuple<Vector3, List<float>>>();

        float filterAngle = 45.0f;
        foreach (var item in rayData) {
            // Don't use vertical surfaces
            float angle = Mathf.Abs( Vector3.Angle(Vector3.up, item.Item2));
            if (angle > filterAngle)
                continue;
            // Resolve Inverse Kinematics from Joint1
            Vector3 directionFromJoint1 = transform.InverseTransformVector(item.Item1 + (joints[1].position - joints[0].position));
            // Don't use far distances
            if (directionFromJoint1.magnitude >= maxDistance){
                continue;
            }
            List<float> angles = InverseFunction(directionFromJoint1);
            // Don't use signatures
            if (angles[0] == float.NaN || angles[1] == float.NaN || angles[2] == float.NaN){
                continue;
            }
            feasibleSolutions.Add(new Tuple<Vector3, List<float>>(transform.TransformVector(directionFromJoint1), angles));
            if (isShow) Debug.DrawRay(joints[0].position, transform.TransformVector(directionFromJoint1), Color.gray);
        }
        return feasibleSolutions;
    }
    private Tuple<bool, Vector3> FindObstacleFreeSolution(List<Transform> joints, 
        Func<Vector3, List<float>> InverseFunction, bool isShow = false){
        // Return true and direction if solution have found, otherwise return false
        var solutions = FindFeasibleLegSolutions(joints, InverseFunction, false);
        //if no solutions exist
        if (solutions.Count == 0){
            return new Tuple<bool, Vector3>(false, -Vector3.up);
            // return new Tuple<bool, Vector3>(false, Vector3.zero);
        }
        
        Vector3 meanNormal = Vector3.zero;
        int collisionCounter = 0;
        foreach (var solution in solutions) {
            var jointCollisionData = IsAnglesInCollision(joints, solution.Item2, false);
            if (jointCollisionData.Item2 != Vector3.zero){
                meanNormal += jointCollisionData.Item2; collisionCounter++;
                continue;
            }
            if (isShow) Debug.DrawRay(joints[0].position, solution.Item1, Color.green);
            return new Tuple<bool, Vector3>(true, solution.Item1);
        }
        //if no solution without collison
        return new Tuple<bool, Vector3>(false, meanNormal / collisionCounter);
    }
    public Vector3 IsPropagatableMovement(){
        var solutionFL = FindObstacleFreeSolution(FLjoints, ResolveInverseKinematicsFL);
        var solutionFR = FindObstacleFreeSolution(FRjoints, ResolveInverseKinematicsFR);
        var solutionRL = FindObstacleFreeSolution(RLjoints, ResolveInverseKinematicsRL);
        var solutionRR = FindObstacleFreeSolution(RRjoints, ResolveInverseKinematicsRR);
        bool isBodyInCollision = false;
        Tuple<Vector3, Vector3> bodyCollision = InCollision(body, false);
        if (bodyCollision.Item2 != Vector3.zero)
            isBodyInCollision = true;
        if (solutionFL.Item1 && solutionFR.Item1 && solutionRL.Item1 && solutionRR.Item1 && !isBodyInCollision){
            return Vector3.zero;
        }
        Vector3 deviation = (bodyCollision.Item2 + solutionFL.Item2 + solutionFR.Item2 + solutionRL.Item2 + solutionRR.Item2) / 4.0f;
        return deviation;
    }
    public List<Tuple<Vector3, Quaternion>> GetHeapMovements(Vector3 deltaTransition, Quaternion deltaRotation, float dStep = 0.05f){
        // Calculate heap translation and rotation movements
        List<Tuple<Vector3, Quaternion>> res = new List<Tuple<Vector3, Quaternion>>(4);
        foreach (var heapPose in heapRelativePose) {
            Vector3 deltaRot = Vector3.ClampMagnitude(deltaRotation * heapPose - heapPose, dStep);
            Vector3 deltaTrans = Vector3.ClampMagnitude(deltaTransition, dStep);
            res.Add(new Tuple<Vector3, Quaternion>(deltaTrans, Quaternion.FromToRotation(heapPose, heapPose + deltaRot)));
        }
        return res;
    }
    private Tuple<Vector3, Vector3> InCollision(Transform obj, bool isShow = false){
        /* Return mean collision point and min normal vector of collision
        */
        List<Vector3> verts = new List<Vector3>();
        BoxCollider b = obj.GetComponent<BoxCollider>();
        for (int x = -1; x < 2; x += 2)
            for (int y = -1; y < 2; y += 2)
                for (int z = -1; z < 2; z += 2){
                    Vector3 v = (b.center + new Vector3(x * b.size.x, y * b.size.y, z * b.size.z) * 0.5f);
                    verts.Add( obj.TransformPoint(v) );
                }

        Vector3 meanPoint = Vector3.zero; Vector3 meanNormal = Vector3.zero;
        int hitNumber = 0;
        for (int i = 0; i < verts.Count; i++) {
            for (int j = 0; j < verts.Count; j++) {
                if (i == j) continue;
                RaycastHit hit;
                Vector3 direction = verts[j] - verts[i];
                if (Physics.Raycast(verts[i], direction, out hit, direction.magnitude, ~(1 << 6))){
                    meanPoint += hit.point;
                    meanNormal += hit.normal;
                    hitNumber++;
                    // collisionData.Add(new Tuple<Vector3, Vector3>(hit.point, hit.normal));
                    // res = true;
                    if (isShow)
                        Debug.DrawLine(verts[i], hit.point, Color.red);
                } else if (isShow){
                    Debug.DrawRay(verts[i], direction, Color.green);
                }
            }
        }
        if (hitNumber > 0){
            meanPoint /= hitNumber;
            meanNormal /= hitNumber;
        }
        return new Tuple<Vector3, Vector3>(meanPoint, meanNormal);
    }
    public Vector3 GetDeviation(){
        Vector3 res = Vector3.zero;
        Vector3 meanPose = Vector3.zero;
        Vector3 meanNorm = Vector3.zero;
        foreach (var item in collisionData) {
            // Don't use vertical surfaces
            float angle = Mathf.Abs(Vector3.Angle(transform.up, item.Item2));
            // if ( angle < 45.0f || angle > 135.0f)
            //     continue;
            Debug.DrawRay(item.Item1, item.Item2 * 0.05f, Color.gray);
            meanPose = Vector3.Lerp(meanPose, item.Item1, 0.5f);
            meanNorm = Vector3.Lerp(meanNorm, item.Item2, 0.5f);
            // meanPose = (meanPose + item.Item1) * 0.5f;
            // meanNorm = (meanNorm + item.Item2) * 0.5f;
        }
        Debug.DrawRay(meanPose, meanNorm * (transform.position - meanPose).magnitude, Color.red);
        return meanNorm * (transform.position - meanPose).magnitude;
    }
}
