using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour {
    [Header("Common parameters")]
    private Transform body;
    private float lengthJoint1, lengthJoint2, lengthJoint3;
    public float discretization = 0.5f;
    public List<float> maxAngles = new List<float>{90.0f, 180.0f, 180.0f};
    public List<float> minAngles = new List<float>{-90.0f, -180.0f, -180.0f};
    private float minDistance = 0.1053526f, maxDistance = 0.3111828f;
    
    [Header("FL leg variables")]
    private Transform FLjoint1, FLjoint2, FLjoint3, FLEndEffector;
    public List<float> FLAngles = new List<float>{0.0f, 0.0f, 0.0f};
    
    [Header("FR leg variables")]
    private Transform FRjoint1, FRjoint2, FRjoint3, FREndEffector;
    public List<float> FRAngles = new List<float>{0.0f, 0.0f, 0.0f};
    
    [Header("RL leg variables")]
    private Transform RLjoint1, RLjoint2, RLjoint3, RLEndEffector;
    public List<float> RLAngles = new List<float>{0.0f, 0.0f, 0.0f};
    
    [Header("RR leg variables")]
    private Transform RRjoint1, RRjoint2, RRjoint3, RREndEffector;
    public List<float> RRAngles = new List<float>{0.0f, 0.0f, 0.0f};
    void Awake() {
        body = transform.GetChild(0);
        // Define FL joints
        FLjoint1 = body.GetChild(0);
        FLjoint2 = FLjoint1.GetChild(0);
        FLjoint3 = FLjoint2.GetChild(0);
        FLEndEffector = FLjoint3.GetChild(0);
        // Define FR joints
        FRjoint1 = body.GetChild(1);
        FRjoint2 = FRjoint1.GetChild(0);
        FRjoint3 = FRjoint2.GetChild(0);
        FREndEffector = FRjoint3.GetChild(0);
        // Define RL joints
        RLjoint1 = body.GetChild(2);
        RLjoint2 = RLjoint1.GetChild(0);
        RLjoint3 = RLjoint2.GetChild(0);
        RLEndEffector = RLjoint3.GetChild(0);
        // Define RR joints
        RRjoint1 = body.GetChild(3);
        RRjoint2 = RRjoint1.GetChild(0);
        RRjoint3 = RRjoint2.GetChild(0);
        RREndEffector = RRjoint3.GetChild(0);

        lengthJoint1 = Vector3.Distance(FLjoint1.position, FLjoint2.position);
        lengthJoint2 = Vector3.Distance(FLjoint2.position, FLjoint3.position);
        lengthJoint3 = Vector3.Distance(FLjoint3.position, FLEndEffector.position);
    }
    private float GetJoint1Angle_LeftLegs(Vector3 pos){
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y,2.0f) + Mathf.Pow(pos.x,2.0f));
        float conjugateJoint1Alpha = Mathf.Acos( Mathf.Abs(lengthJoint1)/pos_xy);
        float conjugateJoint1Betta = Mathf.Acos( Mathf.Abs(pos.x)/pos_xy);
        float angle = 0.0f;

        if (pos.x > 0.0f)
            angle = Mathf.PI - conjugateJoint1Betta - conjugateJoint1Alpha;
        else if (pos.x > -lengthJoint1)
            angle = conjugateJoint1Betta - conjugateJoint1Alpha;
        else
            angle = -(conjugateJoint1Alpha - conjugateJoint1Betta);
        
        angle *= Mathf.Rad2Deg;
        if (angle < minAngles[0])
            angle = minAngles[0];
        else if (angle > maxAngles[0])
            angle = maxAngles[0];
        return angle;
    }
    private float GetJoint1Angle_RightLegs(Vector3 pos){
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y,2.0f) + Mathf.Pow(pos.x,2.0f));
        float conjugateJoint1Alpha = Mathf.Acos( Mathf.Abs(lengthJoint1)/pos_xy);
        float conjugateJoint1Betta = Mathf.Acos( Mathf.Abs(pos.x)/pos_xy);
        float angle = 0.0f;

        if (pos.x < 0.0f)
            angle = Mathf.PI - conjugateJoint1Betta - conjugateJoint1Alpha;
        else if (pos.x < lengthJoint1)
            angle = conjugateJoint1Betta - conjugateJoint1Alpha;
        else
            angle = -(conjugateJoint1Alpha - conjugateJoint1Betta);
        angle *= -Mathf.Rad2Deg;

        if (angle < minAngles[0])
            angle = minAngles[0];
        else if (angle > maxAngles[0])
            angle = maxAngles[0];
        return angle;
    }
    private float GetJoint3Angle(Vector3 pos){
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y,2.0f) + Mathf.Pow(pos.x,2.0f));
        float h = Mathf.Sqrt(Mathf.Pow(pos_xy,2.0f) - Mathf.Pow(lengthJoint1,2.0f));
        float pos_joints23 = Mathf.Sqrt( Mathf.Pow(pos.z, 2.0f) + Mathf.Pow(h, 2.0f) );

        float angle = Mathf.Acos( 
            (Mathf.Pow(pos_joints23, 2) - Mathf.Pow(lengthJoint2,2) - Mathf.Pow(lengthJoint3,2))/
            (-2.0f * lengthJoint2 * lengthJoint3)) * Mathf.Rad2Deg;

        if (angle < minAngles[2])
            angle = minAngles[2];
        else if (angle > maxAngles[2])
            angle = maxAngles[2];
        return angle;
    }
    private float GetJoint2Angle(Vector3 pos){
        float angle = 0.0f;
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y,2.0f) + Mathf.Pow(pos.x,2.0f));
        float h = Mathf.Sqrt(Mathf.Pow(pos_xy,2.0f) - Mathf.Pow(lengthJoint1,2.0f));
        float pos_joints23 = Mathf.Sqrt( Mathf.Pow(pos.z, 2.0f) + Mathf.Pow(h, 2.0f) );

        float conjugateJoint2 = Mathf.Acos( 
            (Mathf.Pow(lengthJoint3, 2) - Mathf.Pow(lengthJoint2,2) - Mathf.Pow(pos_joints23,2))/
            (-2.0f * lengthJoint2 * pos_joints23));
        if (pos.z >= 0)
            conjugateJoint2 += Mathf.Acos( Mathf.Abs(pos.z)/pos_joints23);
        else
            conjugateJoint2 += Mathf.PI - Mathf.Acos( Mathf.Abs(pos.z)/pos_joints23);
        angle = -(Mathf.PI - conjugateJoint2) * Mathf.Rad2Deg;

        if (angle < minAngles[1])
            angle = minAngles[1];
        else if (angle > maxAngles[1])
            angle = maxAngles[1];
        return angle;
    }
    private List<float> ResolveInverseKinematicsFL(Vector3 pos){
        Debug.DrawRay(FLjoint1.position, transform.TransformVector(pos), Color.yellow);
        // Debug.Log(pos.x + ", " + pos.y + ", " + pos.z);
        return new List<float> { GetJoint1Angle_LeftLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsFR(Vector3 pos){
        Debug.DrawRay(FRjoint1.position, transform.TransformVector(pos), Color.yellow);
        // Debug.Log(pos.x + ", " + pos.y + ", " + pos.z);
        return new List<float> { GetJoint1Angle_RightLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsRL(Vector3 pos){
        Debug.DrawRay(RLjoint1.position, transform.TransformVector(pos), Color.yellow);
        // Debug.Log(pos.x + ", " + pos.y + ", " + pos.z);
        return new List<float> { GetJoint1Angle_LeftLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsRR(Vector3 pos){
        Debug.DrawRay(RRjoint1.position, transform.TransformVector(pos), Color.yellow);
        // Debug.Log(pos.x + ", " + pos.y + ", " + pos.z);
        return new List<float> { GetJoint1Angle_RightLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private void ApplyFLAngles(List<float> angles){
        FLjoint1.localRotation = Quaternion.Euler(angles[0], 0, 0);
        FLjoint2.localRotation = Quaternion.Euler(0, angles[1], 0);
        FLjoint3.localRotation = Quaternion.Euler(0, angles[2], 0);
    }
    private void ApplyFRAngles(List<float> angles){
        FRjoint1.localRotation = Quaternion.Euler(angles[0], 0, 0);
        FRjoint2.localRotation = Quaternion.Euler(0, angles[1], 0);
        FRjoint3.localRotation = Quaternion.Euler(0, angles[2], 0);
    }
    private void ApplyRLAngles(List<float> angles){
        RLjoint1.localRotation = Quaternion.Euler(angles[0], 0, 0);
        RLjoint2.localRotation = Quaternion.Euler(0, angles[1], 0);
        RLjoint3.localRotation = Quaternion.Euler(0, angles[2], 0);
    }
    private void ApplyRRAngles(List<float> angles){
        RRjoint1.localRotation = Quaternion.Euler(angles[0], 0, 0);
        RRjoint2.localRotation = Quaternion.Euler(0, angles[1], 0);
        RRjoint3.localRotation = Quaternion.Euler(0, angles[2], 0);
    }
    private Tuple<float, Vector3> GetLegMinima(Vector3 pos, Quaternion rot){
        List<Vector3> rayDirections = new List<Vector3>();
        for (float latitude = 0.0f; latitude < 2.0f * Mathf.PI; latitude += discretization){
            for (float longitude = 0.0f; longitude < Mathf.PI; longitude += discretization){
                Quaternion yRotation = Quaternion.Euler(Mathf.Rad2Deg * longitude, Mathf.Rad2Deg * latitude, 0.0f);
                rayDirections.Add(yRotation * Vector3.forward * (lengthJoint2 + lengthJoint3) );
            }
        }
        
        List<Tuple<Vector3, Vector3>> rayHitData = new List<Tuple<Vector3, Vector3>>(rayDirections.Count);
        foreach (Vector3 dir in rayDirections) {
            RaycastHit hit;
            // Debug.DrawRay(pos, dir, Color.gray);
            if (Physics.Raycast(pos, dir, out hit, dir.magnitude, ~0)){
                Vector3 delta = hit.point - pos;
                rayHitData.Add( new Tuple<Vector3, Vector3>(delta, hit.normal) );
            } else {
                rayHitData.Add( new Tuple<Vector3, Vector3>(Vector3.zero, Vector3.zero) );
            }
        }
        
        Tuple<float, Vector3> minDelta = new Tuple<float, Vector3>(float.MaxValue, Vector3.zero);
        foreach (var rayItem in rayHitData){
            if (rayItem.Item1 == Vector3.zero)
                continue;
            if (Mathf.Abs(Vector3.Angle(Vector3.up, rayItem.Item2)) > 45.0f)
                continue;
            if (rayItem.Item1.magnitude < minDelta.Item1)
                minDelta = new Tuple<float, Vector3>(rayItem.Item1.magnitude, rayItem.Item1);
        }
        return minDelta;
    }
    public Vector3 GetDisplacementByLeg(){
        Vector3 shift = Vector3.zero;
        Tuple<float, Vector3> FLdelta = GetLegMinima(FLjoint2.position, FLjoint2.rotation);
        Tuple<float, Vector3> FRdelta = GetLegMinima(FRjoint2.position, FRjoint2.rotation);
        Tuple<float, Vector3> RLdelta = GetLegMinima(RLjoint2.position, RLjoint2.rotation);
        Tuple<float, Vector3> RRdelta = GetLegMinima(RRjoint2.position, RRjoint2.rotation);
        
        Tuple<float, Vector3> maxDelta = RRdelta;
        if (FLdelta.Item1 > FRdelta.Item1 && FLdelta.Item1 > RLdelta.Item1 && FLdelta.Item1 > RRdelta.Item1)
            maxDelta = FLdelta;
        else if (FRdelta.Item1 > FLdelta.Item1 && FRdelta.Item1 > RLdelta.Item1 && FRdelta.Item1 > RRdelta.Item1)
            maxDelta = FRdelta;
        else if (RLdelta.Item1 > FLdelta.Item1 && RLdelta.Item1 > FRdelta.Item1 && RLdelta.Item1 > RRdelta.Item1)
            maxDelta = RLdelta;
        shift = Vector3.ProjectOnPlane( maxDelta.Item2, Vector3.up);
        return shift;
    }
    public Vector3 GetBodyHeightByLeg(float meanHeight = 0.26f){
        Vector3 shift = Vector3.zero;
        Tuple<float, Vector3> FLdelta = GetLegMinima(FLjoint2.position, FLjoint2.rotation);
        Tuple<float, Vector3> FRdelta = GetLegMinima(FRjoint2.position, FRjoint2.rotation);
        Tuple<float, Vector3> RLdelta = GetLegMinima(RLjoint2.position, RLjoint2.rotation);
        Tuple<float, Vector3> RRdelta = GetLegMinima(RRjoint2.position, RRjoint2.rotation);
        
        // Debug.DrawRay(FLjoint2.position, FLdelta.Item2, Color.blue);
        // Debug.DrawRay(FRjoint2.position, FRdelta.Item2, Color.blue);
        // Debug.DrawRay(RLjoint2.position, RLdelta.Item2, Color.blue);
        // Debug.DrawRay(RRjoint2.position, RRdelta.Item2, Color.blue);

        Vector3 FLeanPose = (meanHeight - FLdelta.Item2.magnitude) * Vector3.up;
        Vector3 FReanPose = (meanHeight - FRdelta.Item2.magnitude) * Vector3.up;
        Vector3 RLeanPose = (meanHeight - RLdelta.Item2.magnitude) * Vector3.up;
        Vector3 RReanPose = (meanHeight - RRdelta.Item2.magnitude) * Vector3.up;

        // Debug.DrawRay(FLjoint2.position + FLdelta.Item2, FLeanPose, Color.black);
        // Debug.DrawRay(FRjoint2.position + FRdelta.Item2, FReanPose, Color.black);
        // Debug.DrawRay(RLjoint2.position + RLdelta.Item2, RLeanPose, Color.black);
        // Debug.DrawRay(RRjoint2.position + RRdelta.Item2, RReanPose, Color.black);

        Tuple<float, Vector3> bodyDelta = GetLegMinima(body.position, RRjoint2.rotation);
        Debug.DrawRay(transform.position, bodyDelta.Item2, Color.yellow);
        shift = ((FLjoint2.position + FLdelta.Item2 + FLeanPose) + 
            (FRjoint2.position + FRdelta.Item2 + FReanPose) + 
            (RLjoint2.position + RLdelta.Item2 + RLeanPose) + 
            (RRjoint2.position + RRdelta.Item2 + RReanPose))/4.0f - transform.position;
            
        Debug.DrawRay(transform.position, shift, Color.green);
        return shift;
    }
    public void PlaceFoot(){
        Tuple<float, Vector3> FLdelta = GetLegMinima(FLjoint2.position, FLjoint2.rotation);
        Tuple<float, Vector3> FRdelta = GetLegMinima(FRjoint2.position, FRjoint2.rotation);
        Tuple<float, Vector3> RLdelta = GetLegMinima(RLjoint2.position, RLjoint2.rotation);
        Tuple<float, Vector3> RRdelta = GetLegMinima(RRjoint2.position, RRjoint2.rotation);

        Vector3 FLDirection = FLdelta.Item2 + FLjoint2.position - FLjoint1.position;
        Vector3 FRDirection = FRdelta.Item2 + FRjoint2.position - FRjoint1.position;
        Vector3 RLDirection = RLdelta.Item2 + RLjoint2.position - RLjoint1.position;
        Vector3 RRDirection = RRdelta.Item2 + RRjoint2.position - RRjoint1.position;
        
        Vector3 pos = transform.InverseTransformVector(FLDirection);
        ApplyFLAngles(ResolveInverseKinematicsFL(pos));
        pos = transform.InverseTransformVector(FRDirection);
        ApplyFRAngles(ResolveInverseKinematicsFR(pos));       
        pos = transform.InverseTransformVector(RLDirection);
        ApplyRLAngles(ResolveInverseKinematicsRL(pos));
        pos = transform.InverseTransformVector(RRDirection);
        ApplyRRAngles(ResolveInverseKinematicsRR(pos));
    }
    
    // private Vector3 GetBodyDisplacements(Quaternion rot){
        // Vector3 res = Vector3.zero;

        // List<Vector3> rayDirections = new List<Vector3>();
        // for (float angle = 0.0f; angle < 2.0f * Mathf.PI; angle += discretization){
        //     Quaternion yRotation = Quaternion.Euler(0.0f, Mathf.Rad2Deg * angle, 0.0f);
        //     rayDirections.Add(rot * yRotation * Vector3.forward * 0.3f);
        // }

        // List<Tuple<Vector3, Vector3>> rayHitData = new List<Tuple<Vector3, Vector3>>(rayDirections.Count);
        // foreach (Vector3 dir in rayDirections) {
        //     // Debug.DrawRay(pos, dir, Color.gray);
        //     RaycastHit hit;
        //     if (Physics.Raycast(pos, dir, out hit, dir.magnitude, ~0)){
        //         rayHitData.Add( new Tuple<Vector3, Vector3>(hit.point, hit.normal) );
        //     } else {
        //         rayHitData.Add( new Tuple<Vector3, Vector3>(Vector3.zero, Vector3.zero) );
        //     }
        // }

    //     List<Vector3> nonZeroDisplacements = new List<Vector3>();
    //     for (int id = 0; id < rayDirections.Count; ++id){
    //         Vector3 hitPoint = rayHitData[id].Item1;
    //         Vector3 hitNormal = rayHitData[id].Item2;
    //         Vector3 displacement = hitNormal * Mathf.Abs((pos - hitPoint).magnitude - rayDirections[id].magnitude);
    //         Debug.DrawRay(hitPoint, displacement, Color.white);
    //         if (displacement != Vector3.zero){
    //             nonZeroDisplacements.Add(displacement);
    //         }
    //     }

    //     if (nonZeroDisplacements.Count > 0){
    //         foreach (Vector3 v in nonZeroDisplacements)
    //             res += v;
    //         res /= nonZeroDisplacements.Count;
    //     }
        // return res;
    // }
}
