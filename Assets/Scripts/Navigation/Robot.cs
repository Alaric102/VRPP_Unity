using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour
{
    // Common parameters
    private Transform body;
    private float lengthJoint1, lengthJoint2, lengthJoint3;
    public List<float> maxAngles = new List<float>{90.0f, 180.0f, 180.0f};
    public List<float> minAngles = new List<float>{-90.0f, -180.0f, -180.0f};
    private float minDistance = 0.1053526f, maxDistance = 0.3111828f;
    // FL leg variables
    private Transform FLjoint1, FLjoint2, FLjoint3, FLEndEffector;
    public List<float> FLAngles = new List<float>{0.0f, 0.0f, 0.0f};
    public Vector3 relativeFLPosition = Vector3.zero;
    // FR leg variables
    private Transform FRjoint1, FRjoint2, FRjoint3, FREndEffector;
    public List<float> FRAngles = new List<float>{0.0f, 0.0f, 0.0f};
    public Vector3 relativeFRPosition = Vector3.zero;
    // RL leg variables
    private Transform RLjoint1, RLjoint2, RLjoint3, RLEndEffector;
    public List<float> RLAngles = new List<float>{0.0f, 0.0f, 0.0f};
    public Vector3 relativeRLPosition = Vector3.zero;
    // RR leg variables
    private Transform RRjoint1, RRjoint2, RRjoint3, RREndEffector;
    public List<float> RRAngles = new List<float>{0.0f, 0.0f, 0.0f};
    private Vector3 lastFLPose = Vector3.zero; 
    private Vector3 lastFRPose = Vector3.zero; 
    private Vector3 lastRLPose = Vector3.zero; 
    private Vector3 lastRRPose = Vector3.zero; 
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
        Debug.Log(pos.x + ", " + pos.y + ", " + pos.z);
        return new List<float> { GetJoint1Angle_LeftLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsFR(Vector3 pos){
        Debug.DrawRay(FRjoint1.position, transform.TransformVector(pos), Color.yellow);
        Debug.Log(pos.x + ", " + pos.y + ", " + pos.z);
        return new List<float> { GetJoint1Angle_RightLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsRL(Vector3 pos){
        Debug.DrawRay(RLjoint1.position, transform.TransformVector(pos), Color.yellow);
        Debug.Log(pos.x + ", " + pos.y + ", " + pos.z);
        return new List<float> { GetJoint1Angle_LeftLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsRR(Vector3 pos){
        Debug.DrawRay(RRjoint1.position, transform.TransformVector(pos), Color.yellow);
        Debug.Log(pos.x + ", " + pos.y + ", " + pos.z);
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
    private Vector3 FindStableFoot(Vector3 stepDirection, Transform joint2){
        RaycastHit hitHeap;
        if (Physics.Raycast(joint2.position, -Vector3.up, out hitHeap, Mathf.Infinity, ~0)){
            Vector3 direction = hitHeap.point + stepDirection - joint2.position;
            if (Physics.Raycast(joint2.position, direction, out hitHeap, Mathf.Infinity, ~0)){
            }
        }
        for(int i = 0; i < 100 && Vector3.Angle(Vector3.up, hitHeap.normal) > 45.0f;){
            stepDirection -= stepDirection*0.1f;
            if (Physics.Raycast(joint2.position, -Vector3.up, out hitHeap, Mathf.Infinity, ~0)){
                Vector3 direction = hitHeap.point + stepDirection - joint2.position;
                if (Physics.Raycast(joint2.position, direction, out hitHeap, Mathf.Infinity, ~0)){}
            }
        }
        Debug.DrawLine(joint2.position, hitHeap.point, Color.gray);
        Debug.DrawRay(hitHeap.point, hitHeap.normal*0.1f, Color.gray);
        return hitHeap.point;
    }
    public void GetStableFoot(Vector3 stepDirection, int stepNumber){
        Vector3 footPoseFL = FindStableFoot(stepDirection - ((stepNumber + 0) % 4) / 2.0f * stepDirection, FLjoint2);
        Vector3 footPoseFR = FindStableFoot(stepDirection - ((stepNumber + 2) % 4) / 2.0f * stepDirection, FRjoint2);
        Vector3 footPoseRL = FindStableFoot(stepDirection - ((stepNumber + 3) % 4) / 2.0f * stepDirection, RLjoint2);
        Vector3 footPoseRR = FindStableFoot(stepDirection - ((stepNumber + 1) % 4) / 2.0f * stepDirection, RRjoint2);

        Vector3 pos = transform.InverseTransformVector(footPoseFL - FLjoint1.position);
        ApplyFLAngles(ResolveInverseKinematicsFL(pos));
        
        pos = transform.InverseTransformVector(footPoseFR - FRjoint1.position);
        ApplyFRAngles(ResolveInverseKinematicsFR(pos));
        
        pos = transform.InverseTransformVector(footPoseRL - RLjoint1.position);
        ApplyRLAngles(ResolveInverseKinematicsRL(pos));
        
        pos = transform.InverseTransformVector(footPoseRR - RRjoint1.position);
        ApplyRRAngles(ResolveInverseKinematicsRR(pos));

    }
    public Quaternion GetSurfaceNorm(){
        RaycastHit hitFLHeap, hitFRHeap, hitRLHeap, hitRRHeap;
        if (Physics.Raycast(FLjoint2.position, -transform.up, out hitFLHeap, Mathf.Infinity, ~0)){
            Debug.DrawRay(FLjoint2.position, hitFLHeap.normal*0.15f, Color.yellow);
            Debug.DrawRay(FLjoint2.position, transform.up*0.15f, Color.green);
        }
        if (Physics.Raycast(FRjoint2.position, -transform.up, out hitFRHeap, Mathf.Infinity, ~0)){
            Debug.DrawRay(FRjoint2.position, hitFRHeap.normal*0.15f, Color.yellow);
            Debug.DrawRay(FRjoint2.position, transform.up*0.15f, Color.green);
        }
        if (Physics.Raycast(RLjoint2.position, -transform.up, out hitRLHeap, Mathf.Infinity, ~0)){
            Debug.DrawRay(RLjoint2.position, hitRLHeap.normal*0.15f, Color.yellow);
            Debug.DrawRay(RLjoint2.position, transform.up*0.15f, Color.green);
        }
        if (Physics.Raycast(RRjoint2.position, -transform.up, out hitRRHeap, Mathf.Infinity, ~0)){
            Debug.DrawRay(RRjoint2.position, hitRRHeap.normal*0.15f, Color.yellow);
            Debug.DrawRay(RRjoint2.position, transform.up*0.15f, Color.green);
        }
        float angle = Vector3.SignedAngle(transform.up, hitFLHeap.normal, transform.right);
        angle += Vector3.SignedAngle(transform.up, hitFRHeap.normal, transform.right);
        angle += Vector3.SignedAngle(transform.up, hitRLHeap.normal, transform.right);
        angle += Vector3.SignedAngle(transform.up, hitRRHeap.normal, transform.right);
        angle /= 4.0f;
        return Quaternion.Euler(angle, 0.0f, 0.0f);
    }
    public Vector3 GetSurfaceShift(float meanPose){
        RaycastHit hitFLHeap, hitFRHeap, hitRLHeap, hitRRHeap, hitBody;
        if (Physics.Raycast(body.position, -transform.up, out hitBody, Mathf.Infinity, ~0)){
            Debug.DrawRay(body.position, hitBody.normal*0.15f, Color.yellow);
            Debug.DrawRay(body.position, transform.up*0.15f, Color.green);
        }
        if (Physics.Raycast(FLjoint2.position, -transform.up, out hitFLHeap, Mathf.Infinity, ~0)){
            Debug.DrawRay(FLjoint2.position, hitFLHeap.normal*0.15f, Color.yellow);
            Debug.DrawRay(FLjoint2.position, transform.up*0.15f, Color.green);
        }
        if (Physics.Raycast(FRjoint2.position, -transform.up, out hitFRHeap, Mathf.Infinity, ~0)){
            Debug.DrawRay(FRjoint2.position, hitFRHeap.normal*0.15f, Color.yellow);
            Debug.DrawRay(FRjoint2.position, transform.up*0.15f, Color.green);
        }
        if (Physics.Raycast(RLjoint2.position, -transform.up, out hitRLHeap, Mathf.Infinity, ~0)){
            Debug.DrawRay(RLjoint2.position, hitRLHeap.normal*0.15f, Color.yellow);
            Debug.DrawRay(RLjoint2.position, transform.up*0.15f, Color.green);
        }
        if (Physics.Raycast(RRjoint2.position, -transform.up, out hitRRHeap, Mathf.Infinity, ~0)){
            Debug.DrawRay(RRjoint2.position, hitRRHeap.normal*0.15f, Color.yellow);
            Debug.DrawRay(RRjoint2.position, transform.up*0.15f, Color.green);
        }
        Vector3 res = Vector3.zero;
        res += Vector3.down * ((hitFLHeap.point - FLjoint2.position).magnitude - meanPose);
        res += Vector3.down * ((hitFRHeap.point - FRjoint2.position).magnitude - meanPose);
        res += Vector3.down * ((hitRLHeap.point - RLjoint2.position).magnitude - meanPose);
        res += Vector3.down * ((hitRRHeap.point - RRjoint2.position).magnitude - meanPose);
        return res / 4.0f;
    }
}
