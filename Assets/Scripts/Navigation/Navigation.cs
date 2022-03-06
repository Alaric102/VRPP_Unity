using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Navigation : MonoBehaviour
{
    public Transform robotTransform = null;
    private Transform startState;
    private Transform goalState;
    // Start is called before the first frame update
    void Awake()
    {
        startState.position = Vector3.zero;
        startState.rotation = Quaternion.identity;
        goalState.position = Vector3.zero;
        goalState.rotation = Quaternion.identity;

        if (robotTransform == null){
            Debug.Log("Navigation: empty robotTransform");
        } else {
            robotTransform.transform.position = startState.position;
            robotTransform.transform.rotation = startState.rotation;
        }
    }
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void setStartState(Transform tr){ startState = tr; }
    public void setGoalState(Transform tr){ goalState = tr; }
}
