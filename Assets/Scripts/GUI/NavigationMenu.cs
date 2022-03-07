using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Valve.VR;


public class NavigationMenu : MonoBehaviour
{
    private bool menuIsActive = true;
    private Vector2 trackpadVector = Vector2.zero;
    private int menuStatus = ((int)MenuStatus.NOTHING);
    private bool isSettingStartPosition = false, isSettingGoalPosition = false, isSettingRotation = false;
    private GameObject setStartStatePanel = null, setGoalStatePanel = null, startPlanningPanel = null, resetPanel = null;
    private VRUI vrui = null;
    private HelpBar helpBar = null;
    LineRenderer lineRenderer = null;
    private Vector3 setStatePosition;
    private Quaternion setStateRotation;
    private enum MenuStatus : int {
        NOTHING,
        SET_START, 
        SET_GOAL,
        START_PLAN,
        BACK
    } 

    [Header("External modules")]
    public GameObject ActiveController;
    public Navigation navigation = null;
    void Awake()
    {
        // find objects of VRUI
        setStartStatePanel = transform.GetChild(0).gameObject;
        setGoalStatePanel = transform.GetChild(1).gameObject;
        startPlanningPanel = transform.GetChild(2).gameObject;
        resetPanel = transform.GetChild(3).gameObject;
        vrui = transform.parent.GetComponent<VRUI>();
        helpBar = transform.parent.GetChild(transform.parent.childCount - 1).GetComponent<HelpBar>();
        lineRenderer = gameObject.GetComponent<LineRenderer>();
    }

    void Start(){
        lineRenderer.positionCount = 0;

        setStatePosition = Vector3.zero;
        setStateRotation = Quaternion.identity;
    }
    void Update()
    {
        lineRenderer.positionCount = 0;

        if (menuIsActive && !(isSettingStartPosition || isSettingGoalPosition)){
            setStartStatePanel.SetActive(true);
            setGoalStatePanel.SetActive(true);
            startPlanningPanel.SetActive(true);
            resetPanel.SetActive(true);

            getMenuStatus(trackpadVector);
            HighlightSelected();

            helpBar.SetHelpText("Press trackpad to select option.");
        } else {
            setStartStatePanel.SetActive(false);
            setGoalStatePanel.SetActive(false);
            startPlanningPanel.SetActive(false);
            resetPanel.SetActive(false);
            
            helpBar.SetHelpText("Use trackpad to see Navigation Menu");
        }

        if(isSettingStartPosition || isSettingGoalPosition){
            string prefixStr;
            if (isSettingRotation){
                setStateRotation = GetRotation();
                prefixStr = "Press trigger to finish.";
            } else {
                setStatePosition = GetPosition();
                prefixStr = "Press trigger to set rotation.";
            }
            string data = GetHelpText(setStatePosition, setStateRotation);
            helpBar.SetHelpText(prefixStr + "\n" + data, 20);

            if (isSettingStartPosition){
                navigation.setStartState(setStatePosition, setStateRotation);
            } else if (isSettingGoalPosition){
                navigation.setGoalState(setStatePosition, setStateRotation);
            }
        }
    }
    public int getMenuStatus(Vector2 axis){
        float angle = Vector2.Angle(Vector2.right, axis) * Mathf.Sign(axis.y);
        float length = Vector2.SqrMagnitude(axis);
        menuStatus = ((int)MenuStatus.NOTHING);

        if (length > 0.5f){
            if (angle > 120.0f){
                menuStatus = ((int)MenuStatus.SET_START);
            } else if (angle > -60.0f){
                menuStatus = ((int)MenuStatus.NOTHING);
            } else if (angle > -120.0f){
                menuStatus = ((int)MenuStatus.START_PLAN);
            } else {
                menuStatus = ((int)MenuStatus.SET_GOAL);
            }
        } else if (length < 0.2f){
            menuStatus = ((int)MenuStatus.BACK);
        }
        return menuStatus;
    }
    private void HighlightSelected(){
        setStartStatePanel.GetComponent<Image>().color = Color.white;
        setGoalStatePanel.GetComponent<Image>().color = Color.white;    
        startPlanningPanel.GetComponent<Image>().color = Color.white;
        resetPanel.GetComponent<Image>().color = Color.white;

        switch (menuStatus) {
            case ((int)MenuStatus.SET_START):
                setStartStatePanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.SET_GOAL):
                setGoalStatePanel.GetComponent<Image>().color = Color.green;  
                break;
            case ((int)MenuStatus.START_PLAN):
                startPlanningPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.BACK):
                resetPanel.GetComponent<Image>().color = Color.green;
                break;
            default:
                break;
        }
    }

    private Vector3 GetPosition(){        
        Debug.DrawRay(ActiveController.transform.position, 
            ActiveController.transform.TransformDirection(Vector3.forward) * 100,
            Color.yellow);
            
        lineRenderer = gameObject.GetComponent<LineRenderer>();
        RaycastHit hit;
        if (Physics.Raycast(ActiveController.transform.position, 
            ActiveController.transform.TransformDirection(Vector3.forward), 
            out hit, Mathf.Infinity, 7)){
                lineRenderer.positionCount = 2;
                lineRenderer.SetPosition(0, ActiveController.transform.position);
                lineRenderer.SetPosition(1, hit.point);
                return hit.point;
        } else {
            lineRenderer.positionCount = 0;
        }
        return Vector3.zero;
    }
    private Quaternion GetRotation(){
        return ActiveController.transform.rotation;
    }

    private string GetHelpText(Vector3 pos, Quaternion rot){
        Vector3 rotEuler = rot.eulerAngles;
        string res = 
            "Position x: " + Mathf.Round(pos.x).ToString() +
            ", y: " + Mathf.Round(pos.y).ToString() +
            ", z: " + Mathf.Round(pos.z).ToString() +
            "Angle: x: " + Mathf.Round(rotEuler.x).ToString() +
            ", y: " + Mathf.Round(rotEuler.y).ToString() +
            ", z: " + Mathf.Round(rotEuler.z).ToString();
        return res;
    }
    public void processTouchTrackpad(bool newState){
        menuIsActive = newState;
    }
    public void processTrackpadPosition(Vector2 axis){
        trackpadVector = axis;
    }
    public void processTrackpadPressRelease(){
        if (!menuIsActive){
            return;
        }
        switch (menuStatus) {
            case ((int)MenuStatus.SET_START):
                isSettingStartPosition = true;
                break;
            case ((int)MenuStatus.SET_GOAL):
                isSettingGoalPosition = true;
                break;
            case ((int)MenuStatus.START_PLAN):
                
                break;
            case ((int)MenuStatus.BACK):
                vrui.SetActiveMainMenu();
                break;
            default:
                break;
        }
    }
    public void processTriggerPress(){
        Debug.Log("trigger!");
        if (isSettingStartPosition || isSettingGoalPosition){
            if (isSettingRotation){
                isSettingStartPosition = false;
                isSettingGoalPosition = false;
                isSettingRotation = false;
            } else {
                isSettingRotation = true;
            }
        }


    }
}
