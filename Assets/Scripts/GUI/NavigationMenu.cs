using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Valve.VR;


public class NavigationMenu : MonoBehaviour
{
    private GameObject setStartPanel = null, setGoalPanel = null, startPlanningPanel = null, resetPanel = null;
    private bool isSettingStart = false, isSettingGoal = false;
    private LineRenderer lineSetting = null;

    [Header("Mapper objects")]
    public Mapper mapper;
    private enum MainMenuStatus : int
    {
        NOTHING,
        SET_START, 
        SET_GOAL,
        START_PLAN,
        BACK
    }
    private int menuMod = ((int)MainMenuStatus.NOTHING);
    

    [Header("Trigger")]
    public GameObject ControllerLeft;
    public SteamVR_Action_Boolean trigger = null;
    void Awake()
    {
        setStartPanel = transform.GetChild(0).gameObject;
        setGoalPanel = transform.GetChild(1).gameObject;
        startPlanningPanel = transform.GetChild(2).gameObject;
        resetPanel = transform.GetChild(3).gameObject;
        
        trigger.onState += TriggerPress;
        lineSetting = gameObject.GetComponent<LineRenderer>();
        lineSetting.positionCount = 0;
    }

    // Update is called once per frame
    void Update()
    {
        lineSetting.positionCount = 0;
        setStartPanel.SetActive(false);
        setGoalPanel.SetActive(false);
        startPlanningPanel.SetActive(false);
        resetPanel.SetActive(false);
                
        if (isSettingStart || isSettingGoal){
            DrawLine();
        } else {
            setStartPanel.SetActive(true);
            setGoalPanel.SetActive(true);
            startPlanningPanel.SetActive(true);
            resetPanel.SetActive(true);

            ProcessPanels();
        }
    }
    public void Proccess(Vector2 axis){
        float angle = Vector2.Angle(Vector2.right, axis) * Mathf.Sign(axis.y);
        float length = Vector2.SqrMagnitude(axis);
        
        // Set default parameters
        menuMod = ((int)MainMenuStatus.NOTHING);
        
        // Highlight menu selection
        if (length > 0.5f){
            if (angle > 120.0f){
                menuMod = ((int)MainMenuStatus.SET_START);
            } else if (angle > -60.0f){
                menuMod = ((int)MainMenuStatus.NOTHING);
            } else if (angle > -120.0f){
                menuMod = ((int)MainMenuStatus.START_PLAN);
            } else {
                menuMod = ((int)MainMenuStatus.SET_GOAL);
            }
        } else if (length < 0.2f){
            menuMod = ((int)MainMenuStatus.BACK);
        }
        Update();
    }
    public void ProccessTrigger(){
        switch (menuMod) {
            case ((int)MainMenuStatus.NOTHING):
                Debug.Log("Nothing");
                break;
            case ((int)MainMenuStatus.SET_START):
                Debug.Log("Set start");
                isSettingStart = true;
                isSettingGoal = false;
                break;
            case ((int)MainMenuStatus.SET_GOAL):
                Debug.Log("Set goal");
                isSettingStart = false;
                isSettingGoal = true;
                break;
            case ((int)MainMenuStatus.START_PLAN):
                Debug.Log("Start plan");
                mapper.startPlanning();
                break;
            case ((int)MainMenuStatus.BACK):
                Debug.Log("Go to main (1)");
                transform.parent.GetComponent<VRUI>().SetMenuStatus(1);
                break;
            default:
                Debug.Log("Main menu can't recognize trigger.");
                break;
        }
    }

    private void ProcessPanels(){
        resetPanel.GetComponent<Image>().color = Color.white;
        setStartPanel.GetComponent<Image>().color = Color.white;
        setGoalPanel.GetComponent<Image>().color = Color.white;    
        startPlanningPanel.GetComponent<Image>().color = Color.white;
        switch (menuMod)
        {
            case ((int)MainMenuStatus.SET_START):
                setStartPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MainMenuStatus.SET_GOAL):
                setGoalPanel.GetComponent<Image>().color = Color.green;  
                break;
            case ((int)MainMenuStatus.START_PLAN):
                startPlanningPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MainMenuStatus.BACK):
                resetPanel.GetComponent<Image>().color = Color.green;
                break;
            default:
                break;
        }
    }
    public void DrawLine(){        
        Debug.DrawRay(ControllerLeft.transform.position, 
            ControllerLeft.transform.TransformDirection(Vector3.forward) * 100,
            Color.yellow);
            
        RaycastHit hit;
        if (Physics.Raycast(ControllerLeft.transform.position, 
            ControllerLeft.transform.TransformDirection(Vector3.forward), 
            out hit, Mathf.Infinity, 7)){
                lineSetting.positionCount = 2;
                lineSetting.SetPosition(0, ControllerLeft.transform.position);
                lineSetting.SetPosition(1, hit.point);
                if (isSettingGoal){
                    mapper.setGoalPoint(hit.point);
                } else if (isSettingStart){
                    mapper.setStartPoint(hit.point);
                }
        } else {
            lineSetting.positionCount = 0;
        }

    }
    private void TriggerPress(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource){
        isSettingStart = false;
        isSettingGoal = false;
        
        mapper.sendStartPoint2D();
    }
}
