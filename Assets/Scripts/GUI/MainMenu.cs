using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Valve.VR;
public class MainMenu : MonoBehaviour
{
    [Header("Actions")]
    public SteamVR_Action_Boolean touchTrackpad = null;
    private bool menuIsActive = false;
    public SteamVR_Action_Vector2 positionTrackpad = null;
    private Vector2 trackpadVector = Vector2.zero;
    public SteamVR_Action_Boolean pressTrackpad = null;
    private int menuStatus = ((int)MenuStatus.NOTHING);
    private GameObject mapPanel = null, navPanel = null;
    private VRUI vrui = null;
    private HelpBar helpBar = null;
    private enum MenuStatus : int
    {
        NOTHING,
        MAPPING, 
        NAVIGATION
    }
    void Awake()
    {
        // attach actions to functions
        touchTrackpad.onChange += TouchTrackpad;
        positionTrackpad.onAxis += TrackpadPosition;
        pressTrackpad.onStateUp += TrackpadPressRelease;

        // find objects of VRUI
        mapPanel = transform.GetChild(0).gameObject;
        navPanel = transform.GetChild(1).gameObject;
        vrui = transform.parent.GetComponent<VRUI>();
        helpBar = transform.parent.GetChild(transform.parent.childCount - 1).GetComponent<HelpBar>();
    }
    void Update()
    {
        if (menuIsActive){
            mapPanel.SetActive(true);
            navPanel.SetActive(true);

            getMenuStatus(trackpadVector);
            HighlightSelected();
            
            helpBar.SetHelpText("Press trackpad to select option.");
        } else {
            mapPanel.SetActive(false);
            navPanel.SetActive(false);

            helpBar.SetHelpText("Use trackpad to see Main Menu");
        }
    }
    public int getMenuStatus(Vector2 axis){
        float angle = Vector2.Angle(Vector2.right, axis) * Mathf.Sign(axis.y);
        float length = Vector2.SqrMagnitude(axis);
        menuStatus = ((int)MenuStatus.NOTHING);

        if (length > 0.5f){
            if (angle > 0.0f){
                menuStatus = ((int)MenuStatus.MAPPING);
            } else {
                menuStatus = ((int)MenuStatus.NAVIGATION);
            }
        } else if (length < 0.2f){
            menuStatus = ((int)MenuStatus.NOTHING);
        }
        return menuStatus;
    }
    private void HighlightSelected(){
        mapPanel.GetComponent<Image>().color = Color.white;
        navPanel.GetComponent<Image>().color = Color.white;    

        switch (menuStatus) {
            case ((int)MenuStatus.MAPPING):
                mapPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.NAVIGATION):
                navPanel.GetComponent<Image>().color = Color.green;
                break;
            default:
                break;
        }
    }
    private void TouchTrackpad(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource, bool newState){
        menuIsActive = newState;
    }
    private void TrackpadPosition(SteamVR_Action_Vector2 fromAction, SteamVR_Input_Sources fromSource, Vector2 axis, Vector2 delta){
        trackpadVector = axis;
    }
    private void TrackpadPressRelease(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource){
        if (!menuIsActive){
            return;
        }
        switch (menuStatus) {
            case ((int)MenuStatus.MAPPING):
                vrui.SetActiveMappingMenu();
                break;
            case ((int)MenuStatus.NAVIGATION):
                vrui.SetActiveNavigationMenu();
                break;
            default:
                break;
        }
    }
}
