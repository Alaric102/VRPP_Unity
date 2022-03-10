using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Valve.VR;

public class VRUI : MonoBehaviour
{
    // [Header("RoundMenu")]
    public SteamVR_Action_Boolean touchTrackpad = null;
    public SteamVR_Action_Vector2 positionTrackpad = null;
    public SteamVR_Action_Boolean pressTrackpad = null;
    public SteamVR_Action_Boolean pressTrigger = null;
    private enum MenuStatus : int
    {
        NOTHING,
        MAIN,
        MAPPING,
        NAVIGATION
    }
    private GameObject mainMenuObj = null, mapMenuObj = null, navMenuObj = null;
    void Awake()
    {
        touchTrackpad.onChange += TouchTrackpad;
        positionTrackpad.onAxis += TrackpadPosition;
        pressTrackpad.onStateUp += TrackpadPressRelease;
        pressTrigger.onStateUp += TriggerPress;

        // find objects of VRUI
        mainMenuObj = transform.GetChild(0).gameObject;
        mapMenuObj = transform.GetChild(1).gameObject;
        navMenuObj = transform.GetChild(2).gameObject;

    }
    void Start(){
        mainMenuObj.SetActive(true);
        mapMenuObj.SetActive(false);
        navMenuObj.SetActive(false);
    }
    void Update(){

    }
    private void TouchTrackpad(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource, bool newState){
        if (mainMenuObj.activeSelf){
            mainMenuObj.GetComponent<MainMenu>().processTouchTrackpad(newState);
        } else if (mapMenuObj.activeSelf){
            mapMenuObj.GetComponent<MappingMenu>().processTouchTrackpad(newState);
        } else if (navMenuObj.activeSelf){
            navMenuObj.GetComponent<NavigationMenu>().processTouchTrackpad(newState);
        }
    }
    private void TrackpadPosition(SteamVR_Action_Vector2 fromAction, SteamVR_Input_Sources fromSource, Vector2 axis, Vector2 delta){
        if (mainMenuObj.activeSelf){
            mainMenuObj.GetComponent<MainMenu>().processTrackpadPosition(axis);
        } else if (mapMenuObj.activeSelf){
            mapMenuObj.GetComponent<MappingMenu>().processTrackpadPosition(axis);
        } else if (navMenuObj.activeSelf){
            navMenuObj.GetComponent<NavigationMenu>().processTrackpadPosition(axis);
        }
    }
    private void TrackpadPressRelease(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource){
        if (mainMenuObj.activeSelf){
            mainMenuObj.GetComponent<MainMenu>().processTrackpadPressRelease();
        } else if (mapMenuObj.activeSelf){
            mapMenuObj.GetComponent<MappingMenu>().processTrackpadPressRelease();
        } else if (navMenuObj.activeSelf){
            navMenuObj.GetComponent<NavigationMenu>().processTrackpadPressRelease();
        }
    }
    private void TriggerPress(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource){
        if (navMenuObj.activeSelf){
            navMenuObj.GetComponent<NavigationMenu>().processTriggerPress();
        }
    }
    public void SetActiveMainMenu(){
        mapMenuObj.SetActive(false);
        navMenuObj.SetActive(false);

        mainMenuObj.SetActive(true);
    }
    public void SetActiveMappingMenu(){
        mainMenuObj.SetActive(false);
        navMenuObj.SetActive(false);

        mapMenuObj.SetActive(true);
    }
    public void SetActiveNavigationMenu(){
        mainMenuObj.SetActive(false);
        mapMenuObj.SetActive(false);

        navMenuObj.SetActive(true);
    }
}
