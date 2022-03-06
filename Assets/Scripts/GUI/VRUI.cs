using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Valve.VR;

public class VRUI : MonoBehaviour
{
    [Header("RoundMenu")]
    public SteamVR_Action_Boolean touch = null;
    public SteamVR_Action_Boolean press= null;
    public SteamVR_Action_Vector2 touchPosition = null;
    
    private enum MenuStatus : int
    {
        NOTHING,
        MAIN,
        MAPPING,
        NAVIGATION
    }
    private GameObject mainMenuObj = null, mapMenuObj = null, navMenuObj = null;
    private Vector2 trackpadVector = Vector2.zero;
    private bool activateMenu = false;
    private int menuStatus = ((int)MenuStatus.MAIN);
    void Awake()
    {
        // Attach actions
        touch.onChange += Touch;
        press.onStateUp += PressRelease;
        touchPosition.onAxis += Position;

        // Get context menu options
        mainMenuObj = transform.GetChild(0).gameObject;
        mapMenuObj = transform.GetChild(1).gameObject;
        navMenuObj = transform.GetChild(2).gameObject;

        mainMenuObj.SetActive(true);
        mapMenuObj.SetActive(false);
        navMenuObj.SetActive(false);
    }
    void Update(){
        mainMenuObj.GetComponent<Canvas>().enabled = false;
        mainMenuObj.SetActive(false);
        mapMenuObj.GetComponent<Canvas>().enabled = false;
        mapMenuObj.SetActive(false);
        navMenuObj.GetComponent<Canvas>().enabled = false;
        navMenuObj.SetActive(false);

        if (activateMenu){
            switch (menuStatus) {
                case ((int)MenuStatus.MAIN):
                    mainMenuObj.SetActive(true);
                    mainMenuObj.GetComponent<Canvas>().enabled = true;
                    mainMenuObj.GetComponent<MainMenu>().Proccess(trackpadVector);
                    break;
                case ((int)MenuStatus.MAPPING):
                    mapMenuObj.SetActive(true);
                    mapMenuObj.GetComponent<Canvas>().enabled = true;
                    mapMenuObj.GetComponent<MappingMenu>().Proccess(trackpadVector);
                    break;
                case ((int)MenuStatus.NAVIGATION):
                    navMenuObj.SetActive(true);
                    navMenuObj.GetComponent<Canvas>().enabled = true;
                    navMenuObj.GetComponent<NavigationMenu>().Proccess(trackpadVector);
                    break;
                default:
                Debug.Log("Menu is not proccesable.");
                    break;
            }
        }
    }
    void OnDestroy()
    {
        // MenuCanvas.gameObject.SetActive(false);
        touch.onChange -= Touch;
        press.onStateUp -= PressRelease;
        touchPosition.onAxis -= Position;
    }

    private void Position(SteamVR_Action_Vector2 fromAction, SteamVR_Input_Sources fromSource, Vector2 axis, Vector2 delta){
        trackpadVector = axis;
    }
    private void Touch(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource, bool newState){
        activateMenu = newState;
    }

    private void PressRelease(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource){
        switch (menuStatus) {
            case ((int)MenuStatus.MAIN):
                mainMenuObj.GetComponent<MainMenu>().ProccessTrigger();
                break;
            case ((int)MenuStatus.MAPPING):
                mapMenuObj.GetComponent<MappingMenu>().ProccessTrigger();
                break;
            case ((int)MenuStatus.NAVIGATION):
                navMenuObj.GetComponent<NavigationMenu>().ProccessTrigger();
                break;
            default:
                break;
        }
    }

    public void SetMenuStatus(int newStatus){
        menuStatus = newStatus;
    }
}
