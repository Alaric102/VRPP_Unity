using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class HelpBar : MonoBehaviour
{
    private Text helpBarText = null;
    // Start is called before the first frame update
    void Awake()
    {
        helpBarText = transform.GetChild(0).GetChild(0).GetComponent<Text>();
    }
    void Update()
    {
        
    }
    public void SetHelpText(string s, int fontSize = 28){
        helpBarText.text = "HELP: " + s;
        helpBarText.fontSize = fontSize;
    }
}
