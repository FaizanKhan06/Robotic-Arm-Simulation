using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using UnityEngine;
using UnityEngine.UI;

public class OutputDisplay : MonoBehaviour
{
    public Text baseOutput,rootOutput,midOutput,tipOutput,gripperOutput;
    public NewIK ikScript;
    void Update()
    {
        UpdateText("Base", ikScript.initialize.Base, baseOutput);
        UpdateText("Root", ikScript.initialize.root, rootOutput);
        UpdateText("Mid", ikScript.initialize.mid, midOutput);
        UpdateText("Tip", ikScript.initialize.tip, tipOutput);
        UpdateText("Gripper", ikScript.initialize.ServoExtension, gripperOutput);
    }

    void UpdateText(String name, Transform transformObj, Text text){
        UnityEngine.Vector3 values = transformObj.localEulerAngles;
        text.text = name+".rotation = ("+ConvertVal(values.x)+","+ConvertVal(values.y)+","+ConvertVal(values.z)+")";
    }

    //Convert no to string and max 2 decimal places
    String ConvertVal(float value){
        return value.ToString("0.00");
    }
}
