using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class InvertAxisToggle : MonoBehaviour
{
   public TargetController targetScript;
   public Toggle toggle;

    void Start()
    {
        toggle.isOn = targetScript.invertAxis;
        toggle.onValueChanged.AddListener(OnToggleValueChanged);
    }

    void OnToggleValueChanged(bool isOn)
    {
        targetScript.invertAxis = isOn;
    }
}
