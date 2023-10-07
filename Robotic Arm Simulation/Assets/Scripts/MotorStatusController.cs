using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MotorStatusController : MonoBehaviour
{
    public NewIK ikScript;
    public Dropdown dropdown;
    // Start is called before the first frame update
    void Start()
    {
        dropdown.onValueChanged.AddListener(OnDropdownValueChanged);
    }

    void OnDropdownValueChanged(int index)
    {
        switch (index)
        {
            case 0:
                ikScript.isRotateSmoothly = true;
                break;
            case 1:
                ikScript.isRotateSmoothly = false;
                break;
        }
    }
}
