using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SliderComponent : MonoBehaviour
{
    public Slider slider;
    public Text text;

    public NewIK ArmControllerScript;
    public enum Action
    {
        SetExtremityOffset,
        SetExtremityRotation,
        SetGripperOpenPercent,
        SetMotorSpeed
    }
    public Action selectedAction;
    void Start()
    {

        switch (selectedAction)
        {
            case Action.SetExtremityOffset:
                slider.value = ArmControllerScript.extremityOffset;
                break;

            case Action.SetExtremityRotation:
                slider.value = ArmControllerScript.WristRotatePercent;
                break;

            case Action.SetGripperOpenPercent:
                slider.value = ArmControllerScript.gripperPercent;
                break;

            case Action.SetMotorSpeed:
                slider.value = ArmControllerScript.MotorRotationSpeed;
                break;
        }
        text.text = slider.value.ToString("0.00");


        slider.onValueChanged.AddListener((v) => {
            text.text = v.ToString("0.00");
            switch (selectedAction)
            {
                case Action.SetExtremityOffset:
                ArmControllerScript.extremityOffset = v;
                break;

                case Action.SetExtremityRotation:
                ArmControllerScript.WristRotatePercent = v;
                break;

                case Action.SetGripperOpenPercent:
                ArmControllerScript.gripperPercent = v;
                break;

                case Action.SetMotorSpeed:
                ArmControllerScript.MotorRotationSpeed = v;
                break;
            }
        });
    }

    void Update(){
        
    }
}
