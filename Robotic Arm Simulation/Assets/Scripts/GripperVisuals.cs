using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GripperVisuals : MonoBehaviour
{
    
    [Header("Just For Visuals")]
    public Transform leftGripper;
    public Transform rightGripper;
    public Transform RubberBand;
    public NewIK ikScript;
    
    public float maxGripperOpenValue = 0.075f;

    private float leftGripperPosZ,rightGripperPosZ,RubberBandScaleZ;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {

        leftGripperPosZ = MapValues(ikScript.gripperPercent, 0f, -1 * maxGripperOpenValue);
        rightGripperPosZ = MapValues(ikScript.gripperPercent, 0f, maxGripperOpenValue);
        RubberBandScaleZ = MapValues(ikScript.gripperPercent, 10.0f, 20.0f);

        leftGripper.localPosition = new(0,0,leftGripperPosZ);
        rightGripper.localPosition = new(0,0,rightGripperPosZ);

        RubberBand.localScale = new(RubberBand.localScale.x,RubberBand.localScale.y,RubberBandScaleZ);
    }

    float MapValues(float value, float toMin, float toMax)
    {
        // Calculate the normalized value in the new range
        float normalizedValue = (value - 0.0f) / (100.0f - 0.0f);

        // Map the normalized value to the original range
        float originalValue = toMin + (normalizedValue * (toMax - toMin));

        return originalValue;
    }
}
