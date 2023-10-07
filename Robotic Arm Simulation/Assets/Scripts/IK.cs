using UnityEngine;
using System;
using Unity.Collections.LowLevel.Unsafe;
using System.Xml.Serialization;

[System.Serializable]
public class Initialization{
    public Transform Base;
    public Transform root;
    public Transform mid;   
    public Transform tip;
    public Transform target;
    public Transform leftFinger;//-ve
    public Transform rightFinger;//+ve

    /// <summary>
    /// Jsut For Simulation
    /// </summary>
    public Transform servoExtension;
    public Transform rubberBand;
}


[System.Serializable]
public class Outputs{
    [Range(-360f, 360f)]
    public float BaseRotationAngle = 0.0f;
    [Range(-360f, 360f)]
    public float RootRotationAngle = 0.0f;
    [Range(-360f, 360f)]
    public float MidRotationAngle = 0.0f;
    [Range(-360f, 360f)]
    public float TipRotationAngle = 0.0f;
    [Range(-360f, 360f)]
    public float GrabberRotationAngle = 0.0f;
}

[System.Serializable]
public class FloorValues{
    public float floorPos = 0f;
    public float targetFloorOffset = 0.5f;
}

[System.Serializable]
public class GizmoSettings{
    public Color floorColor;
    public Color TargetColor = Color.red;
    public float TargetRadius = 0.05f;

    public bool ShowGizmos = true;
    public bool ShowTargetAngleGizmos = true;
    public bool ShowFloorGizmo = true;
    public bool ShowExtremetiyOffsetGizmo = true;    
    public Mesh cylinderMesh;
}
public class IK : MonoBehaviour
{
    [Header("Initialization")]
    public Initialization initialization;

    [Header ("Some Predifined Value")]
    public float maxFingerOpen = 0.075f;

    [Header("Motor Speed")]
    [Range (0.5f,5.0f)]
    public float MotorRotationSpeed = 0.5f;

    [Header("Basic Arm Contol Features")]
    [Range(0.0f, 1.0f)]
    public float extremityOffset = 0.0f;
    [Range (0.0f, 100.0f)]
    public float WristRotation = 0.0f;

    [Range (0.0f, 100.0f)]
    public float wristOpenPercent = 0.0f;

    [Header("Floor Detection Settings")]
    public FloorValues floorSettings;

    /// <summary>
    /// Private Variables
    /// </summary>
    private float wristRot = 0.0f;
    private float extremityRotation = 0.0f;
    private float refBase,refRoot,refMid,refTip;
    private Vector3 spherePosition;
    private Vector2 boneLenght = Vector2.zero;
    private float boneTotalLength = 0;
    private Vector3 VirtualTargetPos = Vector3.zero;
    private Vector3 theta;
    private Transform Base;
    private Transform root;
    private Transform mid;   
    private Transform tip;
    private Transform target;
    private Transform leftFinger;
    private Transform rightFinger;
    private Transform servoExtension;
    private Transform rubberBand;


    [Header("Outputs")]
    public Outputs outputs;
    
    [Header("Gizmo Settings")]
    public GizmoSettings gizmoSettings;

    private void Start() {
        Base = initialization.Base;
        root = initialization.root;
        mid = initialization.mid;
        tip = initialization.tip;
        target = initialization.target;
        leftFinger = initialization.leftFinger;
        rightFinger = initialization.rightFinger;
        servoExtension = initialization.servoExtension;
        rubberBand = initialization.rubberBand;



        boneLenght.x = Vector3.Distance(root.position, mid.position);
        boneLenght.y = Vector3.Distance(mid.position, tip.position);
        boneTotalLength = (boneLenght.x + boneLenght.y);
    }
    public void SolveIK(float x, float y, float phi)
    {

        float xPosT = x ;
        float yPosT = y ;
        float sqrtValue = Mathf.Sqrt(4 * boneLenght.x * boneLenght.x * (xPosT * xPosT + yPosT * yPosT));
        float inputValue = (boneLenght.x * boneLenght.x - boneLenght.y * boneLenght.y + xPosT * xPosT + yPosT * yPosT) / sqrtValue;
        float tanValue = Mathf.Atan2(yPosT , xPosT);
        float cosValue = Mathf.Acos(inputValue);
        if (inputValue > -1.0f && inputValue < 1.0f)
        {
            theta.x = (tanValue + cosValue) * Mathf.Rad2Deg;
        }else{
            theta.x = (tanValue + Mathf.Acos(0.9999999f)) * Mathf.Rad2Deg;
        }
        theta.y = (Mathf.Atan2(yPosT - boneLenght.x * Mathf.Sin(theta.x * Mathf.Deg2Rad), xPosT - boneLenght.x * Mathf.Cos(theta.x * Mathf.Deg2Rad)) * Mathf.Rad2Deg) - theta.x;
        theta.z = phi - theta.x - theta.y;
    }

    float angleCalc;
    private void Update()
    {

        Vector3 lookDirection2 = target.position - new Vector3(Base.position.x, target.position.y,Base.position.z);
        if (lookDirection2 != Vector3.zero)
        {
            float angleOffset = 90.0f;
            Quaternion rotation = Quaternion.LookRotation(lookDirection2.normalized, Vector3.up);
            rotation *= Quaternion.Euler(0, angleOffset, 0);
            spherePosition = target.position + rotation * Quaternion.Euler(0, 0, extremityRotation * 360.0f) * (Vector3.right * extremityOffset);
        }


        Vector3 targetPos = spherePosition;

        float redLine = Vector3.Distance(root.position,targetPos);
        float blueLine = Vector3.Distance(targetPos, new Vector3(targetPos.x, ( root.position + root.forward * 100).y, targetPos.z));
        if(blueLine<redLine){
            float multipler = (root.position.y > targetPos.y)?-1:1;
            angleCalc = Mathf.Asin(blueLine/redLine) * multipler ;
        }

        float distance = Vector3.Distance(root.position, VirtualTargetPos);
        float xPos = distance * Mathf.Cos(angleCalc);


        float distanceBetweenRootAndTarget = Vector3.Distance(root.position,targetPos);
        if(distanceBetweenRootAndTarget >= boneTotalLength){
            VirtualTargetPos = new Vector3(boneTotalLength * Mathf.Cos(angleCalc), boneTotalLength * Mathf.Sin(angleCalc), 0);
        }else{
            VirtualTargetPos = new Vector3(xPos,targetPos.y, 0);
        }

        if(targetPos.y <=  floorSettings.floorPos + floorSettings.targetFloorOffset){
            VirtualTargetPos = new Vector3(VirtualTargetPos.x, floorSettings.floorPos+floorSettings.targetFloorOffset, VirtualTargetPos.z);
        }
        
        SolveIK(target.position.x, target.position.y, wristRot);

        Vector3 lookDirection = target.position - Base.position;
        if (lookDirection != Vector3.zero)
        {
            float BaseAngleRotation = Mathf.Atan2(lookDirection.x, lookDirection.z) * Mathf.Rad2Deg;
            if(BaseAngleRotation<0){
                BaseAngleRotation+=360.0f;
            }
            float angleBase = Mathf.SmoothDampAngle(Base.localEulerAngles.y,BaseAngleRotation, ref refBase, MotorRotationSpeed);
            outputs.BaseRotationAngle = angleBase;
            Base.rotation = Quaternion.Euler(0, angleBase, 0);
        }

        
        float angleRoot = Mathf.SmoothDampAngle(root.localEulerAngles.z,theta.x, ref refRoot, MotorRotationSpeed);
        outputs.RootRotationAngle = angleRoot;
        root.localRotation = Quaternion.Euler(0, 0, angleRoot).normalized;

        float anglemid = Mathf.SmoothDampAngle(mid.localEulerAngles.z,theta.y, ref refMid, MotorRotationSpeed);
        outputs.MidRotationAngle = anglemid;
        mid.localRotation = Quaternion.Euler(0, 0, anglemid).normalized;

        float angletip = Mathf.SmoothDampAngle(tip.localEulerAngles.z,theta.z, ref refTip, MotorRotationSpeed);
        outputs.TipRotationAngle = angletip;
        tip.localRotation = Quaternion.Euler(0, 0, angletip).normalized;

        //Use Rotate Towards if constant speed required

        extremityRotation  = MapValueWristRotation(-0.25f, 0.25f);
        wristRot  = MapValueWristRotation(90.0f, -90.0f);

        leftFinger.localPosition = new Vector3(0f,0f,MapValueWristOpen(0,maxFingerOpen*(-1.0f)));
        rightFinger.localPosition = new Vector3(0f,0f,MapValueWristOpen(0,maxFingerOpen));
        outputs.GrabberRotationAngle = MapValueWristOpen(90f,0f);
        servoExtension.localRotation = Quaternion.Euler(outputs.GrabberRotationAngle,0f,0f);
        rubberBand.localScale = new Vector3(rubberBand.localScale.x,rubberBand.localScale.y,MapValueWristOpen(10,20));
    }
    float MapValueWristRotation(float toMin, float toMax)
    {
        // Calculate the normalized value in the new range
        float normalizedValue = (WristRotation - 0.0f) / (100.0f - 0.0f);

        // Map the normalized value to the original range
        float originalValue = toMin + (normalizedValue * (toMax - toMin));

        return originalValue;
    }

    float MapValueWristOpen(float toMin, float toMax)
    {
        // Calculate the normalized value in the new range
        float normalizedValue = (wristOpenPercent - 0.0f) / (100.0f - 0.0f);

        // Map the normalized value to the original range
        float originalValue = toMin + (normalizedValue * (toMax - toMin));

        return originalValue;
    }
    
    private void OnDrawGizmosSelected()
    {
        initialization.target.localScale = Vector3.one * gizmoSettings.TargetRadius;
        if(gizmoSettings.ShowGizmos){

            if(gizmoSettings.ShowTargetAngleGizmos){
                Vector3 endPos = initialization.root.position + initialization.root.forward * 100;
                Gizmos.color = Color.red;
                Gizmos.DrawLine(initialization.root.position, initialization.target.position);
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(initialization.root.position, new Vector3(initialization.target.position.x, endPos.y, initialization.target.position.z));
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(initialization.target.position, new Vector3(initialization.target.position.x, endPos.y, initialization.target.position.z));
            }

            if(gizmoSettings.ShowFloorGizmo){
                float boneTotalLengthGizmo = Vector3.Distance(initialization.root.position, initialization.mid.position) + Vector3.Distance(initialization.mid.position, initialization.tip.position);            
                Vector3 size = new(boneTotalLengthGizmo * 2, floorSettings.targetFloorOffset, boneTotalLengthGizmo * 2);
                Gizmos.color = gizmoSettings.floorColor;
                Gizmos.DrawMesh(gizmoSettings.cylinderMesh,new Vector3(initialization.root.position.x,floorSettings.floorPos + floorSettings.targetFloorOffset * 0.5f,initialization.root.position.z),Quaternion.Euler(Vector3.zero), new Vector3(boneTotalLengthGizmo * 2,floorSettings.targetFloorOffset,boneTotalLengthGizmo * 2));
                Gizmos.color = Color.black;
                DrawCircle(new Vector3(initialization.root.position.x,floorSettings.floorPos,initialization.root.position.z), Quaternion.Euler(Vector3.zero), boneTotalLengthGizmo);
                DrawCircle(new Vector3(initialization.root.position.x,floorSettings.floorPos + floorSettings.targetFloorOffset,initialization.root.position.z), Quaternion.Euler(Vector3.zero), boneTotalLengthGizmo);
            }

            Vector3 lookDirection = initialization.target.position - new Vector3(initialization.Base.position.x, initialization.target.position.y,initialization.Base.position.z);
            if (lookDirection != Vector3.zero)
            {
                float angleOffset = 90.0f;
                Quaternion rotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
                rotation *= Quaternion.Euler(0, angleOffset, 0);

                int numPoints = 36;
                float angleIncrement = 360.0f / numPoints;

                Vector3 prevPoint = initialization.target.position + rotation * Vector3.right * extremityOffset;

                if(gizmoSettings.ShowExtremetiyOffsetGizmo){
                    for (int i = 1; i <= numPoints; i++)
                    {
                        float angle = i * angleIncrement * Mathf.Deg2Rad;
                        Vector3 point = initialization.target.position + rotation * new Vector3(Mathf.Cos(angle) * extremityOffset, Mathf.Sin(angle) * extremityOffset, 0.0f);
                        Gizmos.color = Color.white;
                        Gizmos.DrawLine(prevPoint, point);
                        prevPoint = point;
                    }
                }


                spherePosition = initialization.target.position + rotation * Quaternion.Euler(0, 0, extremityRotation * 360.0f) * (Vector3.right * extremityOffset);

                Gizmos.color = gizmoSettings.TargetColor;
                Gizmos.DrawSphere(spherePosition, gizmoSettings.TargetRadius);
            }
        }
    }
    void DrawCircle(Vector3 basePosition, Quaternion circleOrientation, float radius)
    {

        float angleStep = 360f / 32;

        Vector3 prevPoint1 = basePosition + circleOrientation * Vector3.forward * radius;

        for (int i = 1; i <= 32; i++)
        {
            float angle = i * angleStep;
            Vector3 point = basePosition + circleOrientation * Quaternion.Euler(0, angle, 0) * Vector3.forward * radius;
            Gizmos.DrawLine(prevPoint1, point);
            prevPoint1 = point;
        }
    }
}