using System;
using UnityEngine;

[System.Serializable]
public class GizmoSetting{
    public Color floorColor = Color.red, targetColor = Color.black;
    public float TargetSize = 0.5f;
    public bool ShowGizmos = true;
    public bool ShowRootTargetGizmo = true;
    public bool showOffsetGizmo = true;
    public bool ShowTargetGizmo = true;
    public bool showFloorGizmo = true;
    public Mesh cylinderMesh;
}

[System.Serializable]
public class FloorDetection{
    public Transform floor;
    public float floorOffset = 0.5f;
}

[System.Serializable]
public class Initialize{
    public Transform Base,root,mid,tip,target,floor;
    public Transform ServoExtension;

}

public class NewIK : MonoBehaviour
{
    [Header("Initialization")]
    public Initialize initialize;
    [Header("Floor Settings")]
    public FloorDetection floorDetection;

    [Header("Motor Settings")]
    public float MotorRotationSpeed = 0.5f;
    public bool isRotateSmoothly = true;

    [Header("Some Arm Settings")]
    public float extremityOffset;
    [Range(0.0f,100.0f)]
    public float WristRotatePercent = 50f;
    [Range(0.0f,100.0f)]
    public float gripperPercent = 0f;

    [Header("Gizmo Settings")]
    public GizmoSetting gizmoSettings;


    private Vector3 theta;
    private Vector3 boneLengths;
    private Vector3 A,B,C;
    private Vector2 targetPos;
    private Vector3 VirtualTargetPos;
    private float angle;


    private float servoGripperRotationX;    
    private float extremityRotation;
    private float TipRotation = 0;
    void Start()
    {
        boneLengths = new(Vector3.Distance(initialize.root.position,initialize.mid.position), Vector3.Distance(initialize.mid.position,initialize.tip.position), Vector3.Distance(initialize.root.position,initialize.mid.position) + Vector3.Distance(initialize.mid.position,initialize.tip.position));
    }
    void Update()
    {
        UpdateValues();

        SolveIK(new(VirtualTargetPos.x, VirtualTargetPos.y), TipRotation);

        RotateMotors();
    }

    float MapValues(float value, float toMin, float toMax)
    {
        // Calculate the normalized value in the new range
        float normalizedValue = (value - 0.0f) / (100.0f - 0.0f);

        // Map the normalized value to the original range
        float originalValue = toMin + (normalizedValue * (toMax - toMin));

        return originalValue;
    }

    private float refBase,refRoot,refMid,refTip,refGripper;

    void RotateMotors(){
        if(isRotateSmoothly){
            RotateSmoothly();
        }else{
            RotateAtContantSpeed();
        }
    }

    void RotateSmoothly(){
        Vector3 lookDirection = spherePosition - initialize.Base.position;
        if (lookDirection != Vector3.zero)
        {
            float BaseAngleRotation = Mathf.Atan2(lookDirection.x, lookDirection.z) * Mathf.Rad2Deg;
            if(BaseAngleRotation<0){
                BaseAngleRotation+=360.0f;
            }
            float angleBase = Mathf.SmoothDampAngle(initialize.Base.localEulerAngles.y,BaseAngleRotation, ref refBase, MotorRotationSpeed);
            initialize.Base.rotation = Quaternion.Euler(0, angleBase, 0);
        }

        float angleRoot = Mathf.SmoothDampAngle(initialize.root.localEulerAngles.z,theta.x, ref refRoot, MotorRotationSpeed);
        initialize.root.localRotation = Quaternion.Euler(0,0,angleRoot);

        float anglemid = Mathf.SmoothDampAngle(initialize.mid.localEulerAngles.z,theta.y-90f, ref refMid, MotorRotationSpeed);
        initialize.mid.localRotation = Quaternion.Euler(0,0,anglemid);

        float angletip = Mathf.SmoothDampAngle(initialize.tip.localEulerAngles.z,theta.z, ref refTip, MotorRotationSpeed);
        initialize.tip.localRotation = Quaternion.Euler(0, 0, angletip);
        
        //float angleGripper = Mathf.SmoothDampAngle(initialize.ServoExtension.localEulerAngles.x,servoGripperRotationX, ref refGripper, MotorRotationSpeed);
        initialize.ServoExtension.localRotation = Quaternion.Euler(servoGripperRotationX,0,0);
    }

    void RotateAtContantSpeed(){
        Vector3 lookDirection = spherePosition - initialize.Base.position;
        if (lookDirection != Vector3.zero)
        {
            float BaseAngleRotation = Mathf.Atan2(lookDirection.x, lookDirection.z) * Mathf.Rad2Deg;
            if(BaseAngleRotation<0){
                BaseAngleRotation+=360.0f;
            }
            initialize.Base.rotation = Quaternion.RotateTowards(initialize.Base.rotation, Quaternion.Euler(0, BaseAngleRotation, 0), MotorRotationSpeed * Time.deltaTime);
        }

        initialize.root.localRotation = Quaternion.RotateTowards(initialize.root.localRotation, Quaternion.Euler(0,0,theta.x), MotorRotationSpeed * Time.deltaTime);

        initialize.mid.localRotation = Quaternion.RotateTowards(initialize.mid.localRotation, Quaternion.Euler(0,0,theta.y-90f), MotorRotationSpeed * Time.deltaTime);

        initialize.tip.localRotation = Quaternion.RotateTowards(initialize.tip.localRotation, Quaternion.Euler(0,0,theta.z), MotorRotationSpeed * Time.deltaTime);
        
        //initialize.ServoExtension.localRotation = Quaternion.RotateTowards(initialize.ServoExtension.localRotation, Quaternion.Euler(servoGripperRotationX,0,0), MotorRotationSpeed * Time.deltaTime);
        initialize.ServoExtension.localRotation = Quaternion.Euler(servoGripperRotationX,0,0);
    }

    public void SolveIK(Vector2 targetPosition, float phi)
    {

        float xPosT = targetPosition.x ;
        float yPosT = targetPosition.y ;
        float sqrtValue = Mathf.Sqrt(4 * boneLengths.x * boneLengths.x * (xPosT * xPosT + yPosT * yPosT));
        float inputValue = (boneLengths.x * boneLengths.x - boneLengths.y * boneLengths.y + xPosT * xPosT + yPosT * yPosT) / sqrtValue;
        float tanValue = Mathf.Atan2(yPosT , xPosT);
        float cosValue = Mathf.Acos(inputValue);
        if (inputValue > -1.0f && inputValue < 1.0f)
        {
            theta.x = (tanValue + cosValue) * Mathf.Rad2Deg;
        }else{
            theta.x = (tanValue + Mathf.Acos(0.9999999f)) * Mathf.Rad2Deg;
        }
        theta.y = (Mathf.Atan2(yPosT - boneLengths.x * Mathf.Sin(theta.x * Mathf.Deg2Rad), xPosT - boneLengths.x * Mathf.Cos(theta.x * Mathf.Deg2Rad)) * Mathf.Rad2Deg) - theta.x;
        theta.z = phi - theta.x - theta.y;
    }
    [HideInInspector]
    public Vector3 spherePosition;
    void UpdateValues(){
        extremityRotation = MapValues(WristRotatePercent, -0.25f,0.25f);
        TipRotation = MapValues(WristRotatePercent, 90f, -90f);

        servoGripperRotationX = MapValues(gripperPercent, 90.0f, 0f);

        //Initializing & Updating Some Variable
        boneLengths = new(Vector3.Distance(initialize.root.position,initialize.mid.position), Vector3.Distance(initialize.mid.position,initialize.tip.position), Vector3.Distance(initialize.root.position,initialize.mid.position) + Vector3.Distance(initialize.mid.position,initialize.tip.position));
        A = initialize.root.position;
        B = initialize.target.position;
        C = new(initialize.target.position.x, initialize.root.position.y, initialize.target.position.z);

        //For Adding Extremities and rotation
        Vector3 lookDirection = initialize.target.position - new Vector3(initialize.Base.position.x, initialize.target.position.y,initialize.Base.position.z);
        if (lookDirection != Vector3.zero)
        {
            float angleOffset = 90.0f;
            Quaternion rotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
            rotation *= Quaternion.Euler(0, angleOffset, 0);
            spherePosition = initialize.target.position + rotation * Quaternion.Euler(0, 0, extremityRotation * 360.0f) * (Vector3.right * extremityOffset);

            B = spherePosition;
            C = new(spherePosition.x, initialize.root.position.y, spherePosition.z);
            
            float negativeMultiplier = (A.y > B.y)? -1 : 1;
            targetPos = new(Vector3.Distance(A,C), Vector3.Distance(B,C) * negativeMultiplier);
        }

        angle = Mathf.Atan2(targetPos.y, targetPos.x) * Mathf.Rad2Deg;

        float maxLength = Mathf.Min(boneLengths.z - 0.00009f, Vector3.Distance(A,B));

        VirtualTargetPos = new(maxLength * Mathf.Cos(angle*Mathf.Deg2Rad), Mathf.Max(floorDetection.floor.position.y + floorDetection.floorOffset,maxLength * Mathf.Sin(angle*Mathf.Deg2Rad)), 0);
    }

    void OnDrawGizmos(){
        if(gizmoSettings.ShowGizmos){
            UpdateValues();
            if(gizmoSettings.ShowRootTargetGizmo){
                Gizmos.color = Color.red;
                Gizmos.DrawLine(A,B);

                Gizmos.color = Color.blue;
                Gizmos.DrawLine(B,C);

                Gizmos.color = Color.green;
                Gizmos.DrawLine(A,C);
            }

            if(gizmoSettings.ShowTargetGizmo){
                Gizmos.color = gizmoSettings.targetColor;
                Gizmos.DrawSphere(VirtualTargetPos, gizmoSettings.TargetSize);
            }

            if(gizmoSettings.showFloorGizmo){
                float boneTotalLengthGizmo = Vector3.Distance(initialize.root.position, initialize.mid.position) + Vector3.Distance(initialize.mid.position, initialize.tip.position);
                Gizmos.color = gizmoSettings.floorColor;
                Gizmos.DrawMesh(gizmoSettings.cylinderMesh,new Vector3(initialize.root.position.x,floorDetection.floor.position.y + floorDetection.floorOffset * 0.5f,initialize.root.position.z),Quaternion.Euler(Vector3.zero), new Vector3(boneTotalLengthGizmo * 2,floorDetection.floorOffset,boneTotalLengthGizmo * 2));
                Gizmos.color = Color.black;
                DrawCircle(new Vector3(initialize.root.position.x,floorDetection.floor.position.y,initialize.root.position.z), Quaternion.Euler(Vector3.zero), boneTotalLengthGizmo);
                DrawCircle(new Vector3(initialize.root.position.x,floorDetection.floor.position.y + floorDetection.floorOffset,initialize.root.position.z), Quaternion.Euler(Vector3.zero), boneTotalLengthGizmo);
            }
        }

        if(gizmoSettings.showOffsetGizmo){
            Vector3 lookDirection = initialize.target.position - new Vector3(initialize.Base.position.x, initialize.target.position.y,initialize.Base.position.z);
            if (lookDirection != Vector3.zero)
            {
                float angleOffset = 90.0f;
                Quaternion rotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
                rotation *= Quaternion.Euler(0, angleOffset, 0);

                int numPoints = 36;
                float angleIncrement = 360.0f / numPoints;

                Vector3 prevPoint = initialize.target.position + rotation * Vector3.right * extremityOffset;

                if(true){
                    for (int i = 1; i <= numPoints; i++)
                    {
                        float angle = i * angleIncrement * Mathf.Deg2Rad;
                        Vector3 point = initialize.target.position + rotation * new Vector3(Mathf.Cos(angle) * extremityOffset, Mathf.Sin(angle) * extremityOffset, 0.0f);
                        Gizmos.color = Color.white;
                        Gizmos.DrawLine(prevPoint, point);
                        prevPoint = point;
                    }
                }


                Vector3 spherePositionGizmo = initialize.target.position + rotation * Quaternion.Euler(0, 0, extremityRotation * 360.0f) * (Vector3.right * extremityOffset);

                Gizmos.color = gizmoSettings.targetColor;
                Gizmos.DrawSphere(spherePositionGizmo, gizmoSettings.TargetSize);
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