using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DebugObjectsControl : MonoBehaviour
{

    [Header("Debugging")]
    public NewIK ikScript;
    public GameObject SpherePrefab;
    public LineRenderer lineRenderer;
    public MeshRenderer meshRenderer;
    public Toggle toggle;
    
    // Start is called before the first frame update
    void Start()
    {
        GameObject instantiatedSpherePrefab = Instantiate(SpherePrefab, ikScript.initialize.target);
        SpherePrefab = instantiatedSpherePrefab;
        meshRenderer = SpherePrefab.GetComponent<MeshRenderer>();
    }

    // Update is called once per frame
    void Update()
    {
        if(toggle.isOn){
            lineRenderer.enabled = true;
            meshRenderer.enabled = true;
            DrawExtremityOffsetLine();
            DrawSphere();
        }else{
            lineRenderer.enabled = false;
            meshRenderer.enabled = false;
        }
    }

    void DrawExtremityOffsetLine(){
        Vector3 lookDirection = ikScript.initialize.target.position - new Vector3(ikScript.initialize.Base.position.x, ikScript.initialize.target.position.y, ikScript.initialize.Base.position.z);
        if (lookDirection != Vector3.zero)
        {
            float angleOffset = 90.0f;
            Quaternion rotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
            rotation *= Quaternion.Euler(0, angleOffset, 0);

            int numPoints = 36;
            float angleIncrement = 360.0f / numPoints;
            Vector3[] linePositions = new Vector3[numPoints + 1]; // +1 to include the initial point

            Vector3 prevPoint = ikScript.initialize.target.position + rotation * Vector3.right * ikScript.extremityOffset;

            for (int i = 0; i <= numPoints; i++)
            {
                float angle = i * angleIncrement * Mathf.Deg2Rad;
                Vector3 point = ikScript.initialize.target.position + rotation * new Vector3(Mathf.Cos(angle) * ikScript.extremityOffset, Mathf.Sin(angle) * ikScript.extremityOffset, 0.0f);
                linePositions[i] = point;
            }

            lineRenderer.positionCount = linePositions.Length;
            lineRenderer.SetPositions(linePositions);
        }
        else
        {
            lineRenderer.positionCount = 0; // Hide the line if lookDirection is zero
        }
    }

    void DrawSphere(){
        SpherePrefab.transform.position = ikScript.spherePosition;
    }
}
