/*
using UnityEngine;

public class TargetController : MonoBehaviour
{
    public Transform target;

    Vector3 mOffset;
    float mZCoord;
    private bool isDragging = false;

    public bool invertAxis = false;

    
    void OnMouseDown() {
        mZCoord = Camera.main.WorldToScreenPoint(target.position).z;
        mOffset = target.position - GetMouseWorldPos();
        isDragging = true;
    }

    Vector3 GetMouseWorldPos(){
        Vector3 mousePoint = Input.mousePosition * ((invertAxis)?-1:1);
        mousePoint.z = mZCoord;
        return Camera.main.ScreenToWorldPoint(mousePoint);
    }

    void OnMouseDrag(){
        target.position = GetMouseWorldPos() + mOffset;
    }

    void OnMouseUp()
    {
        isDragging = false;
    }

    void Update()
    {

        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                GameObject clickedObject = hit.collider.gameObject;

                if (clickedObject == this.gameObject)
                {
                    OnMouseDown();
                }
            }
        }
        if(Input.GetMouseButtonUp(0)){
            OnMouseUp();
        }

        if (isDragging)
        {
            OnMouseDrag();
        }
    }
}
*/

using UnityEngine;

public class TargetController : MonoBehaviour
{
    public Transform target;
    Vector3 mOffset;
    bool isDragging = false;
    public bool invertAxis = false;

    void Update()
    {
        if (Input.GetMouseButtonDown(0) && !isDragging)
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                GameObject clickedObject = hit.collider.gameObject;
                if (clickedObject == this.gameObject)
                {
                    OnMouseDown();
                }
            }
        }

        if (Input.GetMouseButtonUp(0) && isDragging)
        {
            OnMouseUp();
        }

        if (isDragging)
        {
            OnMouseDrag();
        }
    }

    void OnMouseDown()
    {
        Vector3 mousePosition = Input.mousePosition;
        mOffset = target.position - GetMouseWorldPos(mousePosition);
        isDragging = true;
    }

    void OnMouseDrag()
    {
        Vector3 mousePosition = Input.mousePosition;
        target.position = GetMouseWorldPos(mousePosition) + mOffset;
    }

    void OnMouseUp()
    {
        isDragging = false;
    }

    Vector3 GetMouseWorldPos(Vector3 mousePosition)
    {
        Vector3 mousePoint = mousePosition * (invertAxis ? -1 : 1);
        mousePoint.z = Camera.main.WorldToScreenPoint(target.position).z;
        return Camera.main.ScreenToWorldPoint(mousePoint);
    }
}
