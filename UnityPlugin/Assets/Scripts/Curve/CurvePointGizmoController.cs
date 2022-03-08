using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RTG;
using UnityEngine.EventSystems;

public class CurvePointGizmoController : MonoBehaviour
{
    public bool m_enableRotationGizmo = false;
    public CurveController m_curveController;
    public CurveUIController m_curveUIController;
    public GameObject m_selectedPoint;
    private GameObject m_hoveredPoint;
    private ObjectTransformGizmo m_objectRotationGizmo;
    private ObjectTransformGizmo m_objectTranslationGizmo;


    // Start is called before the first frame update
    void Start()
    {
        m_objectRotationGizmo = RTGizmosEngine.Get.CreateObjectRotationGizmo();
        m_objectRotationGizmo.SetTransformSpace(GizmoSpace.Local);
        m_objectRotationGizmo.Gizmo.SetEnabled(false);
        m_objectRotationGizmo.Gizmo.PostUpdateEnd += OnRotationGizmoPostUpdateEnd;

        m_objectTranslationGizmo = RTGizmosEngine.Get.CreateObjectMoveGizmo();
        m_objectTranslationGizmo.SetTransformSpace(GizmoSpace.Local);
        m_objectTranslationGizmo.Gizmo.SetEnabled(false);
        m_objectTranslationGizmo.Gizmo.PostUpdateEnd += OnTranslationGizmoPostUpdateEnd;
    }

    // Update is called once per frame
    void Update()
    {
        // Check selected point if mouse doesn't hit UI or the gizmo
        if (Input.GetMouseButtonDown(0) && RTGizmosEngine.Get.HoveredGizmo == null && 
            EventSystem.current.currentSelectedGameObject == null)
        {
            GameObject pickedObject = PickCurvePoint();
            if (pickedObject != m_selectedPoint)
            {
                ChangeSelectedPoint(pickedObject);
            }
        }
        
        if (Input.GetKeyDown(KeyCode.W)) { EnableTranslationGizmo(); }

        // Only key point can be rotated
        if (m_enableRotationGizmo && Input.GetKeyDown(KeyCode.E) && 
            m_curveController.IsRotationKey(m_selectedPoint)) { EnableRotationGizmo(); }

        // Update transform of gizmo because the transform of the selectPoint may be changed by UI
        if (m_selectedPoint)
        {
            m_objectTranslationGizmo.SetTargetObject(m_selectedPoint);
            m_objectRotationGizmo.SetTargetObject(m_selectedPoint);
        }

    }

    private GameObject PickCurvePoint()
    {
        // Build a ray using the current mouse cursor position
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

        RaycastHit hit;
        // Check if the ray hit the curve point
        if (Physics.Raycast(ray, out hit, Mathf.Infinity))
        {
            if (hit.collider.gameObject.CompareTag("KeyPoint") ||
                hit.collider.gameObject.CompareTag("ControlPoint"))
            {
                return hit.collider.gameObject;
            }
            else { return null; }
        }

        // No object is intersected by the ray. Return null.
        return null;
    }

    public void ChangeSelectedPoint(GameObject newSeletcedObject)
    {
        // Store the new target object
        m_selectedPoint = newSeletcedObject;
        m_curveController.SetSelectedPoint(m_selectedPoint);
        if (m_curveUIController) m_curveUIController.SetSelectedPoint(m_selectedPoint);

        EnableTranslationGizmo();
    }

    public void EnableTranslationGizmo()
    {
        m_objectRotationGizmo.Gizmo.SetEnabled(false);
        m_objectTranslationGizmo.Gizmo.SetEnabled(false);
        // Check if the new target object is valid
        if (m_selectedPoint != null)
        {
            m_objectTranslationGizmo.SetTargetObject(m_selectedPoint);
            m_objectTranslationGizmo.Gizmo.SetEnabled(true);
        }
    }

    public void EnableRotationGizmo()
    {
        m_objectRotationGizmo.Gizmo.SetEnabled(false);
        m_objectTranslationGizmo.Gizmo.SetEnabled(false);
        // Check if the new target object is valid
        if (m_selectedPoint != null)
        {
            m_objectRotationGizmo.SetTargetObject(m_selectedPoint);
            m_objectRotationGizmo.Gizmo.SetEnabled(true);
            m_objectTranslationGizmo.SetTargetObject(m_selectedPoint);
            m_objectTranslationGizmo.Gizmo.SetEnabled(true);
        }
    }

    private void OnRotationGizmoPostUpdateEnd(RTG.Gizmo gizmo)
    {
        if (m_selectedPoint == null) { return; }
        // Update the axes
        m_objectTranslationGizmo.Gizmo.SetEnabled(false);
        m_objectTranslationGizmo.SetTargetObject(m_selectedPoint);
        m_objectTranslationGizmo.Gizmo.SetEnabled(true);
        
        if (m_selectedPoint.CompareTag("KeyPoint"))
        {
            m_curveController.EditRotationKey(m_selectedPoint, m_selectedPoint.transform.rotation);
        }

    }

    private void OnTranslationGizmoPostUpdateEnd(RTG.Gizmo gizmo)
    {
        if (m_selectedPoint == null) { return; }

        if (m_selectedPoint.CompareTag("KeyPoint"))
        {
            m_curveController.EditVecKey(m_selectedPoint, m_selectedPoint.transform.position);
        }
        else if (m_selectedPoint.CompareTag("ControlPoint"))
        {
            m_curveController.EditVecControlPoint(m_selectedPoint, m_selectedPoint.transform.position);
        }
    }

    public void Clear()
    {
        ChangeSelectedPoint(null);
    }
}
