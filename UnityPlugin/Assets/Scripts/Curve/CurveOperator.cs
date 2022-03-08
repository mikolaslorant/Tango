using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CurveOperator : MonoBehaviour
{
    public LayerMask m_environmentLayer;
    public KeyCode m_holdKeyCode = KeyCode.LeftControl;     // Hold this key and press left button to create a new key
    public CurveController m_curveController;
    public CurvePointGizmoController m_curvePointGizmoController;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(m_holdKeyCode) && Input.GetMouseButtonDown(0))
        {
            RaycastCreatKey();
        }
        if (Input.GetKey(m_holdKeyCode) && Input.GetKeyDown(KeyCode.Delete))
        {
            DeleteSelectedKey();
        }
    }

    public void RaycastCreatKey()
    {
        //Create a ray from the mouse click position
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, Mathf.Infinity, m_environmentLayer))
        {
            m_curveController.AppendVecKey(hit.point + hit.normal);     // Offset the key a bit along the normal
        }
    }

    public void DeleteSelectedKey()
    {
        GameObject selectedPoint = m_curvePointGizmoController.m_selectedPoint;
        if (selectedPoint && selectedPoint.CompareTag("KeyPoint"))
        {
            m_curveController.DeleteKey(selectedPoint);
            m_curvePointGizmoController.ChangeSelectedPoint(null);
        }
    }
}
