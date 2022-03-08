using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CurveController : MonoBehaviour
{
    private const float MIN_DISTANCE = 0.01f;   // A min distance to check if the key point should be updated

    public enum VecInterpolationType { LINEAR, CUBIC_BERNSTEIN, CUBIC_CASTELJAU, CUBIC_MATRIX, CUBIC_HERMITE, CUBIC_BSPLINE };
    public enum QuatInterpolationType { LINEAR, CUBIC };
    public enum EulerInterpolationType { LINEAR, CUBIC };
    public enum RotationMode { EULER_ANGLES, QUATERNION };

    public VecInterpolationType m_vecInterpolationType = VecInterpolationType.LINEAR;
    public QuatInterpolationType m_quatInterpolationType = QuatInterpolationType.LINEAR;
    public EulerInterpolationType m_eulerInterpolationType = EulerInterpolationType.LINEAR;
    public RotationMode m_rotationMode = RotationMode.QUATERNION;

    public GameObject m_controlPointPrefab;
    public GameObject m_keyPointPrefab;

    public GameObject m_controlPointLinePrefab;     // Prefab with the line renderer to render the control point line
    public LineRenderer m_curveLineRenderer;
    public LineRenderer m_controlPointLineRenderer;
    private List<LineRenderer> m_hermiteControlPointLineRenderers = new List<LineRenderer>();

    private int m_id;

    public GameObject m_selectedPoint;

    private List<GameObject> m_keyPointList = new List<GameObject>();
    private List<GameObject> m_controlPointList = new List<GameObject>();
    private List<GameObject> m_rotationKeyPointList = new List<GameObject>();

    public bool m_showControlPoint = false;

    private bool m_curveChanged = false;   // A dirty varible to record if the curve changed
    public bool m_drawCurve = true;

    private GameObject m_controlPointParent;
    private GameObject m_keyPointParent;
    private GameObject m_lineRendererParent;

    // Start is called before the first frame update
    void Start()
    {
        m_id = CurvePlugin.CreateCurve();

        // Create some empty parent objects
        m_keyPointParent = new GameObject("Key Points");
        m_keyPointParent.transform.SetParent(this.transform);

        m_controlPointParent = new GameObject("Control Points");
        m_controlPointParent.transform.SetParent(this.transform);

        m_lineRendererParent = new GameObject("Hermite Lines");
        m_lineRendererParent.transform.SetParent(this.transform);

        //AppendVecKey(new Vector3(0, 0, 0));
        //AppendVecKey(new Vector3(3, 0, 0));
        //CurvePlugin.CurveValue curveValue = new CurvePlugin.CurveValue();
        //CurvePlugin.GetValue(m_id, 0.5, ref curveValue);
        //Debug.Log(PluginHelpFunction.DoubleArrayToVector3(curveValue.vec));

        CurvePlugin.SetVecInterpolationType(m_id, (int)m_vecInterpolationType);
        CurvePlugin.SetQuatInterpolationType(m_id, (int)m_quatInterpolationType);
        CurvePlugin.SetEulerInterpolationType(m_id, (int)m_eulerInterpolationType);

        m_curveLineRenderer.sortingOrder = 0;
        m_controlPointLineRenderer.sortingOrder = 0;
    }

    // Update is called once per frame
    void Update()
    {
        if (m_curveChanged && m_drawCurve)
        {
            m_curveChanged = false;
            DrawCurve();
        }
    }

    public void ClearCurve()
    {
        CurvePlugin.ResetCurve(m_id);
        if (CurvePlugin.GetVecKeyNum(m_id) != 0)
        {
            Debug.LogError("Clear Error");
        }
        foreach (GameObject obj in m_keyPointList)
        {
            if (obj) { Destroy(obj); }
        }
        foreach (GameObject obj in m_controlPointList)
        {
            if (obj) { Destroy(obj); }
        }
        m_keyPointList.Clear();
        m_controlPointList.Clear();
        m_rotationKeyPointList.Clear();

        // Clear line renderers
        m_curveLineRenderer.positionCount = 0;
        m_controlPointLineRenderer.positionCount = 0;
        foreach (LineRenderer line in m_hermiteControlPointLineRenderers)
        {
            Destroy(line.gameObject);
        }
        m_hermiteControlPointLineRenderers.Clear();

        m_curveChanged = true;
    }


    #region Draw Curve
    // Reset all line renderer
    private void ResetLineRenderer()
    {
        // Clear line renderers
        m_curveLineRenderer.positionCount = 0;
        m_controlPointLineRenderer.positionCount = 0;
        int maxKeyNum = CurvePlugin.GetVecKeyNum(m_id);
        // Each key has a hermite control point line
        while (m_hermiteControlPointLineRenderers.Count < maxKeyNum)
        {
            GameObject lineObj = Instantiate(m_controlPointLinePrefab, m_lineRendererParent.transform);
            m_hermiteControlPointLineRenderers.Add(lineObj.GetComponent<LineRenderer>());
            // lineObj.SetActive(false);
        }
        // Hide all lines
        foreach (LineRenderer lineRenderer in m_hermiteControlPointLineRenderers)
        {
            lineRenderer.positionCount = 0;
        }
    }

    // Prepare key and control point game objects and set them inactive
    private void ResetControlPoints()
    {
        int maxControlPointNum = CurvePlugin.GetVecKeyNum(m_id) * 4 + 2;
        while (m_controlPointList.Count < maxControlPointNum)
        {
            GameObject controlPoint = Instantiate(m_controlPointPrefab, m_controlPointParent.transform);
            m_controlPointList.Add(controlPoint);
            // controlPoint.SetActive(false);
        }

        // Hide all control points
        foreach (GameObject obj in m_controlPointList)
        {
            obj.SetActive(false);
        }
    }

    public void DrawCurve()
    {
        ResetControlPoints();
        ResetLineRenderer();
        if (CurvePlugin.GetVecKeyNum(m_id) == 0) { return; }

        // Get cached curve points
        int cachedPointNum = 0;
        IntPtr cachedPointPtr = IntPtr.Zero;
        CurvePlugin.GetCachedCurve(m_id, ref cachedPointNum, ref cachedPointPtr);
        if (cachedPointNum < 2) { return; }
        List<Vector3> cachedPoints = PluginHelpFunction.DoubleArrayPointerToVector3List(cachedPointNum, cachedPointPtr);

        // Reset LineRenderer to draw the curve
        m_curveLineRenderer.positionCount = cachedPointNum;
        m_curveLineRenderer.SetPositions(cachedPoints.ToArray());

        // Get Control Points
        if (!m_showControlPoint || m_vecInterpolationType == VecInterpolationType.LINEAR) { return; }
        int controlPointNum = 0;
        IntPtr controlPointPtr = IntPtr.Zero;
        double[] startPoint = new double[3];
        double[] endPoint = new double[3];
        CurvePlugin.GetControlPoints(m_id, startPoint, endPoint, ref controlPointNum, ref controlPointPtr);
        // The start point and the end point are not included, so the size is controlPointNum - 2
        List<Vector3> controlPoints = PluginHelpFunction.DoubleArrayPointerToVector3List(controlPointNum - 2, controlPointPtr);
        // Draw control point lines and set the position of control points
        switch(m_vecInterpolationType)
        {
            case VecInterpolationType.CUBIC_BERNSTEIN:
            case VecInterpolationType.CUBIC_CASTELJAU:
            case VecInterpolationType.CUBIC_MATRIX:
                m_controlPointLineRenderer.positionCount = controlPointNum;
                // Add the start point and the end point
                controlPoints.Insert(0, PluginHelpFunction.DoubleArrayToVector3(startPoint));
                controlPoints.Add(PluginHelpFunction.DoubleArrayToVector3(endPoint));
                m_controlPointLineRenderer.SetPositions(controlPoints.ToArray());
                // Set control points
                for (int i = 0; i < controlPoints.Count; ++i)
                {
                    m_controlPointList[i].transform.position = controlPoints[i];
                    // Skip key points
                    if (i == 0 || (i - 1) % 4 == 1 || (i - 1) % 4 == 2 || i == controlPointNum - 1)
                    {
                        m_controlPointList[i].SetActive(true);
                    }
                }
                break;
            case VecInterpolationType.CUBIC_HERMITE:
                // Each key has a hermite control point line
                for (int i = 0; i < m_keyPointList.Count; ++i)
                {
                    // Set the line renderer
                    Vector3 slope = controlPoints[i];
                    Vector3 pos0 = m_keyPointList[i].transform.position;
                    Vector3 pos1 = pos0 + slope;
                    LineRenderer lineRenderer = m_hermiteControlPointLineRenderers[i];
                    lineRenderer.positionCount = 2;
                    lineRenderer.SetPosition(0, pos0);
                    lineRenderer.SetPosition(1, pos1);
                    // Set the control point
                    m_controlPointList[i + 1].transform.position = pos1;
                    m_controlPointList[i + 1].SetActive(true);
                }
                break;
            case VecInterpolationType.CUBIC_BSPLINE:
                // No start point and end point
                m_controlPointLineRenderer.positionCount = controlPointNum - 2;
                m_controlPointLineRenderer.SetPositions(controlPoints.ToArray());
                // Set control points
                for (int i = 0; i < controlPoints.Count; ++i)
                {
                    m_controlPointList[i + 1].transform.position = controlPoints[i];
                    // Skip the first and the last control points because they are at the same position as the key points
                    if (!(i == 1 || i == controlPointNum - 4))
                    {
                        m_controlPointList[i + 1].SetActive(true);
                    }
                }
                break;
            default:
                break;
        }
    }
    #endregion

    #region Key Updating
    public void AppendVecKey(Vector3 pos)
    {
        GameObject keyPoint = Instantiate(m_keyPointPrefab, pos, Quaternion.identity);
        keyPoint.transform.SetParent(m_keyPointParent.transform);
        m_keyPointList.Add(keyPoint);


        CurvePlugin.AppendVecKey(m_id, PluginHelpFunction.Vector3ToDoubleArray(pos));

        m_curveChanged = true;
    }

    // Insert a rotation key into both the euler curve and the quaternion curve
    // The keyID in these two curves should be the same
    public void InsertRotationKey(GameObject key)
    {
        if (m_rotationKeyPointList.IndexOf(key) != -1)
        {
            Debug.Log("This rotation key has been added.");
            return;
        }
        double t = GetVecKeyID(key);   // Assume t equals keyID
        int eulerID = CurvePlugin.InsertEulerKey(m_id, t, PluginHelpFunction.Vector3ToDoubleArray(Quaternion.identity.eulerAngles));
        int quatID = CurvePlugin.InsertQuatKey(m_id, t, PluginHelpFunction.QuaternionToDoubleArray(Quaternion.identity));
        if (eulerID != quatID)
        {
            Debug.LogError("Euler curve ID and Quat curve ID are different");
        }
        m_rotationKeyPointList.Insert(eulerID, key);
    }

    // Delete the vec key as well as the rotation key (if applicable)
    // If the key number = 0, then clear the whole curve
    public void DeleteKey(GameObject key)
    {
        int keyID = GetVecKeyID(key);
        CurvePlugin.DeleteVecKey(m_id, keyID);
        m_keyPointList.RemoveAt(keyID);

        DeleteRotationKey(key);

        Destroy(key);
        m_curveChanged = true;
    }

    public void DeleteRotationKey(GameObject key)
    {
        int keyID = GetRotationKeyID(key);
        if (keyID == -1)
        {
            Debug.Log("This key is not a rotation key.");
            return;
        }
        CurvePlugin.DeleteEulerKey(m_id, keyID);
        CurvePlugin.DeleteQuatKey(m_id, keyID);
        m_rotationKeyPointList.RemoveAt(keyID);
    }

    public void EditVecKey(GameObject key, Vector3 pos)
    {
        key.transform.position = pos;
        int keyID = GetVecKeyID(key);
        CurvePlugin.EditVecKey(m_id, keyID, PluginHelpFunction.Vector3ToDoubleArray(pos));

        m_curveChanged = true;
    }

    public void EditEulerKey(GameObject key, Vector3 euler)
    {
        int keyID = GetRotationKeyID(key);
        CurvePlugin.EditEulerKey(m_id, keyID, PluginHelpFunction.Vector3ToDoubleArray(euler));
    }

    public void EditQuatKey(GameObject key, Quaternion quat)
    {
        int keyID = GetRotationKeyID(key);
        CurvePlugin.EditQuatKey(m_id, keyID, PluginHelpFunction.QuaternionToDoubleArray(quat));
    }

    public void EditVecControlPoint(GameObject controlPoint, Vector3 pos)
    {
        controlPoint.transform.position = pos;
        int cpID = GetControlPointID(controlPoint);
        if (m_vecInterpolationType == VecInterpolationType.CUBIC_HERMITE)
        {
            pos -= m_keyPointList[cpID - 1].transform.position;
        }
        CurvePlugin.EditVecControlPoint(m_id, cpID, PluginHelpFunction.Vector3ToDoubleArray(pos));
        m_curveChanged = true;
    }

    public void EditRotationKey(GameObject key, Quaternion quat)
    {
        key.transform.rotation = quat;
        int keyID = GetRotationKeyID(key);
        if (keyID == -1)
        {
            Debug.Log("This key is not a rotation key.");
            return;
        }
        EditEulerKey(key, quat.eulerAngles);
        EditQuatKey(key, quat);
    }

    public bool IsRotationKey(GameObject key)
    {
        return m_rotationKeyPointList.Exists(item => item == key);
    }
    #endregion

    #region Interpolation Type
    public void SetVectInterpolationType(int type)
    {
        CurvePlugin.SetVecInterpolationType(m_id, type);
        m_vecInterpolationType = (VecInterpolationType)type;

        m_curveChanged = true;
    }

    public void SetQuatInterpolationType(int type)
    {
        CurvePlugin.SetQuatInterpolationType(m_id, type);
        m_quatInterpolationType = (QuatInterpolationType)type;
    }

    public void SetEulerInterpolationType(int type)
    {
        CurvePlugin.SetEulerInterpolationType(m_id, type);
        m_eulerInterpolationType = (EulerInterpolationType)type;
    }
    #endregion

    #region Getter and Setter
    public int GetVecKeyID(GameObject obj)
    {
        return m_keyPointList.IndexOf(obj);
    }

    public int GetRotationKeyID(GameObject obj)
    {
        return m_rotationKeyPointList.IndexOf(obj);
    }
    
    public int GetControlPointID(GameObject obj)
    {
        return m_controlPointList.IndexOf(obj);
    }

    public void SetShowControlPoints(bool show)
    {
        m_showControlPoint = show;
        m_curveChanged = true;
    }

    public void SetSelectedPoint(GameObject obj)
    {
        m_selectedPoint = obj;
    }

    public float GetDuration()
    {
        return (float)CurvePlugin.GetVecDuration(m_id);
    }

    public void SetRotationModeEuler()
    {
        m_rotationMode = RotationMode.EULER_ANGLES;
    }

    public void SetRotationModeQuat()
    {
        m_rotationMode = RotationMode.QUATERNION;
    }

    public int GetKeyNumber()
    {
        return m_keyPointList.Count;
    }

    public Vector3 GetKeyPosition(int keyID)
    {
        return m_keyPointList[keyID].transform.position;
    }

    public void GetCurveValue(float t, out Vector3 pos, out Quaternion quat)
    {
        CurvePlugin.CurveValue curveValue = new CurvePlugin.CurveValue();
        CurvePlugin.GetValue(m_id, t, ref curveValue);
        pos = PluginHelpFunction.DoubleArrayToVector3(curveValue.vec);
        switch (m_rotationMode)
        {
            case RotationMode.EULER_ANGLES:
                quat = Quaternion.Euler(PluginHelpFunction.DoubleArrayToVector3(curveValue.euler));
                break;
            case RotationMode.QUATERNION:
                quat = PluginHelpFunction.DoubleArrayToQuaternion(curveValue.quat);
                break;
            default:
                quat = Quaternion.identity;
                break;

        }
    }
    #endregion
}
