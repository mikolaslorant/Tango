using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class CurveUIController : MonoBehaviour
{

    public TMPro.TMP_InputField m_xposInputField;
    public TMPro.TMP_InputField m_yposInputField;
    public TMPro.TMP_InputField m_zposInputField;

    public TMPro.TMP_InputField m_xangleInputField;
    public TMPro.TMP_InputField m_yangleInputField;
    public TMPro.TMP_InputField m_zangleInputField;

    public TMPro.TMP_InputField m_wquatInputField;
    public TMPro.TMP_InputField m_xquatInputField;
    public TMPro.TMP_InputField m_yquatInputField;
    public TMPro.TMP_InputField m_zquatInputField;

    public Dropdown m_quatTypeDropDown;
    public Dropdown m_eulerTypeDropDown;

    public Toggle m_rotationKeyToggle;
    public Toggle m_eulerToggle;
    public Toggle m_quatToggle;

    public CurveController m_curveController;
    public GameObject m_selectedPoint;

    // Start is called before the first frame update
    void Start()
    {
        SetRotationInputActive(false);
        SwitchRotationType();
    }

    // Update is called once per frame
    void Update()
    {
        SetTranslationText();
        SetRotationText();
    }

    void SetTranslationText()
    {
        if (m_selectedPoint == null) { return; }
        Vector3 pos = m_selectedPoint.transform.position;
        m_xposInputField.text = pos.x.ToString();
        m_yposInputField.text = pos.y.ToString();
        m_zposInputField.text = pos.z.ToString();
    }

    void SetRotationText()
    {
        if (!m_curveController.IsRotationKey(m_selectedPoint)) { return; }
        Quaternion quat = m_selectedPoint.transform.rotation;
        Vector3 euler = quat.eulerAngles;

        m_xangleInputField.text = euler.x.ToString();
        m_yangleInputField.text = euler.y.ToString();
        m_zangleInputField.text = euler.z.ToString();

        m_wquatInputField.text = quat.w.ToString();
        m_xquatInputField.text = quat.x.ToString();
        m_yquatInputField.text = quat.y.ToString();
        m_zquatInputField.text = quat.z.ToString();
    }

    void SetRotationInputActive(bool active)
    {
        bool showQuat = m_quatToggle.isOn;
        bool showEuler = m_eulerToggle.isOn;
        m_xangleInputField.gameObject.SetActive(active && showEuler);
        m_yangleInputField.gameObject.SetActive(active && showEuler);
        m_zangleInputField.gameObject.SetActive(active && showEuler);
        m_wquatInputField.gameObject.SetActive(active && showQuat);
        m_xquatInputField.gameObject.SetActive(active && showQuat);
        m_yquatInputField.gameObject.SetActive(active && showQuat);
        m_zquatInputField.gameObject.SetActive(active && showQuat);
    }

    public void SetSelectedPoint(GameObject obj)
    {
        m_selectedPoint = obj;
        if (m_selectedPoint == null) {
            m_rotationKeyToggle.gameObject.SetActive(false);
            m_rotationKeyToggle.isOn = false;
            SetRotationInputActive(false);
            return;
        }
        if (m_selectedPoint.CompareTag("KeyPoint"))
        {
            m_rotationKeyToggle.gameObject.SetActive(true);
            bool isRotationKey = m_curveController.IsRotationKey(m_selectedPoint);
            m_rotationKeyToggle.isOn = isRotationKey;
            SetRotationInputActive(isRotationKey);
        }
        else
        {
            m_rotationKeyToggle.gameObject.SetActive(false);
            SetRotationInputActive(false);
        }
    }

    public void SwitchRotationType()
    {
        bool showQuat = m_quatToggle.isOn;
        bool showEuler = m_eulerToggle.isOn;
        m_eulerTypeDropDown.gameObject.SetActive(showEuler);
        m_quatTypeDropDown.gameObject.SetActive(showQuat);
        if (showQuat) { m_curveController.SetRotationModeEuler(); }
        if (showEuler) { m_curveController.SetRotationModeQuat(); }
        SetRotationInputActive(m_curveController.IsRotationKey(m_selectedPoint));
    }

    public void SwitchRotationKey()
    {
        if (m_rotationKeyToggle.isOn)
        {
            SetRotationInputActive(true);
            if (!m_curveController.IsRotationKey(m_selectedPoint))
            {
                m_curveController.InsertRotationKey(m_selectedPoint);
                m_selectedPoint.GetComponent<PointColorController>().SetRotationKey(true);
            }
            
        }
        else
        {
            SetRotationInputActive(false);
            if (m_curveController.IsRotationKey(m_selectedPoint))
            {
                m_curveController.DeleteRotationKey(m_selectedPoint);
                m_selectedPoint.GetComponent<PointColorController>().SetRotationKey(false);
            }
            
        }
    }

    public void EditEulerKey()
    {
        float x = float.Parse(m_xangleInputField.text);
        float y = float.Parse(m_yangleInputField.text);
        float z = float.Parse(m_zangleInputField.text);

        m_curveController.EditRotationKey(m_selectedPoint, Quaternion.Euler(x, y, z));
    }

    public void EditQuatKey()
    {
        float w = float.Parse(m_wquatInputField.text);
        float x = float.Parse(m_xquatInputField.text);
        float y = float.Parse(m_yquatInputField.text);
        float z = float.Parse(m_zquatInputField.text);
        m_curveController.EditRotationKey(m_selectedPoint, new Quaternion(x, y, z, w));
    }

    public void EditVecKeyOrControlPoint()
    {
        if (!m_selectedPoint) { return; }
        float x = float.Parse(m_xposInputField.text);
        float y = float.Parse(m_yposInputField.text);
        float z = float.Parse(m_zposInputField.text);
        if (m_selectedPoint.CompareTag("KeyPoint"))
        {
            m_curveController.EditVecKey(m_selectedPoint, new Vector3(x, y, z));
        }
        else if (m_selectedPoint.CompareTag("ControlPoint"))
        {
            m_curveController.EditVecControlPoint(m_selectedPoint, new Vector3(x, y, z));
        }
    }

    public void Clear()
    {
        SetSelectedPoint(null);
    }


}
