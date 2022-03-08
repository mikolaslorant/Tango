using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AnimatedObjectController : MonoBehaviour
{
    public float m_speed = 0.5f;     // The t increment per second
    public Material m_transparentMaterial;
    public Material m_defaultMaterial;

    public CurveController m_curveController;

    [SerializeField] private bool m_canMove = false;

    [SerializeField] private float m_t = 0;

    public bool isPreview = false;
    public GameObject previewKey;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        //if (isPreview)
        //{
        //    if (previewKey == null)
        //    {
        //        StopPreview();
        //        return;
        //    }
        //    this.transform.position = previewKey.transform.position;
        //    this.transform.rotation = previewKey.transform.rotation;
        //}
    }

    private void FixedUpdate()
    {
        if (!m_canMove) { return; }
        if (m_t < m_curveController.GetDuration()) { m_t += Time.fixedDeltaTime * m_speed; }
        m_curveController.GetCurveValue(m_t, out Vector3 pos, out Quaternion quat);
        this.transform.position = pos;
        this.transform.rotation = quat;
    }

    public void StartMove()
    {
        if (m_curveController.GetKeyNumber() < 2) { return; }
        m_t = 0;
        m_curveController.GetCurveValue(m_t, out Vector3 pos, out Quaternion quat);
        this.transform.position = pos;
        this.transform.rotation = quat;
        this.gameObject.SetActive(true);
        m_canMove = true;
        
        
    }

    public void StopMove()
    {
        m_canMove = false;
        this.gameObject.SetActive(false);
    }

    //public void Preview(GameObject key)
    //{
    //    this.gameObject.SetActive(true);
    //    isPreview = true;
    //    m_canMove = false;
    //    this.gameObject.GetComponent<Renderer>().material = m_transparentMaterial;
    //    previewKey = key;

    //    this.transform.position = previewKey.transform.position;
    //    this.transform.localRotation = previewKey.transform.rotation;
    //}

    //public void StopPreview()
    //{
    //    this.gameObject.SetActive(false);
    //    isPreview = false;
    //    previewKey = null;
    //}
}
