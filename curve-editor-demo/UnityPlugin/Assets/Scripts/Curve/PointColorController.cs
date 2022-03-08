using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointColorController : MonoBehaviour
{
    private Color m_initialColor;
    private Color m_defaultColor;
    private Renderer m_renderer;

    private Color m_selectedColor = Color.red;
    private Color m_rotationKeyColor = Color.yellow;
    // Start is called before the first frame update
    void Start()
    {
        m_renderer = this.GetComponent<Renderer>();
        m_initialColor = m_renderer.material.color;
        m_defaultColor = m_initialColor;       
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnMouseOver()
    {
        m_renderer.material.color = m_selectedColor;
    }

    private void OnMouseExit()
    {
        m_renderer.material.color = m_defaultColor;
    }

    public void SetRotationKey(bool isRotationKey)
    {
        m_defaultColor = isRotationKey ? m_rotationKeyColor : m_initialColor;
        m_renderer.material.color = m_defaultColor;
    }
}
