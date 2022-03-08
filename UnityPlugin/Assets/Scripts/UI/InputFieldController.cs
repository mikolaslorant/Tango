using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InputFieldController : MonoBehaviour
{
    private TMPro.TMP_InputField m_inputField;
    public float step = 10;
    // Start is called before the first frame update
    void Start()
    {
        m_inputField = this.gameObject.GetComponent<TMPro.TMP_InputField>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void AddNumber()
    {
        float value = float.Parse(m_inputField.text) + step;
        m_inputField.text = value.ToString();
    }

    public void MinusNumber()
    {
        float value = float.Parse(m_inputField.text) - step;
        m_inputField.text = value.ToString();
    }

}
