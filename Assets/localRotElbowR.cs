using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class localRotElbowR : MonoBehaviour
{
    float localX=0;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetKeyDown("u"))
        {
            Debug.Log("Eje X manipulado por euler angles: "+localX);
            Debug.Log(""
                +" X: "+transform.localEulerAngles.x
                +" Y: "+transform.localEulerAngles.y
                +" Z: "+transform.localEulerAngles.z
            );
        }
    }
}

