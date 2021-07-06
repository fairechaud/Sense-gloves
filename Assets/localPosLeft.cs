using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HTC.UnityPlugin.Vive;
using HTC.UnityPlugin.VRModuleManagement;

public class localPosLeft : MonoBehaviour
{
    // Start is called before the first frame update
    public float[] leftPos;
    public Vector3 translation;
    public Vector3 eulerAngles;
    public Vector3 scale = new Vector3(1,1,1);



    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetKeyDown("0"))
        {
            
            //localPos = transform.position;
            //Debug.Log("LOCAL ROT: "+transform.localRotation);
            //Debug.Log("Right hand position: "+localPos);
            Debug.Log("Right hand 'local' position in x: "+transform.localPosition.x);
            Debug.Log("Right hand 'local' position in y: "+-(transform.localPosition.y));
            Debug.Log("Right hand 'local' position in z: "+transform.localPosition.z);        
        }
        // leftPos [0] = transform.localPosition.x;
        // leftPos [1] = -(transform.localPosition.y);
        // leftPos [2] = transform.localPosition.z;
    }
}
