using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SG;


public class getRightAngles : MonoBehaviour
{
    //private Vector3[][] jointAngles;
    //float[] indexFlexR;
    //double R2D = 180/Mathf.PI;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
        //indexFlex = sums jointAngles 0-2 that belong to finger 1 (index)
        // for (int i=0; i>=2; i++)
        // {
        //     indexFlexR += fingerAngles.jointAngles[1][i];
        // }
        //translates index flexion to degrees
        //double totalFlex = indexFlexR;

        if (Input.GetKeyDown("a"))
        {
            Debug.Log("Sum flexion of index: ");
            //Debug.Log("Total flexion of index 'in degrees': "+totalFlex);
            //Debug.Log("fingerFlexion: "+fingerAngles[0][1]);
        }
    }
}
