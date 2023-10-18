using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HTC.UnityPlugin.Vive;
using HTC.UnityPlugin.Utility;

//region Networking libraries
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
//using System.IO.Ports;
using System.Threading;


public class localPosRight : MonoBehaviour
{
    bool sendData = false;
    bool initialTrackersRecorded = false;
    RigidPose initialTrackerChest;
    RigidPose rawInitialTrackerChest;
    RigidPose rawTrackerChest;
    RigidPose trackerChest;
    Vector3 shiftChest = new Vector3(0,0,0);
    RigidPose rawChest;
    RigidPose chest;
    RigidPose chestTrans;
    Vector3 fEulerChest;
    Vector3 eulerChest;
    string cmdChest;
    Vector3 eulerHead;
    Vector3 fEulerHead;
    RigidPose initialTrackerHead;
    RigidPose trackerHead;
    Vector3 shiftHead = new Vector3(0,0,0);
    RigidPose head;
    RigidPose initialTrackerRight;
    RigidPose trackerRight;
    // RigidPose initialTrackerRelbow;
    RigidPose initialTrackerLeft;  
    // RigidPose trackerRelbow;
    //RigidPose trackerLeft;
    Vector3 shiftRight = new Vector3(0,0,0);
   // Vector3 shiftLeft = new Vector3(0,0,0);
    RigidPose right;
    // RigidPose Relbow;
    //RigidPose left;
    // Start is called before the first frame update
    //private float[] leftPos;
    Vector3 rightPos;
    static Vector3 handPos;
    //define a client
    //UdpClient client = new UdpClient();
    RigidPose rightTrans;
    // RigidPose RelbowTrans;
    Vector3 Scale= new Vector3(1,1,1);
    UdpClient clientHead = new UdpClient();//create a client
    UdpClient clientChest = new UdpClient();//create a client
    byte[] bytesent;
    string cmdHead;
    string cmdRShoulder;
    string cmdLShoulder;

    // Start is called before the first frame update
    //public float [] rightPos;
   // public Vector3 rightPos;
    void Start()
    {
        //rotAxisChest.y  = -rotAxisChest.y;
        // rotAxisRight.z = -rotAxisRight.z;
        clientHead.Connect(new IPEndPoint(IPAddress.Parse("10.0.0.20"),9003)); //connect to client on first frame
        clientChest.Connect(new IPEndPoint(IPAddress.Parse("10.0.0.20"),9004));
        //client.Connect(new IPEndPoint(IPAddress.Parse("10.0.0.10"),9000)); //connect to client on first frame
    }

    public static Vector3 getRightPosition() 
    {
        return new Vector3(handPos.z, handPos.x, -handPos.y);
    }

  

    // Update is called once per frame
    void Update()
    {
        rightPos.x = transform.localPosition.z;
        rightPos.y = transform.localPosition.x;
        rightPos.z = -transform.localPosition.y; 

        rawTrackerChest = VivePose.GetPoseEx(TrackerRole.Tracker1);
        trackerHead = VivePose.GetPoseEx(DeviceRole.Hmd);
        // rightTrans = VivePose.GetPoseEx(TrackerRole.Tracker2);
        // RelbowTrans = VivePose.GetPoseEx(TrackerRole.Tracker3);

        chestTrans = rawTrackerChest;

        trackerChest.rot.x = chestTrans.rot.x;
        trackerChest.rot.y = chestTrans.rot.z;      //first change (negated*1 y)second change (negated*2 y)
        trackerChest.rot.z = chestTrans.rot.y;      //fifth change only chest.z and right.z are negated
        trackerChest.rot.w = chestTrans.rot.w;

        //trackerRight.rot.x = rightTrans.rot.x;
        //trackerRight.rot.y = rightTrans.rot.z;
        //trackerRight.rot.z = rightTrans.rot.y;
        //trackerRight.rot.w = rightTrans.rot.w;

        // trackerRight.rot.x = -(rightTrans.rot.z);  //fourth change no one is negated
        // trackerRight.rot.y = rightTrans.rot.x;      //first change (negated y)second change (negated*2 y)third change negated y and z on both 
        // trackerRight.rot.z = -(rightTrans.rot.y);
        // trackerRight.rot.w = rightTrans.rot.w;
        
        // trackerRelbow.rot.x = -(RelbowTrans.rot.z);  //fourth change no one is negated
        // trackerRelbow.rot.y = RelbowTrans.rot.x;      //first change (negated y)second change (negated*2 y)third change negated y and z on both 
        // trackerRelbow.rot.z = -(RelbowTrans.rot.y);
        // trackerRelbow.rot.w = RelbowTrans.rot.w;
        
        
        // handPos = transform.localPosition;
       /*  if(Input.GetKeyDown("1"))
        {           
            //transform.localRotation = Quaternion.identity;
            //originChestRight = VivePose.GetPoseEx(TrackerRole.Tracker1).pos;
            //localPos = transform.position;
            //Debug.Log("LOCAL ROT: "+transform.localRotation);
            //Debug.Log("Right hand position: "+localPos);
            Debug.Log("Right hand 'local' position in x: "+transform.localPosition.z);
            Debug.Log("Right hand 'local' position in y: "+transform.localPosition.x);
            Debug.Log("Right hand 'local' position in z: "+-transform.localPosition.y);
        } */
        // rightPos.x = transform.localPosition.x;
        // rightPos.y = -(transform.localPosition.y);
        // rightPos.z = transform.localPosition.z;  
        if(Input.GetKeyDown("p")){
            sendData = false;
        }
        if(Input.GetKeyDown("y")){
            // Debug.Log(cmdChest);
            Debug.Log(cmdHead);
        }
        if(Input.GetKeyDown("k")){
            // send the kill message
            bytesent = Encoding.ASCII.GetBytes("<1.8.5.1>");
            clientHead.Send(bytesent,bytesent.Length);
            Debug.Log("Killed");
        }
        if(Input.GetKeyDown("o")){
            Debug.Log("Calibrated with current pose.");
            if(VivePose.IsValidEx(TrackerRole.Tracker1) && VivePose.IsValidEx(DeviceRole.Hmd))// && VivePose.IsValidEx(TrackerRole.Tracker2))// && VivePose.IsValidEx(TrackerRole.Tracker3))
            { 
                initialTrackerChest = trackerChest;
                rawInitialTrackerChest = rawTrackerChest;
                //initialTrackerChest.rot = initialTrackerChest.rot;// * rotAxisChest;
                initialTrackerHead = trackerHead;
                initialTrackerHead.rot = Quaternion.Inverse(initialTrackerHead.rot) * rawInitialTrackerChest.rot;
                // initialTrackerRight = trackerRight;
                // initialTrackerRight.rot = Quaternion.Inverse(initialTrackerRight.rot) * initialTrackerChest.rot;// * rotAxisRight;
                // initialTrackerLeft = trackerLeft;
                // initialTrackerLeft.rot = Quaternion.Inverse(initialTrackerLeft.rot) * initialTrackerChest.rot;
                // initialTrackerRelbow = trackerRelbow;
                // initialTrackerRelbow.rot = Quaternion.Inverse(initialTrackerRelbow.rot)*initialTrackerChest.rot;
                initialTrackersRecorded = true;
                sendData = true;
            }
        }

     if(sendData){
     //   if(Input.GetKeyDown("r")){ //sends a matrix message over UDP
            //Debug.Log("TestR");
            if(initialTrackersRecorded && VivePose.IsValidEx(TrackerRole.Tracker1) && VivePose.IsValidEx(DeviceRole.Hmd))// && VivePose.IsValidEx(TrackerRole.Tracker2))// && VivePose.IsValidEx(TrackerRole.Tracker3))
            {
                rawChest.rot = Quaternion.Inverse(rawTrackerChest.rot) * rawInitialTrackerChest.rot;// * rotAxisChest;
                chest.rot = Quaternion.Inverse(trackerChest.rot) * initialTrackerChest.rot;// * rotAxisChest;
                head.rot = trackerHead.rot * initialTrackerHead.rot;
                head.rot = Quaternion.Inverse(head.rot) * rawTrackerChest.rot;
                right.rot = trackerRight.rot * initialTrackerRight.rot;
                right.rot = Quaternion.Inverse(right.rot) * trackerChest.rot;// * rotAxisRight;
                
// /* uncomment to enable head-torso (BEGIN) Maybe integrate it in localRot.cs for better resources

                fEulerChest = rawChest.rot.eulerAngles;
                fEulerHead = head.rot.eulerAngles;

                eulerChest.x = Convert.ToInt16(fEulerChest.x);
                eulerChest.y = Convert.ToInt16(fEulerChest.y);
                
                eulerHead.x = Convert.ToInt16(fEulerHead.x);
                eulerHead.y = Convert.ToInt16(fEulerHead.y);
                eulerHead.z = Convert.ToInt16(fEulerHead.z);

                // x es si
                // y es no
                // z es meh
                // Conversion de 0:360 a -180:180

                if(eulerChest.x>180){
                    eulerChest.x -= 360;
                }
                if(eulerChest.y>180){
                    eulerChest.y -= 360;
                }
                if(eulerChest.z>180){
                    eulerChest.z -= 360;
                }
                if(eulerHead.x>180){
                    eulerHead.x -= 360;
                }
                if(eulerHead.y>180){
                    eulerHead.y -= 360;
                }
                if(eulerHead.z>180){
                    eulerHead.z -= 360;
                }
                eulerHead.x = -eulerHead.x;
                // Conversion de limites
                if(eulerChest.x < -6){
                    eulerChest.x = -6;
                }
                if(eulerChest.x > 40){
                    eulerChest.x = 40;
                }
                if(eulerChest.y < -90){
                    eulerChest.y = -90;
                }
                if(eulerChest.y > 90){
                    eulerChest.y = 90;
                }

                if(eulerHead.x < -20){
                    eulerHead.x = -20;
                }
                if(eulerHead.x > 60){
                    eulerHead.x = 60;
                }
                if(eulerHead.y < -80){
                    eulerHead.y = 80;
                }
                if(eulerHead.y > 80){
                    eulerHead.y = 80;
                }
                if(eulerHead.z < -10){
                    eulerHead.z = -10;
                }
                if(eulerHead.z > 10){
                    eulerHead.z = 10;
                }
                // si.no (x.y)
                cmdChest = "<1.6.1."+eulerChest.x+"."+eulerChest.y+">";
                // no.si.meh (y.x.z)
                cmdHead = "<1.7.1."+eulerHead.y+"."+eulerHead.x+"."+eulerHead.z+">";

                // Debug.Log(cmdChest+"\n"+cmdHead);
                if (cmdChest!=null)
                {
                    // send the chest message
                    bytesent = Encoding.ASCII.GetBytes(cmdChest);
                    clientChest.Send(bytesent,bytesent.Length);
                    Debug.Log("Message "+cmdChest+" sent with success");
                }
                if (cmdHead!=null)
                {
                    // send the head message
                    bytesent = Encoding.ASCII.GetBytes(cmdHead);
                    clientHead.Send(bytesent,bytesent.Length);
                    Debug.Log("Message "+cmdHead+" sent with success");
                }
// uncomment to enable head-torso (END) */ 

                
                // Matrix4x4 mR = Matrix4x4.Rotate(right.rot);
                // Matrix4x4 m = Matrix4x4.TRS(rightPos,right.rot,Scale);
                // Matrix4x4 mRE = Matrix4x4.Rotate(Relbow.rot);
                
                //Debug.Log("Matrix: \n"+mR);
                //Debug.Log("matrix\n"+m);
                
               
                // +mR[0,0]+"\t"+(-mR[0,1])+"\t"+mR[0,2]+"\t"+mR[0,3]+"\n"
                // +(-mR[1,0])+"\t"+mR[1,1]+"\t"+(-mR[1,2])+"\t"+mR[1,3]+"\n"
                // +mR[2,0]+"\t"+(-mR[2,1])+"\t"+mR[2,2]+"\t"+mR[2,3]+"\n"
                // +mR[3,0]+"\t"+mR[3,1]+"\t"+mR[3,2]+"\t"+mR[3,3]+"\n");  
                
                /*
                Debug.Log("Rotation Matrix Elbow: \n"
                 
                 +mRE[0,0]+"\t"+(-mRE[0,1])+"\t"+mRE[0,2]+"\t"+mRE[0,3]+"\n"
                +(-mRE[1,0])+"\t"+mRE[1,1]+"\t"+(-mRE[1,2])+"\t"+mRE[1,3]+"\n"
                +mRE[2,0]+"\t"+(-mRE[2,1])+"\t"+mRE[2,2]+"\t"+mRE[2,3]+"\n"
                +mRE[3,0]+"\t"+mRE[3,1]+"\t"+mRE[3,2]+"\t"+mRE[3,3]+"\n");*/
                
                // string matrixMessage = ( //TRS matrix UDP message 3*4

                // +m[0,0]     +","   +(-m[0,1])  +","   +m[0,2]     +","+  m[0,3]*1000+","
                // +(-m[1,0])  +","   +m[1,1]     +","   +(-m[1,2])  +","+  m[1,3]*1000+","
                // +m[2,0]     +","   +(-m[2,1])  +","   +m[2,2]     +","+  m[2,3]*1000
                //+m[3,0]     +","   +m[3,1]     +","   +m[3,2]     +","+  m[3,3]+","
//uncomment to enable elbow rotation+","+(trackerRelbow.rot.z)
                // );    
                
            /* if (matrixMessage!=null)
                 {
                     byte[] bytesent = Encoding.ASCII.GetBytes(matrixMessage); //probablemente se deba cambiar el nombre a otra variable (por ejemplo byte[] matrixByteSent) para no generar conflicto entre todos los mensajes 
                     client.Send(bytesent,bytesent.Length);
                     Debug.Log("Message "+matrixMessage+" sent with success");
                 }*/
            // Debug.Log("Matrix TRS:\n"

            // +m[0,0]     +"\t"   +(m[0,1])  +"\t"   +m[0,2]     +"\t"+  m[0,3]+"\n"
            // +(m[1,0])  +"\t"   +m[1,1]     +"\t"   +(m[1,2])  +"\t"+  m[1,3]+"\n"
            // +m[2,0]     +"\t"   +(m[2,1])  +"\t"   +m[2,2]     +"\t"+  m[2,3]+"\n"
            // +m[3,0]     +"\t"   +m[3,1]     +"\t"   +m[3,2]     +"\t"+  m[3,3]+"\n");
            }    
                
        }

    }
}    
        


